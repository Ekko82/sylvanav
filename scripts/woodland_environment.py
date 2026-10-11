"""Procedural woodland courtyard for the Inspire showcase (Blender 4.5).

No external assets, animation, camera, or lights.  Call build_environment(scene)
once on an empty environment layer.  All generated objects begin with ENV_.
The returned `sample_objects` are real scene meshes for synthetic point sampling.
"""

import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent))

import bpy
import math
import random
from mathutils import Vector


def _collection(name, scene):
    col = bpy.data.collections.new(name)
    scene.collection.children.link(col)
    return col


def _material(name, color, roughness=.75, noise_scale=8, bump=.08):
    mat = bpy.data.materials.new(name)
    mat.use_nodes = True
    n, l = mat.node_tree.nodes, mat.node_tree.links
    p = n.get("Principled BSDF")
    p.inputs["Base Color"].default_value = (*color, 1)
    p.inputs["Roughness"].default_value = roughness
    tex = n.new("ShaderNodeTexNoise")
    tex.inputs["Scale"].default_value = noise_scale
    tex.inputs["Detail"].default_value = 4
    tex.inputs["Roughness"].default_value = .68
    coord = n.new("ShaderNodeTexCoord")
    l.new(coord.outputs["Generated"], tex.inputs["Vector"])
    ramp = n.new("ShaderNodeValToRGB")
    ramp.color_ramp.elements[0].position = .15
    ramp.color_ramp.elements[0].color = (*(max(.005, v * .68) for v in color), 1)
    ramp.color_ramp.elements[1].position = .85
    ramp.color_ramp.elements[1].color = (*(min(.85, v * 1.25) for v in color), 1)
    l.new(tex.outputs["Fac"], ramp.inputs["Fac"])
    l.new(ramp.outputs["Color"], p.inputs["Base Color"])
    if bump:
        b = n.new("ShaderNodeBump")
        b.inputs["Strength"].default_value = .4
        b.inputs["Distance"].default_value = bump
        l.new(tex.outputs["Fac"], b.inputs["Height"])
        l.new(b.outputs["Normal"], p.inputs["Normal"])
    return mat


def _leaf_material(name, color):
    mat = _material(name, color, .56, 11, .0003)
    n, l = mat.node_tree.nodes, mat.node_tree.links
    p = n.get("Principled BSDF")
    p.inputs["Subsurface Weight"].default_value = .045
    p.inputs["Subsurface Radius"].default_value = (.02, .04, .015)
    transluc = n.new("ShaderNodeBsdfTranslucent")
    transluc.inputs["Color"].default_value = (*color, 1)
    mix = n.new("ShaderNodeMixShader")
    mix.inputs[0].default_value = .17
    out = n.get("Material Output")
    l.new(p.outputs[0], mix.inputs[1])
    l.new(transluc.outputs[0], mix.inputs[2])
    l.new(mix.outputs[0], out.inputs["Surface"])
    return mat


def _mesh_object(name, vertices, faces, col, materials, material_ids=None):
    mesh = bpy.data.meshes.new(name + "_Mesh")
    mesh.from_pydata(vertices, [], faces)
    mesh.update()
    for mat in materials:
        mesh.materials.append(mat)
    obj = bpy.data.objects.new(name, mesh)
    col.objects.link(obj)
    for i, p in enumerate(mesh.polygons):
        p.material_index = material_ids[i] if material_ids else 0
        p.use_smooth = True
    return obj


def _tube(vertices, faces, mids, points, radii, sides=9, material_id=0):
    """Tapered branch mesh with slightly uneven bark cross sections."""
    start = len(vertices)
    for j, (pt, radius) in enumerate(zip(points, radii)):
        tangent = Vector(points[min(j + 1, len(points) - 1)]) - Vector(points[max(j - 1, 0)])
        if tangent.length < 1e-8:
            tangent = Vector((0, 0, 1))
        tangent.normalize()
        ref = Vector((0, 0, 1)) if abs(tangent.z) < .9 else Vector((1, 0, 0))
        u = tangent.cross(ref).normalized()
        v = tangent.cross(u).normalized()
        for k in range(sides):
            a = 2 * math.pi * k / sides
            uneven = 1 + .06 * math.sin(k * 2.1 + j * .65)
            co = Vector(pt) + (u * math.cos(a) + v * math.sin(a)) * radius * uneven
            vertices.append(tuple(co))
    for j in range(len(points) - 1):
        for k in range(sides):
            k1 = (k + 1) % sides
            faces.append((start + j * sides + k, start + j * sides + k1,
                          start + (j + 1) * sides + k1, start + (j + 1) * sides + k))
            mids.append(material_id)
    faces.append(tuple(start + k for k in reversed(range(sides))))
    mids.append(material_id)
    faces.append(tuple(start + (len(points) - 1) * sides + k for k in range(sides)))
    mids.append(material_id)


def _leaf(vertices, faces, mids, center, length, width, orientation, matid):
    """Pointed, curved elliptical leaf: six edges and a raised central vein."""
    u, v, normal = orientation
    start = len(vertices)
    shape = [(-.50, 0, 0), (-.23, .48, .035), (.20, .43, .02),
             (.50, 0, 0), (.20, -.43, .02), (-.23, -.48, .035), (0, 0, .08)]
    for x, y, z in shape:
        co = center + u * x * length + v * y * width + normal * z * length
        vertices.append(tuple(co))
    for k in range(6):
        faces.append((start + 6, start + k, start + (k + 1) % 6))
        mids.append(matid)


def _tree_variant(seed, col, bark, leaves):
    rng = random.Random(seed)
    verts, faces, mids = [], [], []
    height = rng.uniform(4.65, 5.15)
    # A bending trunk and real roots rather than a cylinder buried in the lawn.
    trunk = [(.035 * math.sin(i * .6), .026 * math.sin(i * .8), height * i / 9) for i in range(10)]
    _tube(verts, faces, mids, trunk, [.19 * (1 - i / 10) ** .8 + .012 for i in range(10)], 13)
    for k in range(7):
        a = k * 2 * math.pi / 7 + rng.uniform(-.12, .12)
        _tube(verts, faces, mids,
              [(0, 0, .22), (.30 * math.cos(a), .30 * math.sin(a), .045),
               (.65 * math.cos(a), .65 * math.sin(a), .01)], [.095, .065, .01], 9)
    clusters = []
    for k in range(11):
        a = k * 2.39996 + rng.uniform(-.25, .25)
        z = 1.6 + k * .23
        radius = (1.66 - .07 * k) * rng.uniform(.85, 1.12)
        p0 = Vector((.035 * math.sin(k), .025 * math.cos(k), z))
        p1 = Vector((math.cos(a) * radius * .44, math.sin(a) * radius * .44, z + .38))
        p2 = Vector((math.cos(a) * radius, math.sin(a) * radius, z + .76))
        _tube(verts, faces, mids, [p0, p1, p2], [.073 - k * .0038, .035, .009], 9)
        for q in range(3):
            angle = a + (q - 1) * .61 + rng.uniform(-.18, .18)
            base = p1.lerp(p2, .23 + q * .25)
            reach = rng.uniform(.46, .76)
            end = base + Vector((math.cos(angle) * reach, math.sin(angle) * reach, rng.uniform(.25, .60)))
            mid = base.lerp(end, .48) + Vector((0, 0, .06))
            _tube(verts, faces, mids, [base, mid, end], [.023, .011, .0027], 7)
            # Separate leaf clusters along branch tips make an irregular open crown.
            clusters.append((end, rng.uniform(.37, .58), rng.uniform(.36, .53), rng.uniform(.24, .42)))
            for t in (-1, 1):
                twig_end = mid + Vector((math.cos(angle + t * .67) * .3,
                                          math.sin(angle + t * .67) * .3, .21))
                _tube(verts, faces, mids, [mid, twig_end], [.007, .0017], 5)
    clusters.extend([(Vector((rng.uniform(-.6, .6), rng.uniform(-.6, .6), height - .15)), .48, .48, .35)
                     for _ in range(6)])
    for center, rx, ry, rz in clusters:
        for _ in range(250):
            # Bounded ellipsoid sample, no sphere/crown solid mesh.
            while True:
                dx, dy, dz = (rng.uniform(-1, 1) for _ in range(3))
                if dx * dx + dy * dy + dz * dz <= 1:
                    break
            c = center + Vector((dx * rx, dy * ry, dz * rz))
            a = rng.uniform(0, 2 * math.pi)
            pitch = rng.uniform(-.6, .6)
            u = Vector((math.cos(a) * math.cos(pitch), math.sin(a) * math.cos(pitch), math.sin(pitch)))
            v = Vector((-math.sin(a), math.cos(a), rng.uniform(-.2, .2))).normalized()
            normal = u.cross(v).normalized()
            _leaf(verts, faces, mids, c, rng.uniform(.065, .115), rng.uniform(.031, .060),
                  (u, v, normal), 1 + rng.randrange(len(leaves)))
    obj = _mesh_object("ENV_TreeVariant_%s" % seed, verts, faces, col, [bark] + leaves, mids)
    return obj


def _tuft_patch(seed, col, materials):
    rng = random.Random(seed)
    v, f, ids = [], [], []
    for _ in range(58):
        angle = rng.uniform(0, 2 * math.pi)
        radius = .28 * math.sqrt(rng.random())
        x, y = math.cos(angle) * radius, math.sin(angle) * radius
        for k in range(rng.randrange(4, 8)):
            a = rng.uniform(0, 2 * math.pi)
            height = rng.uniform(.055, .13)
            width = rng.uniform(.003, .006)
            bend = rng.uniform(.02, .07)
            u = Vector((math.cos(a), math.sin(a), 0))
            w = Vector((-u.y, u.x, 0)) * width
            p = Vector((x, y, .001))
            start = len(v)
            v.extend([tuple(p - w), tuple(p + w), tuple(p + u * bend * .45 + Vector((0, 0, height * .64)) + w * .55),
                      tuple(p + u * bend * .45 + Vector((0, 0, height * .64)) - w * .55),
                      tuple(p + u * bend + Vector((0, 0, height)))])
            f.extend([(start, start + 1, start + 2, start + 3), (start + 3, start + 2, start + 4)])
            ids.extend([rng.randrange(len(materials))] * 2)
    return _mesh_object("ENV_GrassPatch_%s" % seed, v, f, col, materials, ids)


def _shrub(seed, x, y, col, bark, leaves):
    rng = random.Random(seed)
    vertices, faces, mids = [], [], []
    for k in range(18):
        a = k * 2.39996
        end = Vector((rng.uniform(.17, .42) * math.cos(a),
                      rng.uniform(.17, .42) * math.sin(a), rng.uniform(.27, .58)))
        _tube(vertices, faces, mids, [(0, 0, 0), end * .43, end], [.011, .007, .0015], 6)
        for _ in range(40):
            center = end + Vector((rng.uniform(-.12, .12), rng.uniform(-.12, .12), rng.uniform(-.09, .10)))
            angle = rng.uniform(0, 2 * math.pi)
            u = Vector((math.cos(angle), math.sin(angle), rng.uniform(-.25, .25))).normalized()
            v = Vector((-u.y, u.x, .08)).normalized()
            _leaf(vertices, faces, mids, center, rng.uniform(.09, .15), rng.uniform(.03, .055),
                  (u, v, u.cross(v).normalized()), 1 + rng.randrange(len(leaves)))
    obj = _mesh_object("ENV_Shrub_%s" % seed, vertices, faces, col, [bark] + leaves, mids)
    obj.location = (x, y, .002)
    return obj


def _stone(name, x, y, width, depth, col, material, rng):
    n = 7
    verts = []
    for z in (.001, rng.uniform(.020, .028)):
        for k in range(n):
            a = 2 * math.pi * k / n
            verts.append((x + math.cos(a) * width * .51 * rng.uniform(.94, 1.02),
                          y + math.sin(a) * depth * .51 * rng.uniform(.94, 1.02), z))
    faces = [tuple(range(n - 1, -1, -1)), tuple(range(n, n * 2))]
    faces.extend([(k, (k + 1) % n, (k + 1) % n + n, k + n) for k in range(n)])
    obj = _mesh_object(name, verts, faces, col, [material])
    bevel = obj.modifiers.new("Soft stone edges", "BEVEL")
    bevel.width = .008
    bevel.segments = 2
    obj.modifiers.new("Weighted stone normals", "WEIGHTED_NORMAL")
    return obj


def _block(name, location, size, col, material, bevel=0):
    x, y, z = (s * .5 for s in size)
    vertices = [(-x, -y, -z), (x, -y, -z), (x, y, -z), (-x, y, -z),
                (-x, -y, z), (x, -y, z), (x, y, z), (-x, y, z)]
    faces = [(0, 3, 2, 1), (4, 5, 6, 7), (0, 1, 5, 4), (1, 2, 6, 5), (2, 3, 7, 6), (3, 0, 4, 7)]
    obj = _mesh_object(name, vertices, faces, col, [material])
    obj.location = location
    if bevel:
        m = obj.modifiers.new("Soft edges", "BEVEL")
        m.width, m.segments = bevel, 2
        obj.modifiers.new("Weighted normals", "WEIGHTED_NORMAL")
    return obj


def build_environment(scene):
    """Build environment once and return mesh objects plus named collections.

    Plane Z=0; stone top ~0.025m. Central flight square [-1.3,1.3]^2 has no
    tall objects. Forward camera side (negative Y) stays open. Lighting is
    deliberately left to the parent showcase script.
    """
    rng = random.Random(8241)
    ground_col = _collection("ENV_Ground", scene)
    trees_col = _collection("ENV_Trees", scene)
    details_col = _collection("ENV_Details", scene)
    grass_col = _collection("ENV_Grass", scene)
    prototypes_col = _collection("ENV_Prototypes", scene)
    soil = _material("ENV_LawnSoil", (.105, .137, .043), .91, 135, .012)
    stone_mats = [_material("ENV_Sandstone_%d" % i, c, .83, 6, .012) for i, c in enumerate([
        (.37, .38, .34), (.40, .39, .35), (.33, .35, .32), (.44, .43, .38)])]
    bark = _material("ENV_Bark", (.16, .115, .078), .91, 32, .018)
    leaf_mats = [_leaf_material("ENV_Leaf_%d" % i, c) for i, c in enumerate([
        (.063, .132, .021), (.091, .170, .027), (.124, .205, .036), (.15, .225, .042)])]
    grass_mats = [_material("ENV_Grass_%d" % i, c, .75, 5, .0001) for i, c in enumerate([
        (.11, .18, .027), (.17, .24, .042), (.075, .14, .02)])]
    wood = _material("ENV_WeatheredWood", (.24, .16, .093), .8, 16, .004)
    dark_metal = _material("ENV_BenchMetal", (.075, .08, .075), .47, 6, .0001)
    wall_mat = _material("ENV_Plaster", (.46, .43, .36), .85, 36, .01)
    ground = _mesh_object("ENV_GroundPlane", [(-35, -35, 0), (35, -35, 0), (35, 35, 0), (-35, 35, 0)],
                          [(0, 1, 2, 3)], ground_col, [soil])
    stones = []
    # Central, slightly irregular courtyard stones and a front entry path.
    for row in range(6):
        for k in range(6):
            x = (k - 2.5) * .405 + (.045 if row % 2 else -.045)
            y = (row - 2.5) * .415
            stones.append(_stone("ENV_CourtyardStone_%02d_%02d" % (row, k), x, y, .392, .403,
                                 ground_col, stone_mats[rng.randrange(4)], rng))
    for row in range(8):
        y = -1.40 - row * .42
        for k in range(2):
            stones.append(_stone("ENV_PathStone_%02d_%d" % (row, k), (k - .5) * .43, y, .42, .40,
                                 ground_col, stone_mats[rng.randrange(4)], rng))
    variants = [_tree_variant(seed, prototypes_col, bark, leaf_mats) for seed in (113, 227, 349)]
    for obj in variants:
        obj.hide_render = True
        obj.hide_set(True)
    trees = []
    positions = [(-3.6, 2.7, .90), (3.25, 2.4, .94), (-5.0, -1.1, .83),
                 (5.0, -.7, .92), (-5.9, 6.9, 1.15), (5.4, 7.3, 1.07),
                 (-1.5, 9.3, 1.15), (2.1, 11.3, 1.22)]
    for i, (x, y, scale) in enumerate(positions):
        proto = variants[i % 3]
        obj = bpy.data.objects.new("ENV_Tree_%02d" % i, proto.data)
        trees_col.objects.link(obj)
        obj.location = (x, y, 0)
        obj.rotation_euler.z = rng.uniform(0, 2 * math.pi)
        obj.scale = (scale, scale, scale * rng.uniform(.96, 1.08))
        trees.append(obj)
    patches = [_tuft_patch(s, prototypes_col, grass_mats) for s in (23, 47, 71)]
    for p in patches:
        p.hide_render = True
        p.hide_set(True)
    grasses = []
    for i in range(600):
        x, y = rng.uniform(-7.8, 7.8), rng.uniform(-5.6, 11.7)
        if abs(x) < 1.40 and abs(y) < 1.45:
            continue
        if abs(x) < .70 and -5.0 < y < -1.0:
            continue
        obj = bpy.data.objects.new("ENV_Grass_%03d" % i, patches[i % 3].data)
        grass_col.objects.link(obj)
        obj.location = (x, y, 0)
        obj.rotation_euler.z = rng.uniform(0, 2 * math.pi)
        s = rng.uniform(.63, 1.4)
        obj.scale = (s, s, s * rng.uniform(.7, 1.0))
        grasses.append(obj)
    shrubs = [_shrub(seed, x, y, details_col, bark, leaf_mats)
              for seed, x, y in [(515, -3.5, 5.1), (618, 4.6, 4.3), (729, 2.0, 10.9), (813, -6.0, 2.6)]]
    # A low plaster garden wall and a simple timber bench give distance cues.
    wall = _block("ENV_BackGardenWall", (0, 12.9, .47), (19, .20, .94), details_col, wall_mat, .018)
    _block("ENV_WallCap", (0, 12.9, .96), (19.1, .24, .09), details_col, stone_mats[1], .015)
    bench_parts = []
    bx, by = 3.8, 6.5
    for i in range(4):
        bench_parts.append(_block("ENV_BenchSeat_%d" % i, (bx, by + (i - 1.5) * .095, .43),
                                  (1.55, .085, .045), details_col, wood, .007))
    for i in range(3):
        bench_parts.append(_block("ENV_BenchBack_%d" % i, (bx, by + .22, .64 + i * .11),
                                  (1.55, .05, .075), details_col, wood, .007))
    for xoff in (-.57, .57):
        for yoff in (-.15, .15):
            bench_parts.append(_block("ENV_BenchLeg_%s_%s" % (xoff, yoff), (bx + xoff, by + yoff, .21),
                                      (.035, .035, .40), details_col, dark_metal, .004))
        bench_parts.append(_block("ENV_BenchSupport_%s" % xoff, (bx + xoff, by + .23, .64),
                                  (.035, .035, .65), details_col, dark_metal, .004))
    # Leaf litter at tree bases: restrained detail for foreground depth.
    litter_v, litter_f, litter_ids = [], [], []
    for tx, ty, _ in positions[:4]:
        for _ in range(85):
            a = rng.uniform(0, 2 * math.pi)
            rad = rng.uniform(.35, 1.25)
            c = Vector((tx + math.cos(a) * rad, ty + math.sin(a) * rad, .012))
            orient = rng.uniform(0, 2 * math.pi)
            u = Vector((math.cos(orient), math.sin(orient), 0))
            v = Vector((-u.y, u.x, 0))
            _leaf(litter_v, litter_f, litter_ids, c, rng.uniform(.05, .09), .035, (u, v, Vector((0, 0, 1))), 0)
    litter = _mesh_object("ENV_LeafLitter", litter_v, litter_f, details_col, [bark], litter_ids)
    sample = [ground] + stones + trees + grasses + [o for o in details_col.objects if o.type == 'MESH']
    triangles = sum(sum(len(p.vertices) - 2 for p in o.data.polygons) for o in sample)
    return {"sample_objects": sample, "ground": ground, "stones": stones, "trees": trees,
            "grass": grasses, "shrubs": shrubs, "collections": [ground_col, trees_col, details_col, grass_col],
            "prototypes_collection": prototypes_col, "estimated_triangles": triangles,
            "clear_flight_bounds_xy": [-1.3, 1.3, -1.3, 1.3], "stone_top_z": .028}


if __name__ == "__main__":
    # Standalone QA scene, deliberately separate from the parent's showcase.
    import json
    from pathlib import Path
    bpy.ops.object.select_all(action='SELECT')
    bpy.ops.object.delete(use_global=False)
    scene = bpy.context.scene
    result = build_environment(scene)
    camera_data = bpy.data.cameras.new("QA_Camera")
    camera = bpy.data.objects.new("QA_Camera", camera_data)
    scene.collection.objects.link(camera)
    camera.location = (-2.1, -3.0, 1.15)
    camera.rotation_euler = (Vector((0, .7, 1.05)) - camera.location).to_track_quat('-Z', 'Y').to_euler()
    camera_data.lens = 29
    scene.camera = camera
    sun_data = bpy.data.lights.new("QA_Sun", 'SUN')
    sun_data.energy = 2.0
    sun_data.angle = math.radians(6)
    sun = bpy.data.objects.new("QA_Sun", sun_data)
    scene.collection.objects.link(sun)
    sun.rotation_euler = (math.radians(31), math.radians(-24), math.radians(-26))
    scene.world.use_nodes = True
    scene.world.node_tree.nodes['Background'].inputs['Color'].default_value = (.65, .75, .88, 1)
    scene.world.node_tree.nodes['Background'].inputs['Strength'].default_value = .45
    scene.render.engine = 'CYCLES'
    scene.cycles.samples = 32
    scene.cycles.use_denoising = True
    scene.render.resolution_x = 1280
    scene.render.resolution_y = 720
    scene.render.resolution_percentage = 100
    scene.view_settings.view_transform = 'AgX'
    scene.view_settings.exposure = .4
    out = Path(__file__).resolve().parent
    scene.render.filepath = str(out / 'smoke_environment.png')
    bpy.ops.wm.save_as_mainfile(filepath=str(out / 'smoke_environment.blend'))
    report = {'estimated_triangles': result['estimated_triangles'], 'sample_objects': len(result['sample_objects']),
              'trees': len(result['trees']), 'stone_top_z': result['stone_top_z']}
    (out / 'smoke_environment_report.json').write_text(json.dumps(report, indent=2), 'utf-8')
    bpy.ops.render.render(write_still=True)
