"""Organic woodland replacement, built entirely from local procedural geometry.

Import build_environment(scene) instead of woodland_environment.build_environment.
Skip the caller's extra background-tree / grass duplication: this module already
builds three depth layers. Reachable visible meshes are included in sample_objects,
with no modifiers, displacement-only silhouettes or billboard trees. Real PBR
maps can be supplied through apply_pbr_textures after geometry is built.
No cameras, lights, original project files or GPU renders are changed here.
"""

import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent))

import math
import random
import sys
from pathlib import Path

import bpy
from mathutils import Vector

sys.path.insert(0, str(Path(__file__).resolve().parent))
from woodland_environment import _collection, _mesh_object, _leaf, _leaf_material, _tube, _tuft_patch, _shrub


def _smooth(value):
    value = min(1.0, max(0.0, value))
    return value * value * (3 - 2 * value)


def _terrain_z(x, y):
    """Actual mesh relief; central flight area stays flat for existing foot height."""
    clearance = _smooth((max(abs(x), abs(y)) - 1.65) / 2.1)
    wave = (.095 * math.sin(x * .31 + math.sin(y * .24))
            + .065 * math.sin(y * .43 + x * .17)
            + .024 * math.sin(x * 1.47 + y * .66) * math.sin(y * 1.15)
            + .012 * math.sin(x * 3.17 - y * 2.41))
    distant_bank = .30 * _smooth((math.hypot(x, y) - 18) / 32)
    return clearance * (wave + distant_bank)


def _ramp(nodes, colors):
    ramp = nodes.new("ShaderNodeValToRGB")
    elements = ramp.color_ramp.elements
    elements[0].position, elements[0].color = colors[0][0], (*colors[0][1], 1)
    elements[1].position, elements[1].color = colors[-1][0], (*colors[-1][1], 1)
    for position, color in colors[1:-1]:
        elements.new(position).color = (*color, 1)
    return ramp


def _surface_material(name, colors, scale, bump_distance, stretch=None):
    mat = bpy.data.materials.new(name)
    mat.use_nodes = True
    nodes, links = mat.node_tree.nodes, mat.node_tree.links
    shader = nodes.get("Principled BSDF")
    shader.inputs["Roughness"].default_value = .88
    shader.inputs["Specular IOR Level"].default_value = .27
    coordinates = nodes.new("ShaderNodeTexCoord")
    vector = coordinates.outputs["Object"]
    if stretch:
        mapping = nodes.new("ShaderNodeVectorMath")
        mapping.operation = "MULTIPLY"
        mapping.inputs[1].default_value = stretch
        links.new(vector, mapping.inputs[0])
        vector = mapping.outputs[0]
    macro = nodes.new("ShaderNodeTexNoise")
    macro.inputs["Scale"].default_value = scale
    macro.inputs["Detail"].default_value = 5
    macro.inputs["Roughness"].default_value = .72
    links.new(vector, macro.inputs["Vector"])
    ramp = _ramp(nodes, colors)
    links.new(macro.outputs["Fac"], ramp.inputs["Fac"])
    links.new(ramp.outputs["Color"], shader.inputs["Base Color"])
    fine = nodes.new("ShaderNodeTexNoise")
    fine.inputs["Scale"].default_value = 90 if "Bark" in name else 135
    fine.inputs["Detail"].default_value = 3
    links.new(vector, fine.inputs["Vector"])
    micro_bump = nodes.new("ShaderNodeBump")
    micro_bump.inputs["Distance"].default_value = bump_distance * .17
    micro_bump.inputs["Strength"].default_value = .65
    links.new(fine.outputs["Fac"], micro_bump.inputs["Height"])
    macro_bump = nodes.new("ShaderNodeBump")
    macro_bump.inputs["Distance"].default_value = bump_distance
    macro_bump.inputs["Strength"].default_value = .72
    links.new(macro.outputs["Fac"], macro_bump.inputs["Height"])
    links.new(micro_bump.outputs["Normal"], macro_bump.inputs["Normal"])
    links.new(macro_bump.outputs["Normal"], shader.inputs["Normal"])
    return mat


def _curved_spine(points, radii, subdivisions=4):
    """Centrally tangent cubic sections avoid rigid straight branch elbows."""
    points = [Vector(point) for point in points]
    curved_points, curved_radii = [], []
    for index in range(len(points) - 1):
        p0 = points[max(index - 1, 0)]
        p1, p2 = points[index:index + 2]
        p3 = points[min(index + 2, len(points) - 1)]
        for step in range(subdivisions):
            t = step / subdivisions
            point = .5 * ((2 * p1) + (-p0 + p2) * t
                     + (2 * p0 - 5 * p1 + 4 * p2 - p3) * t * t
                     + (-p0 + 3 * p1 - 3 * p2 + p3) * t * t * t)
            curved_points.append(point)
            curved_radii.append(radii[index] + (radii[index + 1] - radii[index]) * t)
    curved_points.append(points[-1])
    curved_radii.append(radii[-1])
    return curved_points, curved_radii


def _rough_tube(vertices, faces, mids, points, radii, sides, material_id=0, uv_faces=None):
    points, radii = _curved_spine(points, radii, 3 if len(points) > 8 else 4)
    start = len(vertices)
    face_start = len(faces)
    _tube(vertices, faces, mids, points, radii, sides, material_id)
    # Physical bark ridges make trunks less smooth even in silhouette. This is
    # mesh relief, so the BVH and rendered tree use the same exterior surface.
    for ring, (center, radius) in enumerate(zip(points, radii)):
        center = Vector(center)
        for index in range(sides):
            vertex_index = start + ring * sides + index
            radial = Vector(vertices[vertex_index]) - center
            ridge = .09 * math.sin(index * 2.73 + ring * .16)
            vertices[vertex_index] = tuple(center + radial * (1 + ridge))
    if uv_faces is not None:
        distances = [0.0]
        for a, b in zip(points, points[1:]):
            distances.append(distances[-1] + (b - a).length)
        for ring in range(len(points) - 1):
            for index in range(sides):
                # Seam vertices receive separate loop UVs: u wraps only across
                # the final circumferential face, without smearing the whole map.
                u0 = math.tau * radii[ring] * index / sides
                u1 = math.tau * radii[ring] * (index + 1) / sides
                u2 = math.tau * radii[ring + 1] * (index + 1) / sides
                u3 = math.tau * radii[ring + 1] * index / sides
                uv_faces[face_start + ring * sides + index] = [
                    (u0, distances[ring]), (u1, distances[ring]),
                    (u2, distances[ring + 1]), (u3, distances[ring + 1])]


def _tree(seed, quality, col, bark, leaf_mats):
    rng = random.Random(seed)
    vertices, faces, ids = [], [], []
    tube_uv_faces = {}
    height = rng.uniform(6.0, 9.0)
    radius = rng.uniform(.15, .27)
    lean = Vector((rng.uniform(-.43, .43), rng.uniform(-.43, .43), 0))
    trunk_points = []
    trunk_radii = []
    for index in range(15):
        u = index / 14
        trunk_points.append(lean * u + Vector((.055 * math.sin(u * 8 + seed),
                                               .045 * math.sin(u * 6 + seed * .3),
                                               height * .77 * u)))
        trunk_radii.append(radius * (1 - u) ** .85 + .018)
    _rough_tube(vertices, faces, ids, trunk_points, trunk_radii, 17, uv_faces=tube_uv_faces)
    # Irregular root flares, short buttresses and side roots, not a repeated star.
    for index in range(rng.randrange(4, 8)):
        angle = rng.uniform(0, math.tau)
        reach = rng.uniform(.35, .85)
        pts = [Vector((0, 0, .25)), Vector((math.cos(angle) * reach * .35,
               math.sin(angle) * reach * .35, .09)),
               Vector((math.cos(angle) * reach, math.sin(angle) * reach, .005))]
        _rough_tube(vertices, faces, ids, pts, [radius * .58, radius * .38, .01], 9,
                    uv_faces=tube_uv_faces)
    stems = []
    # Unequal leaders and asymmetric lateral limbs produce distinct crowns.
    for index in range(rng.randrange(3, 5)):
        angle = rng.uniform(0, math.tau)
        start = trunk_points[rng.randrange(7, 11)]
        crown = Vector((math.cos(angle) * rng.uniform(.65, 1.25),
                        math.sin(angle) * rng.uniform(.65, 1.25),
                        height * rng.uniform(.90, 1.08))) + lean
        bend = start.lerp(crown, .46) + Vector((rng.uniform(-.24, .24),
                                               rng.uniform(-.24, .24), -.13))
        _rough_tube(vertices, faces, ids, [start, bend, crown],
                    [radius * .48, radius * .25, .009], 11, uv_faces=tube_uv_faces)
        stems.append((bend, crown))
    for index in range(10):
        angle = rng.uniform(0, math.tau)
        start = trunk_points[rng.randrange(5, 13)]
        reach = rng.uniform(.8, 1.9) * (1.1 - start.z / height * .40)
        end = start + Vector((math.cos(angle) * reach, math.sin(angle) * reach,
                              rng.uniform(.3, 1.4)))
        bend = start.lerp(end, .46) + Vector((0, 0, -.10))
        _rough_tube(vertices, faces, ids, [start, bend, end],
                    [rng.uniform(.04, .085), .025, .003], 9, uv_faces=tube_uv_faces)
        stems.append((bend, end))
    leaf_budget = {"near": 5800, "middle": 2800, "far": 1000}[quality]
    leaves_per_tip = max(10, leaf_budget // (len(stems) * 5))
    for stem_start, stem_end in stems:
        main_direction = (stem_end - stem_start).normalized()
        for branch_index in range(5):
            angle = rng.uniform(0, math.tau)
            start = stem_start.lerp(stem_end, rng.uniform(.20, .95))
            end = start + main_direction * rng.uniform(.25, .7) + Vector((
                math.cos(angle) * rng.uniform(.22, .64),
                math.sin(angle) * rng.uniform(.22, .64), rng.uniform(-.12, .38)))
            midpoint = start.lerp(end, .54) + Vector((0, 0, rng.uniform(-.06, .08)))
            _rough_tube(vertices, faces, ids, [start, midpoint, end],
                        [.012, .006, .0014], 5, uv_faces=tube_uv_faces)
            # Leaf directions loosely follow the twigs, with uneven growth and
            # open gaps. Crown coverage comes from overlapping twig fans.
            for leaf_index in range(leaves_per_tip):
                u = rng.random() ** .53
                center = midpoint.lerp(end, u) + Vector((rng.gauss(0, .20),
                                      rng.gauss(0, .20), rng.gauss(0, .15)))
                azimuth = angle + rng.uniform(-1.9, 1.9)
                pitch = rng.uniform(-.70, .65)
                leaf_u = Vector((math.cos(azimuth) * math.cos(pitch),
                                 math.sin(azimuth) * math.cos(pitch), math.sin(pitch)))
                leaf_v = Vector((-math.sin(azimuth), math.cos(azimuth),
                                 rng.uniform(-.2, .2))).normalized()
                length = rng.uniform(.07, .16)
                _leaf(vertices, faces, ids, center, length, length * rng.uniform(.34, .58),
                      (leaf_u, leaf_v, leaf_u.cross(leaf_v).normalized()),
                      1 + rng.randrange(len(leaf_mats)))
    obj = _mesh_object("ENV2_TreePrototype_%s_%s" % (seed, quality), vertices,
                       faces, col, [bark] + leaf_mats, ids)
    _metre_uv(obj.data, tube_uv_faces)
    obj.hide_render = True
    obj.hide_set(True)
    return obj


def _rock(name, center, size, rng, col, material):
    # Irregular triangulated stone shell; no unapplied bevel or normal modifier.
    vertices = []
    sides = 9
    for ring in range(3):
        z = (-.4, .1, .58)[ring] * size
        spread = (.60, 1, .60)[ring] * size
        for index in range(sides):
            a = math.tau * index / sides
            vertices.append((math.cos(a) * spread * rng.uniform(.85, 1.12),
                             math.sin(a) * spread * rng.uniform(.75, 1.13),
                             z + rng.uniform(-.09, .09) * size))
    faces = [tuple(reversed(range(sides))), tuple(range(sides * 2, sides * 3))]
    for ring in range(2):
        for index in range(sides):
            nxt = (index + 1) % sides
            faces.extend([(ring * sides + index, ring * sides + nxt,
                           (ring + 1) * sides + nxt),
                          (ring * sides + index, (ring + 1) * sides + nxt,
                           (ring + 1) * sides + index)])
    obj = _mesh_object(name, vertices, faces, col, [material])
    obj.location = center
    return obj


def _metre_uv(mesh, tube_faces=None):
    """Assign planar metres per surface, suitable for tiled PBR normal maps."""
    layer = mesh.uv_layers.get("Surface metres")
    if layer is not None and tube_faces is None:
        return
    if layer is None:
        layer = mesh.uv_layers.new(name="Surface metres")
    for polygon in mesh.polygons:
        if tube_faces is not None and polygon.index in tube_faces:
            for loop_index, uv in zip(polygon.loop_indices, tube_faces[polygon.index]):
                layer.data[loop_index].uv = uv
            continue
        normal = polygon.normal
        axis = max(range(3), key=lambda index: abs(normal[index]))
        axes = ((1, 2), (0, 2), (0, 1))[axis]
        for loop_index in polygon.loop_indices:
            vertex = mesh.vertices[mesh.loops[loop_index].vertex_index].co
            layer.data[loop_index].uv = (vertex[axes[0]], vertex[axes[1]])


def apply_pbr_textures(environment, texture_assets, tile_sizes_m=None):
    """Use already-downloaded, licensed texture files; performs no downloads.

    texture_assets = {'ground': {'albedo': '/path/color.jpg',
       'roughness': '/path/rough.jpg', 'normal': '/path/normal.png',
       'height': '/path/height.png'}, 'bark': {...}, 'stone': {...}}
    Optional keys can be omitted. 'displacement' is accepted as a height alias.
    UV values are metres, repeated at tile_sizes_m={'ground': 2, 'bark': 1}.
    Height maps supply microscopic bump only, preserving final sampled geometry.
    """
    tile_sizes_m = tile_sizes_m or {"ground": 2.0, "bark": 1.0, "stone": 1.0}
    loaded = {}
    for target, paths in texture_assets.items():
        material = environment["materials"][target]
        nodes, links = material.node_tree.nodes, material.node_tree.links
        shader = nodes.get("Principled BSDF")
        coordinates = nodes.new("ShaderNodeTexCoord")
        mapping = nodes.new("ShaderNodeVectorMath")
        mapping.operation = "MULTIPLY"
        tile = tile_sizes_m.get(target, 1.0)
        mapping.inputs[1].default_value = (1 / tile, 1 / tile, 1 / tile)
        links.new(coordinates.outputs["UV"], mapping.inputs[0])
        outputs = {}
        loaded[target] = []
        for channel, filename in paths.items():
            if channel not in ("albedo", "roughness", "normal", "height", "displacement", "ao"):
                continue
            filename = Path(filename).resolve()
            if not filename.is_file():
                raise FileNotFoundError(filename)
            texture = nodes.new("ShaderNodeTexImage")
            texture.label = "%s %s" % (target, channel)
            texture.image = bpy.data.images.load(str(filename), check_existing=True)
            texture.image.colorspace_settings.name = "sRGB" if channel == "albedo" else "Non-Color"
            texture.extension = "REPEAT"
            links.new(mapping.outputs[0], texture.inputs["Vector"])
            outputs[channel] = texture.outputs["Color"]
            loaded[target].append(str(filename))
        if "albedo" in outputs:
            color = outputs["albedo"]
            if "ao" in outputs:
                multiply = nodes.new("ShaderNodeMixRGB")
                multiply.blend_type = "MULTIPLY"
                multiply.inputs[0].default_value = .55
                links.new(color, multiply.inputs[1])
                links.new(outputs["ao"], multiply.inputs[2])
                color = multiply.outputs[0]
            links.new(color, shader.inputs["Base Color"])
        if "roughness" in outputs:
            links.new(outputs["roughness"], shader.inputs["Roughness"])
        normal_output = None
        if "normal" in outputs:
            normal = nodes.new("ShaderNodeNormalMap")
            normal.uv_map = "Surface metres"
            normal.inputs["Strength"].default_value = .85
            links.new(outputs["normal"], normal.inputs["Color"])
            normal_output = normal.outputs["Normal"]
        height = outputs.get("height", outputs.get("displacement"))
        if height is not None:
            bump = nodes.new("ShaderNodeBump")
            bump.inputs["Strength"].default_value = .6
            bump.inputs["Distance"].default_value = .015 if target == "ground" else .01
            links.new(height, bump.inputs["Height"])
            if normal_output is not None:
                links.new(normal_output, bump.inputs["Normal"])
            normal_output = bump.outputs["Normal"]
        if normal_output is not None:
            links.new(normal_output, shader.inputs["Normal"])
    environment["external_assets"] = [filename for files in loaded.values() for filename in files]
    return loaded


def build_environment(scene, seed=20261011):
    """Same return contract as v1; caller controls cameras/lighting/rendering.

    All geometry is final mesh data. Rebuild the scene BVH and cloud JSON after
    switching this environment. Existing 2x2 m ENU route stays unchanged.
    """
    rng = random.Random(seed)
    ground_col = _collection("ENV2_Ground", scene)
    trees_col = _collection("ENV2_Trees", scene)
    details_col = _collection("ENV2_Details", scene)
    grass_col = _collection("ENV2_Grass", scene)
    prototypes_col = _collection("ENV2_Prototypes", scene)
    ground_material = _surface_material("ENV2_ForestFloor", [
        (.14, (.014, .009, .005)), (.39, (.054, .036, .015)),
        (.58, (.089, .086, .034)), (.73, (.061, .105, .021)),
        (.88, (.14, .12, .063))], .68, .014)
    bark = _surface_material("ENV2_FissuredBark", [
        (.18, (.025, .014, .008)), (.42, (.063, .040, .022)),
        (.64, (.15, .095, .051)), (.83, (.21, .17, .10))],
        7.5, .018, (3.8, 3.8, .19))
    rock_mat = _surface_material("ENV2_WeatheredStone", [
        (.2, (.10, .12, .095)), (.55, (.21, .23, .17)),
        (.8, (.32, .31, .24))], 5, .009)
    leaf_colors = [(.035, .083, .012), (.065, .14, .022),
                   (.094, .185, .033), (.13, .215, .045), (.09, .12, .02)]
    leaf_mats = [_leaf_material("ENV2_LiveLeaf_%s" % index, color)
                 for index, color in enumerate(leaf_colors)]
    grass_mats = [_leaf_material("ENV2_GrassMaterial_%s" % index, color)
                  for index, color in enumerate([(.050, .091, .012),
                                                 (.097, .15, .019), (.13, .18, .04)])]
    litter_mats = [_surface_material("ENV2_Litter_%d" % index,
                    [(0, tuple(v * .6 for v in color)), (1, color)], 22, .0007)
                    for index, color in enumerate([(.11, .048, .013),
                        (.18, .11, .037), (.27, .21, .095), (.065, .074, .017)])]

    # Subdivided, gently rolling earth. Object-scale procedural soil replaces
    # the uniform lawn material, and real leaf litter breaks the close silhouette.
    count, extent = 251, 56.25
    vertices = []
    for iy in range(count):
        y = -extent + 2 * extent * iy / (count - 1)
        for ix in range(count):
            x = -extent + 2 * extent * ix / (count - 1)
            vertices.append((x, y, _terrain_z(x, y)))
    faces = []
    for iy in range(count - 1):
        for ix in range(count - 1):
            start = iy * count + ix
            faces.append((start, start + 1, start + count + 1, start + count))
    ground = _mesh_object("ENV2_UndulatingForestGround", vertices, faces,
                          ground_col, [ground_material])
    sample_objects = [ground]

    # Eight independently generated near crowns; lower-density models are
    # deliberate real geometry for distant layers, shared by their scene copies.
    prototypes = {
        quality: [_tree(173 + index * 137, quality, prototypes_col, bark, leaf_mats)
                  for index in range(8 if quality == "near" else 5)]
        for quality in ("near", "middle", "far")
    }
    trees = []
    placements = []
    for quality, inner, outer, total, separation in (
            ("near", 4.7, 10, 14, 2.5),
            ("middle", 10, 23, 28, 3.1),
            ("far", 23, 45, 50, 3.7)):
        attempts = 0
        while sum(p[2] == quality for p in placements) < total:
            attempts += 1
            if attempts > 8000:
                break
            angle = rng.uniform(0, math.tau)
            radius = math.sqrt(rng.uniform(inner * inner, outer * outer))
            x, y = math.cos(angle) * radius, math.sin(angle) * radius
            if any((x - px) ** 2 + (y - py) ** 2 < separation ** 2
                   for px, py, _ in placements):
                continue
            placements.append((x, y, quality))
    for index, (x, y, quality) in enumerate(placements):
        prototype = prototypes[quality][index % len(prototypes[quality])]
        obj = bpy.data.objects.new("ENV2_%sTree_%03d" % (quality, index), prototype.data)
        trees_col.objects.link(obj)
        obj.location = (x, y, _terrain_z(x, y) - .035)
        obj.rotation_euler = (rng.uniform(-.035, .035), rng.uniform(-.035, .035),
                              rng.uniform(0, math.tau))
        scale = rng.uniform(.88, 1.18)
        obj.scale = (scale * rng.uniform(.88, 1.10),
                     scale * rng.uniform(.88, 1.10), scale * rng.uniform(.88, 1.14))
        trees.append(obj)
    sample_objects.extend(trees)

    grass_prototypes = [_tuft_patch(seed + 81 * index, prototypes_col, grass_mats)
                        for index in range(5)]
    for obj in grass_prototypes:
        obj.hide_render = True
        obj.hide_set(True)
    grasses = []
    attempts = 0
    while len(grasses) < 1350 and attempts < 9000:
        attempts += 1
        x, y = rng.uniform(-20, 20), rng.uniform(-20, 20)
        if max(abs(x), abs(y)) < 1.7 or math.hypot(x, y) > 22:
            continue
        density = .42 + .27 * math.sin(x * .83 + math.sin(y)) * math.sin(y * .71)
        if rng.random() > density:
            continue
        prototype = grass_prototypes[len(grasses) % len(grass_prototypes)]
        obj = bpy.data.objects.new("ENV2_GroundGrass_%04d" % len(grasses), prototype.data)
        grass_col.objects.link(obj)
        obj.location = (x, y, _terrain_z(x, y) - .01)
        obj.rotation_euler.z = rng.uniform(0, math.tau)
        obj.scale = (rng.uniform(.8, 1.75), rng.uniform(.75, 1.8), rng.uniform(.5, 1.12))
        grasses.append(obj)
    sample_objects.extend(grasses)

    shrubs = []
    for index in range(28):
        angle = rng.uniform(0, math.tau)
        radius = rng.uniform(3.6, 18)
        x, y = math.cos(angle) * radius, math.sin(angle) * radius
        obj = _shrub(seed + index * 171, x, y, details_col, bark, leaf_mats)
        obj.location.z = _terrain_z(x, y)
        scale = rng.uniform(.7, 1.8)
        obj.scale = (scale, scale, scale * rng.uniform(.65, 1.05))
        shrubs.append(obj)

    # Fallen twigs, small woody debris and stones add local scale and contact.
    for index in range(26):
        x, y = rng.uniform(-16, 16), rng.uniform(-16, 16)
        if max(abs(x), abs(y)) < 2.0:
            continue
        length = rng.uniform(.3, 1.25)
        angle = rng.uniform(0, math.tau)
        vertices, faces, ids = [], [], []
        points = [Vector((x, y, _terrain_z(x, y) + .04))]
        for u in (.4, 1):
            xx, yy = x + math.cos(angle) * length * u, y + math.sin(angle) * length * u
            points.append(Vector((xx, yy, _terrain_z(xx, yy) + .025)))
        _rough_tube(vertices, faces, ids, points, [.035, .026, .015], 9)
        _mesh_object("ENV2_FallenBranch_%03d" % index, vertices, faces,
                     details_col, [bark], ids)
    stones = []
    for index in range(90):
        x, y = rng.uniform(-17, 17), rng.uniform(-17, 17)
        if max(abs(x), abs(y)) < 1.8:
            continue
        size = rng.uniform(.025, .14)
        stones.append(_rock("ENV2_ForestStone_%03d" % index,
                            (x, y, _terrain_z(x, y) + size * .12),
                            size, rng, details_col, rock_mat))

    litter_vertices, litter_faces, litter_ids = [], [], []
    for index in range(6800):
        if index < 4200:
            tx, ty, _ = placements[rng.randrange(min(20, len(placements)))]
            x, y = tx + rng.gauss(0, 1.7), ty + rng.gauss(0, 1.7)
        else:
            x, y = rng.uniform(-18, 18), rng.uniform(-18, 18)
        if max(abs(x), abs(y)) < 1.6:
            continue
        azimuth = rng.uniform(0, math.tau)
        pitch = rng.uniform(-.16, .17)
        u = Vector((math.cos(azimuth), math.sin(azimuth), pitch)).normalized()
        v = Vector((-u.y, u.x, rng.uniform(-.06, .06))).normalized()
        _leaf(litter_vertices, litter_faces, litter_ids,
              Vector((x, y, _terrain_z(x, y) + .005)), rng.uniform(.055, .15),
              rng.uniform(.025, .065), (u, v, u.cross(v).normalized()),
              rng.randrange(len(litter_mats)))
    _mesh_object("ENV2_ScatteredForestLitter", litter_vertices, litter_faces,
                 details_col, litter_mats, litter_ids)
    sample_objects.extend(o for o in details_col.objects if o.type == "MESH")
    all_visible_meshes = sample_objects[:]
    # Shared meshes need UVs only once. Geometric relief remains actual geometry;
    # external PBR normals add sub-centimetre detail on these same surfaces.
    uv_targets = {obj.data.name: obj.data for obj in all_visible_meshes}
    for mesh in uv_targets.values():
        _metre_uv(mesh)
    bpy.context.view_layer.update()
    # Conservatively reject only meshes whose entire world bounding box lies
    # beyond the 18 m sensor reach from every point in a padded flight volume.
    # The visual far forest remains in the render, avoiding a costly huge BVH.
    sensor_min = Vector((-1.8, -1.8, -.05))
    sensor_max = Vector((1.8, 1.8, 2.7))
    def reachable(obj):
        corners = [obj.matrix_world @ Vector(corner) for corner in obj.bound_box]
        low = [min(point[axis] for point in corners) for axis in range(3)]
        high = [max(point[axis] for point in corners) for axis in range(3)]
        distance_squared = sum(max(0, low[axis] - sensor_max[axis],
                                   sensor_min[axis] - high[axis]) ** 2 for axis in range(3))
        return distance_squared <= 18 * 18
    sample_objects = [obj for obj in all_visible_meshes if reachable(obj)]
    mesh_triangle_counts = {obj.data.name: sum(len(p.vertices) - 2 for p in obj.data.polygons)
                            for obj in all_visible_meshes}
    triangle_count = sum(mesh_triangle_counts[obj.data.name] for obj in sample_objects)
    all_visible_triangles = sum(mesh_triangle_counts[obj.data.name] for obj in all_visible_meshes)
    return {
        "sample_objects": sample_objects, "ground": ground, "stones": stones,
        "trees": trees, "grass": grasses, "shrubs": shrubs,
        "collections": [ground_col, trees_col, details_col, grass_col],
        "prototypes_collection": prototypes_col, "estimated_triangles": triangle_count,
        "visible_mesh_objects": all_visible_meshes, "visible_triangles": all_visible_triangles,
        "materials": {"ground": ground_material, "bark": bark, "stone": rock_mat},
        "clear_flight_bounds_xy": [-1.65, 1.65, -1.65, 1.65], "stone_top_z": 0,
        "contains_background": True, "background_extent_m": 45,
        "uv_contract": "Surface metres: ground XY metres; bark ring circumference U and curved spine arc-length V; leaf/cap faces planar fallback",
        "sample_geometry_contract": "all visible final meshes reachable within 18 m from padded sensor volume; farther scene meshes are conservatively excluded by whole-object AABB",
        "external_assets": [],
        "lighting_recommendation": {"world_strength": .16, "sun_energy": 2.1,
             "sun_angle_degrees": .8, "camera_fill_watts": 25,
             "reason": "retain fine leaf shadows and depth; current 5-degree sun and 160 W camera fill flatten the forest"},
    }


if __name__ == "__main__":
    import json
    import time
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete(use_global=False)
    start = time.perf_counter()
    result = build_environment(bpy.context.scene)
    bpy.context.view_layer.update()
    assert all(obj.type == "MESH" and not obj.modifiers for obj in result["sample_objects"])
    assert all(_terrain_z(x, y) == 0 for x, y in ((0, 0), (-1.3, -1.3), (1.3, 1.3)))
    print("ENV2_BUILD_ONLY", json.dumps({
        "seconds": round(time.perf_counter() - start, 3),
        "sample_objects": len(result["sample_objects"]),
        "trees": len(result["trees"]), "grass": len(result["grass"]),
        "triangles": result["estimated_triangles"],
        "visible_triangles": result["visible_triangles"],
        "external_assets": result["external_assets"], "rendered": False,
    }), flush=True)
