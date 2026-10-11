"""Append real, merged understory meshes to an existing ENV2_ woodland.

Call ``append_understory(scene)`` after building the environment and before
building its BVH. The returned sample_objects are the exact visible meshes,
without instances, modifiers, alpha cards, or shader-only foliage. This module
does not save a blend, render, change lights/cameras, or remove existing plants.
Repeated calls return this scene's existing generated meshes without duplicates.
"""

import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent))

import json
import math
import random
import re
import sys
from pathlib import Path

import bpy
from mathutils import Vector

sys.path.insert(0, str(Path(__file__).resolve().parent))
from woodland_environment import _collection, _leaf, _leaf_material, _mesh_object, _tube
from woodland_environment_v2 import _terrain_z


GENERATOR = "append_understory_v1"
CLEAR_HALF_WIDTH = 1.7
TRIANGLE_LIMIT = 2_000_000


class _Geometry:
    def __init__(self):
        self.vertices = []
        self.faces = []
        self.material_ids = []

    @property
    def triangles(self):
        return sum(len(face) - 2 for face in self.faces)

    def make_object(self, name, collection, materials):
        # Entire clumps are placed beyond a conservative footprint margin;
        # check the final geometry too, so the flight-square guarantee is real.
        if any(abs(x) <= CLEAR_HALF_WIDTH and abs(y) <= CLEAR_HALF_WIDTH
               and z <= 2.7 for x, y, z in self.vertices):
            raise ValueError("Understory entered the protected flight volume")
        obj = _mesh_object(name, self.vertices, self.faces, collection,
                           materials, self.material_ids)
        obj["understory_generator"] = GENERATOR
        obj["triangle_count"] = self.triangles
        return obj


def _palette(existing_names, additions):
    materials = [bpy.data.materials[name] for name in existing_names
                 if name in bpy.data.materials]
    for name, color in additions:
        material = bpy.data.materials.get(name)
        if material is None:
            material = _leaf_material(name, color)
        materials.append(material)
    return materials


def _tree_anchors(scene):
    bpy.context.view_layer.update()
    return [(obj.matrix_world.translation.x, obj.matrix_world.translation.y)
            for obj in scene.objects
            if re.match(r"^ENV2_(near|middle|far)Tree_", obj.name)
            and obj.type == "MESH" and not obj.hide_render
            and math.hypot(obj.matrix_world.translation.x,
                           obj.matrix_world.translation.y) < 31]


def _position(rng, inner, outer, anchors, footprint, root_bias=.61):
    """Irregular root colonies plus continuous, patchy woodland ground cover."""
    local_anchors = [(x, y) for x, y in anchors
                     if inner - 2 < math.hypot(x, y) < outer + 2]
    for _ in range(2000):
        if local_anchors and rng.random() < root_bias:
            tx, ty = rng.choice(local_anchors)
            angle = rng.uniform(0, math.tau)
            # Uneven elliptical root colonies, rather than concentric rings.
            sigma = rng.uniform(.48, 1.55 if inner < 12 else 2.3)
            a, b = rng.gauss(0, sigma), rng.gauss(0, sigma * .64)
            x = tx + math.cos(angle) * a - math.sin(angle) * b
            y = ty + math.sin(angle) * a + math.cos(angle) * b
            if math.hypot(x - tx, y - ty) < .23:
                continue
        else:
            angle = rng.uniform(0, math.tau)
            radius = math.sqrt(rng.uniform(inner * inner, outer * outer))
            x, y = radius * math.cos(angle), radius * math.sin(angle)
        radius = math.hypot(x, y)
        if not inner <= radius <= outer:
            continue
        if max(abs(x), abs(y)) <= CLEAR_HALF_WIDTH + footprint:
            continue
        field = (.64 + .20 * math.sin(x * .59 + math.sin(y * .38))
                 + .13 * math.sin(y * .87 - x * .24))
        taper = 1 if outer <= 12 else 1 - .80 * ((radius - 12) / 16)
        if rng.random() < max(.08, field * taper):
            return x, y
    raise RuntimeError("Unable to place understory outside the flight volume")


def _ribbon(geometry, base, azimuth, height, half_width, bend, sway,
            twist, material_id, segments=4):
    """A tapered, twisted, curved narrow blade with an actual pointed tip."""
    forward = Vector((math.cos(azimuth), math.sin(azimuth), 0))
    side = Vector((-forward.y, forward.x, 0))
    start = len(geometry.vertices)
    for index in range(segments):
        t = index / segments
        center = (base + forward * bend * t * t
                  + side * sway * math.sin(math.pi * t)
                  + Vector((0, 0, height * (t - .16 * t ** 3))))
        edge = (side * math.cos(twist * t)
                + forward * math.sin(twist * t))
        width = half_width * (.62 + .55 * math.sin(math.pi * t)) * (1 - t) ** .55
        geometry.vertices.extend((tuple(center - edge * width),
                                  tuple(center + edge * width)))
    tip = base + forward * bend + Vector((0, 0, height * .84))
    geometry.vertices.append(tuple(tip))
    for index in range(segments - 1):
        a = start + index * 2
        geometry.faces.append((a, a + 1, a + 3, a + 2))
        geometry.material_ids.append(material_id)
    last = start + (segments - 1) * 2
    geometry.faces.append((last, last + 1, start + segments * 2))
    geometry.material_ids.append(material_id)


def _grass_patch(geometry, x, y, rng, materials, near):
    angle = rng.uniform(0, math.tau)
    radius = rng.uniform(.19, .49) if near else rng.uniform(.17, .42)
    aspect = rng.uniform(.55, 1.12)
    dominant = rng.randrange(materials)
    tall = near and rng.random() < .19
    count = rng.randrange(29, 43) if near else rng.randrange(14, 27)
    for _ in range(count):
        a = rng.uniform(0, math.tau)
        reach = radius * math.sqrt(rng.random())
        dx, dy = math.cos(a) * reach, math.sin(a) * reach * aspect
        xx = x + dx * math.cos(angle) - dy * math.sin(angle)
        yy = y + dx * math.sin(angle) + dy * math.cos(angle)
        base = Vector((xx, yy, _terrain_z(xx, yy) - .006))
        height = rng.uniform(.13, .38) if not tall else rng.uniform(.27, .57)
        material = dominant if rng.random() < .79 else rng.randrange(materials)
        _ribbon(geometry, base, rng.uniform(0, math.tau), height,
                rng.uniform(.0028, .0072), height * rng.uniform(.25, .65),
                rng.uniform(-.025, .025), rng.uniform(-.7, .7), material,
                4 if near else 3)
    return count


def _fern(geometry, x, y, rng, materials):
    base = Vector((x, y, _terrain_z(x, y) - .004))
    rotation = rng.uniform(0, math.tau)
    dominant = rng.randrange(materials)
    frond_count = rng.randrange(4, 8)
    for frond in range(frond_count):
        angle = rotation + frond * 2.39996 + rng.uniform(-.47, .47)
        forward = Vector((math.cos(angle), math.sin(angle), 0))
        lateral = Vector((-forward.y, forward.x, 0))
        reach, height = rng.uniform(.35, .85), rng.uniform(.28, .72)
        lean = rng.uniform(-.09, .09)

        def spine(t):
            return (base + forward * reach * t
                    + lateral * lean * math.sin(math.pi * t)
                    + Vector((0, 0, height * (.86 * math.sin(math.pi * t * .78)
                                               + .12 * t))))

        points = [spine(index / 6) for index in range(7)]
        _tube(geometry.vertices, geometry.faces, geometry.material_ids,
              points, [max(.00055, .0035 * (1 - index / 7))
                       for index in range(7)], sides=4, material_id=dominant)
        pairs = rng.randrange(9, 15)
        for index in range(pairs):
            t = .12 + .76 * index / (pairs - 1)
            for sign in (-1, 1):
                tt = min(.94, max(.08, t + rng.uniform(-.012, .012)))
                length = reach * .19 * math.sin(math.pi * tt) ** .65 * rng.uniform(.8, 1.18)
                direction = (lateral * sign + forward * rng.uniform(.26, .55)
                             + Vector((0, 0, rng.uniform(-.28, .16)))).normalized()
                across = direction.cross(Vector((0, 0, 1))).normalized()
                normal = direction.cross(across).normalized()
                center = spine(tt) + direction * length * .44
                material = dominant if rng.random() < .87 else rng.randrange(materials)
                _leaf(geometry.vertices, geometry.faces, geometry.material_ids,
                      center, length, length * rng.uniform(.18, .27),
                      (direction, across, normal), material)


def _slender_plant(geometry, x, y, rng, materials):
    azimuth = rng.uniform(0, math.tau)
    dominant = rng.randrange(materials)
    for leaf in range(rng.randrange(7, 13)):
        angle = azimuth + leaf * 2.39996 + rng.uniform(-.5, .5)
        xx, yy = x + rng.uniform(-.035, .035), y + rng.uniform(-.035, .035)
        base = Vector((xx, yy, _terrain_z(xx, yy) - .005))
        length = rng.uniform(.38, .86)
        _ribbon(geometry, base, angle, length, rng.uniform(.005, .013),
                length * rng.uniform(.38, .76), rng.uniform(-.05, .05),
                rng.uniform(-.45, .45), dominant, segments=5)


def _result(objects, collection, reused):
    triangle_count = sum(int(obj["triangle_count"]) for obj in objects)
    grass = next(obj for obj in objects if obj.name.startswith("ENV2_Understory_Grass"))
    fern = next(obj for obj in objects if obj.name.startswith("ENV2_Understory_Ferns"))
    slender = next(obj for obj in objects if obj.name.startswith("ENV2_Understory_SlenderLeaves"))
    return {
        "sample_objects": objects, "new_objects": objects,
        "collection": collection, "collections": [collection],
        "estimated_triangles": triangle_count, "total_triangles": triangle_count,
        "grass_patches": int(grass["patch_count"]),
        "grass_blades": int(grass["blade_count"]),
        "grass_near_patches": int(grass["near_patch_count"]),
        "grass_outer_patches": int(grass["outer_patch_count"]),
        "fern_clumps": int(fern["patch_count"]),
        "slender_leaf_clumps": int(slender["patch_count"]),
        "clear_flight_bounds_xy": [-1.7, 1.7, -1.7, 1.7],
        "clear_flight_max_z": 2.7,
        "distribution": "dense 3-12 m; tapering 12-28 m; irregular tree-root colonies",
        "geometry_contract": "Actual merged mesh leaves and stems; include these objects when rebuilding the scene BVH",
        "reused": reused,
    }


def append_understory(scene, seed=2026101107, grass_near=4200, grass_outer=1600,
                      fern_count=380, slender_count=180):
    """Add dense woodland plants, retaining the central flight volume.

    The seed and counts can be reduced for a CPU smoke test. Defaults make about
    5,800 grass patches and keep additional geometry below two million triangles.
    The function changes only its new collection/materials/objects and stores a
    compact count report on the scene; callers own saving and BVH/cloud rebuilds.
    """
    existing = [obj for obj in scene.objects
                if obj.get("understory_generator") == GENERATOR]
    if existing:
        if len(existing) != 3:
            raise RuntimeError("Partial understory exists; inspect its generated objects before retrying")
        return _result(existing, existing[0].users_collection[0], True)
    if any(not isinstance(count, int) or count <= 0
           for count in (grass_near, grass_outer, fern_count, slender_count)):
        raise ValueError("Understory counts must be positive integers")
    anchors = _tree_anchors(scene)
    grass_materials = _palette(
        ["ENV2_GrassMaterial_%d" % index for index in range(3)],
        [("ENV2_Understory_ShadeGrass", (.039, .108, .016)),
         ("ENV2_Understory_FreshGrass", (.079, .185, .031))])
    fern_materials = _palette(
        ["ENV2_LiveLeaf_%d" % index for index in (0, 1, 2)],
        [("ENV2_Understory_FernGreen", (.066, .171, .027))])
    collection = _collection("ENV2_Understory", scene)
    objects = []
    total_triangles = 0

    grass = _Geometry()
    rng = random.Random(seed)
    blades = 0
    for inner, outer, count, near in ((3, 12, grass_near, True),
                                     (12, 28, grass_outer, False)):
        for _ in range(count):
            x, y = _position(rng, inner, outer, anchors, 1.05)
            blades += _grass_patch(grass, x, y, rng, len(grass_materials), near)
    total_triangles += grass.triangles
    if total_triangles >= TRIANGLE_LIMIT:
        raise ValueError("Requested grass alone exceeds the triangle budget")
    obj = grass.make_object("ENV2_Understory_Grass", collection, grass_materials)
    obj["patch_count"] = grass_near + grass_outer
    obj["near_patch_count"], obj["outer_patch_count"] = grass_near, grass_outer
    obj["blade_count"] = blades
    objects.append(obj)
    del grass

    fern = _Geometry()
    rng = random.Random(seed + 12871)
    for index in range(fern_count):
        inner, outer = (3.2, 12) if index < fern_count * .84 else (12, 24)
        x, y = _position(rng, inner, outer, anchors, 1.1, root_bias=.83)
        _fern(fern, x, y, rng, len(fern_materials))
    total_triangles += fern.triangles
    if total_triangles >= TRIANGLE_LIMIT:
        raise ValueError("Requested ferns exceed the remaining triangle budget")
    obj = fern.make_object("ENV2_Understory_Ferns", collection, fern_materials)
    obj["patch_count"] = fern_count
    objects.append(obj)
    del fern

    slender = _Geometry()
    rng = random.Random(seed + 59243)
    for index in range(slender_count):
        inner, outer = (3.2, 12) if index < slender_count * .85 else (12, 25)
        x, y = _position(rng, inner, outer, anchors, .85, root_bias=.74)
        _slender_plant(slender, x, y, rng, len(fern_materials))
    total_triangles += slender.triangles
    if total_triangles >= TRIANGLE_LIMIT:
        raise ValueError("Requested slender plants exceed the remaining triangle budget")
    obj = slender.make_object("ENV2_Understory_SlenderLeaves", collection, fern_materials)
    obj["patch_count"] = slender_count
    objects.append(obj)
    del slender

    bpy.context.view_layer.update()
    result = _result(objects, collection, False)
    scene["understory_report"] = json.dumps({key: value for key, value in result.items()
                                             if key not in ("sample_objects", "new_objects",
                                                            "collection", "collections")})
    return result


if __name__ == "__main__":
    import time
    started = time.perf_counter()
    report = append_understory(bpy.context.scene)
    print("UNDERSTORY_CPU_BUILD", json.dumps({key: value for key, value in report.items()
          if key not in ("sample_objects", "new_objects", "collection", "collections")}), flush=True)
    print("UNDERSTORY_SECONDS", round(time.perf_counter() - started, 3), flush=True)
