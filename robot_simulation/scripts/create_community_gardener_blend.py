#!/usr/bin/env python3
"""Build a small stylized gardener from community person reference image 1."""

import math
from pathlib import Path

import bpy
from mathutils import Vector


ROOT = Path(__file__).resolve().parents[3]
REFERENCE = ROOT / "复赛资料" / "人员" / "社区人员" / "1.png"
OUTPUT = ROOT / "src" / "robot_simulation" / "models" / "person_community_01_3d"
NAME = "person_community_01_3d"
HEIGHT = 0.15
WIDTH = 0.05


def material(name, color, roughness=0.8, metallic=0.0):
    mat = bpy.data.materials.new(name)
    mat.diffuse_color = (*color, 1)
    mat.use_nodes = True
    shader = next(node for node in mat.node_tree.nodes
                  if node.type == "BSDF_PRINCIPLED")
    shader.inputs["Base Color"].default_value = (*color, 1)
    shader.inputs["Roughness"].default_value = roughness
    shader.inputs["Metallic"].default_value = metallic
    return mat


def finish(obj, name, mat, bevel=0):
    obj.name = name
    obj.data.name = name
    obj.data.materials.append(mat)
    if bevel:
        modifier = obj.modifiers.new("Soft edges", "BEVEL")
        modifier.width = bevel
        modifier.segments = 2
        modifier.affect = "EDGES"
        obj.modifiers.new("Weighted normals", "WEIGHTED_NORMAL")
    for polygon in obj.data.polygons:
        polygon.use_smooth = True
    return obj


def ellipsoid(name, center, radius, mat, segments=24):
    bpy.ops.mesh.primitive_uv_sphere_add(segments=segments, ring_count=12, location=center)
    obj = bpy.context.object
    obj.scale = radius
    return finish(obj, name, mat)


def box(name, center, size, mat, bevel=0.0003):
    bpy.ops.mesh.primitive_cube_add(size=1, location=center)
    obj = bpy.context.object
    obj.dimensions = size
    bpy.ops.object.transform_apply(location=False, rotation=False, scale=True)
    return finish(obj, name, mat, bevel)


def between(name, start, end, radius, mat, vertices=16, radius_end=None):
    direction = Vector(end) - Vector(start)
    middle = (Vector(start) + Vector(end)) / 2
    bpy.ops.mesh.primitive_cone_add(
        vertices=vertices,
        radius1=radius,
        radius2=radius if radius_end is None else radius_end,
        depth=direction.length,
        location=middle,
    )
    obj = bpy.context.object
    obj.rotation_euler = direction.to_track_quat("Z", "Y").to_euler()
    return finish(obj, name, mat)


def path(name, points, thickness, mat):
    curve = bpy.data.curves.new(name, "CURVE")
    curve.dimensions = "3D"
    curve.bevel_depth = thickness
    curve.bevel_resolution = 3
    spline = curve.splines.new("POLY")
    spline.points.add(len(points) - 1)
    for point, position in zip(spline.points, points):
        point.co = (*position, 1)
    obj = bpy.data.objects.new(name, curve)
    bpy.context.collection.objects.link(obj)
    obj.data.materials.append(mat)
    return obj


def mesh(name, vertices, faces, mat, bevel=0):
    data = bpy.data.meshes.new(name)
    data.from_pydata(vertices, [], faces)
    data.update()
    obj = bpy.data.objects.new(name, data)
    bpy.context.collection.objects.link(obj)
    return finish(obj, name, mat, bevel)


def apron_panel(blue):
    # A shallow six-sided body makes the apron readable in front and in profile.
    outline = [(-0.0105, 0.048), (0.0105, 0.048), (0.0115, 0.084),
               (0.007, 0.113), (-0.007, 0.113), (-0.0115, 0.084)]
    vertices = [(x, y, z) for y in (-0.0100, -0.0080) for x, z in outline]
    faces = [tuple(range(5, -1, -1)), tuple(range(6, 12))]
    faces.extend((i, (i + 1) % 6, (i + 1) % 6 + 6, i + 6) for i in range(6))
    return mesh("Blue apron body", vertices, faces, blue, 0.00045)


def model():
    skin = material("warm skin", (0.70, 0.49, 0.38))
    shadow = material("face details", (0.085, 0.067, 0.057))
    cream = material("linen shirt", (0.66, 0.65, 0.58))
    seams = material("linen seams", (0.39, 0.39, 0.34))
    blue = material("indigo apron", (0.077, 0.24, 0.38))
    blue_light = material("apron pocket", (0.10, 0.30, 0.46))
    blue_dark = material("apron seams", (0.03, 0.12, 0.20))
    trouser = material("grey trousers", (0.40, 0.43, 0.39))
    boots = material("rubber boots", (0.08, 0.14, 0.11), 0.48)
    hat = material("brown felt hat", (0.22, 0.16, 0.11))
    hat_band = material("hat band", (0.13, 0.12, 0.11))
    steel = material("garden tool steel", (0.37, 0.46, 0.48), 0.33, 0.65)
    wood = material("garden tool handle", (0.35, 0.20, 0.12))

    # Feet and clothes rise from the ground, with the front facing negative Y.
    for side, x in (("left", -0.007), ("right", 0.007)):
        between(side + " trouser leg", (x, 0.0007, 0.029),
                (x * 0.83, 0, 0.056), 0.0048, trouser, radius_end=0.0056)
        between(side + " boot shaft", (x, 0, 0.007),
                (x, 0.0005, 0.033), 0.0048, boots, radius_end=0.0052)
        ellipsoid(side + " boot foot", (x, -0.0032, 0.0042),
                  (0.0056, 0.008, 0.0034), boots)
        path(side + " boot rim", [(x - 0.004, -0.003, 0.030),
                                    (x, -0.005, 0.030),
                                    (x + 0.004, -0.003, 0.030)], 0.00032, seams)

    ellipsoid("shirt torso", (0, 0, 0.089), (0.014, 0.008, 0.029), cream)
    ellipsoid("shirt collar", (0, -0.001, 0.117), (0.006, 0.005, 0.004), cream)
    path("shirt opening", [(0, -0.0083, 0.113), (0, -0.0085, 0.101)],
         0.00027, seams)
    for side, sign in (("left", -1), ("right", 1)):
        shoulder = (sign * 0.0115, 0, 0.105)
        elbow = (sign * 0.0168, -0.0005, 0.083)
        wrist = (sign * 0.0192, -0.004, 0.065)
        between(side + " upper sleeve", elbow, shoulder, 0.0046, cream,
                radius_end=0.0056)
        between(side + " rolled cuff", (sign * 0.017, -0.001, 0.081),
                (sign * 0.0176, -0.002, 0.077), 0.0047, seams)
        between(side + " forearm", wrist, elbow, 0.0035, skin)
        ellipsoid(side + " hand", (sign * 0.0192, -0.0045, 0.063),
                  (0.003, 0.003, 0.005), skin)

    apron_panel(blue)
    for sign in (-1, 1):
        path("neck strap", [(sign * 0.0068, -0.0102, 0.112),
                             (sign * 0.004, -0.0089, 0.121)], 0.00065, blue_dark)
        path("waist tie", [(sign * 0.010, -0.0107, 0.079),
                           (sign * 0.014, -0.0087, 0.075)], 0.00068, blue_dark)
    box("large apron pocket", (0, -0.0114, 0.069),
        (0.0115, 0.0010, 0.011), blue_light)
    path("pocket opening", [(-0.005, -0.0121, 0.0745),
                             (0.005, -0.0121, 0.0745)], 0.0003, blue_dark)
    box("chest pocket", (0, -0.0113, 0.101),
        (0.0085, 0.0009, 0.007), blue_light)
    path("chest pocket edge", [(-0.004, -0.0119, 0.1045),
                                (0.004, -0.0119, 0.1045)], 0.00025, blue_dark)
    path("apron bow left", [(0, -0.0114, 0.079), (-0.0045, -0.0120, 0.081),
                              (-0.006, -0.0117, 0.077)], 0.00063, blue_dark)
    path("apron bow right", [(0, -0.0114, 0.079), (0.0045, -0.0120, 0.081),
                               (0.0058, -0.0117, 0.076)], 0.00063, blue_dark)
    path("apron knot tail", [(0, -0.0118, 0.078), (0.002, -0.0122, 0.068)],
         0.00058, blue_dark)
    ellipsoid("apron knot", (0, -0.0119, 0.079),
              (0.0016, 0.0011, 0.0015), blue_dark)

    between("neck", (0, 0, 0.115), (0, 0, 0.124), 0.0035, skin)
    ellipsoid("head", (0, -0.0008, 0.133), (0.0070, 0.0061, 0.011), skin)
    ellipsoid("hair", (0, 0.0005, 0.140), (0.0073, 0.0063, 0.0055), shadow)
    for sign in (-1, 1):
        ellipsoid("ear", (sign * 0.0068, -0.0015, 0.132),
                  (0.0016, 0.0016, 0.003), skin)
        ellipsoid("eye", (sign * 0.0026, -0.0065, 0.135),
                  (0.00043, 0.00032, 0.0007), shadow, 12)
        path("eyebrow", [(sign * 0.0015, -0.0066, 0.1374),
                          (sign * 0.004, -0.0062, 0.137)], 0.00025, shadow)
    ellipsoid("nose", (0, -0.0069, 0.132), (0.0008, 0.0007, 0.0016), skin)
    path("smile", [(-0.0022, -0.0061, 0.1287), (0, -0.0071, 0.1282),
                    (0.0022, -0.0061, 0.1287)], 0.00020, shadow)

    ellipsoid("wide hat brim", (0, 0, 0.144),
              (0.018, 0.013, 0.0014), hat)
    ellipsoid("hat crown", (0, 0.001, 0.148),
              (0.0085, 0.007, 0.0041), hat)
    between("hat band", (0, 0.001, 0.145),
            (0, 0.001, 0.1466), 0.0080, hat_band, 24)

    # A narrow spade and secateurs keep the silhouette faithful to the source.
    between("spade handle", (-0.0187, -0.004, 0.061),
            (-0.024, -0.005, 0.053), 0.00075, wood)
    ellipsoid("spade blade", (-0.024, -0.005, 0.052),
              (0.0020, 0.0005, 0.0012), steel)
    between("secateur wood handle", (0.020, -0.0048, 0.061),
            (0.023, -0.0048, 0.052), 0.0010, wood)
    path("secateur left blade", [(0.023, -0.0048, 0.052),
                                  (0.019, -0.005, 0.046),
                                  (0.017, -0.005, 0.042)], 0.00055, steel)
    path("secateur right blade", [(0.023, -0.0048, 0.052),
                                   (0.024, -0.0047, 0.046),
                                   (0.025, -0.0047, 0.042)], 0.00055, steel)
    ellipsoid("secateur pivot", (0.0224, -0.0054, 0.052),
              (0.0010, 0.0006, 0.0010), steel)


def fit_dimensions():
    meshes = [obj for obj in bpy.data.objects if obj.type in {"MESH", "CURVE"}]
    bpy.context.view_layer.update()
    bounds = [obj.matrix_world @ Vector(corner) for obj in meshes
              for corner in obj.bound_box]
    low_x, high_x = min(v.x for v in bounds), max(v.x for v in bounds)
    low_z, high_z = min(v.z for v in bounds), max(v.z for v in bounds)
    scale_x = WIDTH / (high_x - low_x)
    scale_z = HEIGHT / (high_z - low_z)
    center_x = (high_x + low_x) / 2
    for obj in meshes:
        obj.location.x = (obj.location.x - center_x) * scale_x
        obj.location.z = (obj.location.z - low_z) * scale_z
        obj.scale.x *= scale_x
        obj.scale.z *= scale_z
    bpy.context.view_layer.update()
    bounds = [obj.matrix_world @ Vector(corner) for obj in meshes
              for corner in obj.bound_box]
    result = (max(v.x for v in bounds) - min(v.x for v in bounds),
              max(v.z for v in bounds) - min(v.z for v in bounds))
    assert abs(result[0] - WIDTH) < 1e-7
    assert abs(result[1] - HEIGHT) < 1e-7
    return result


def camera_and_reference():
    image = bpy.data.images.load(str(REFERENCE), check_existing=True)
    bpy.ops.object.empty_add(type="IMAGE", location=(0, 0.021, 0.075))
    reference = bpy.context.object
    reference.name = "Reference image - front view"
    reference.data = image
    reference.empty_display_size = HEIGHT
    reference.rotation_euler.x = math.pi / 2
    reference.hide_render = True
    reference.hide_set(True)

    bpy.ops.object.camera_add(location=(0.025, -0.38, 0.089))
    camera = bpy.context.object
    direction = Vector((0, 0, 0.077)) - camera.location
    camera.rotation_euler = direction.to_track_quat("-Z", "Y").to_euler()
    camera.data.type = "ORTHO"
    camera.data.ortho_scale = 0.185
    bpy.context.scene.camera = camera
    for name, position, energy, size in (
        ("softbox left", (-0.13, -0.15, 0.21), 1.1, 0.13),
        ("softbox right", (0.12, 0.05, 0.18), 0.7, 0.11),
    ):
        bpy.ops.object.light_add(type="AREA", location=position)
        light = bpy.context.object
        light.name = name
        light.data.energy = energy
        light.data.shape = "DISK"
        light.data.size = size
        light.rotation_euler = (Vector((0, 0, 0.075)) - light.location).to_track_quat(
            "-Z", "Y").to_euler()


def main():
    if not REFERENCE.is_file():
        raise FileNotFoundError(REFERENCE)
    OUTPUT.mkdir(parents=True, exist_ok=True)
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete(use_global=False)
    bpy.context.scene.unit_settings.system = "METRIC"
    bpy.context.scene.unit_settings.length_unit = "CENTIMETERS"
    model()
    width, height = fit_dimensions()
    camera_and_reference()
    scene = bpy.context.scene
    scene.render.engine = "CYCLES"
    scene.cycles.device = "CPU"
    scene.cycles.samples = 24
    scene.render.resolution_x = 650
    scene.render.resolution_y = 1100
    scene.render.resolution_percentage = 100
    scene.render.film_transparent = False
    scene.render.image_settings.file_format = "PNG"
    scene.render.filepath = str(OUTPUT / (NAME + "_preview.png"))
    scene.world.use_nodes = True
    background = next(node for node in scene.world.node_tree.nodes
                      if node.type == "BACKGROUND")
    background.inputs["Color"].default_value = (0.55, 0.61, 0.62, 1)
    background.inputs["Strength"].default_value = 0.7
    bpy.ops.file.pack_all()
    bpy.context.preferences.filepaths.save_version = 0
    bpy.ops.wm.save_as_mainfile(filepath=str(OUTPUT / (NAME + ".blend")))
    bpy.ops.export_scene.gltf(filepath=str(OUTPUT / (NAME + ".glb")),
                              export_format="GLB", export_yup=True,
                              use_selection=False, export_cameras=False,
                              export_lights=False)
    bpy.ops.render.render(write_still=True)
    print("MODEL_DIMENSIONS_METERS width={:.9f} height={:.9f}".format(width, height))


if __name__ == "__main__":
    main()
