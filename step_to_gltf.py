"""Convert a colored STEP model to a meter-scaled glTF 2.0 asset.

The only runtime dependency is cadquery-ocp, which provides the Open Cascade
STEPCAF reader and tessellator.  STEP models are commonly authored in mm;
the output is written in meters, matching the supplied example.
"""

from __future__ import annotations

import argparse
import json
import math
import struct
from collections import defaultdict
from pathlib import Path

from OCP.BRep import BRep_Tool
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.IFSelect import IFSelect_ReturnStatus
from OCP.Quantity import Quantity_Color
from OCP.STEPCAFControl import STEPCAFControl_Reader
from OCP.TCollection import TCollection_ExtendedString
from OCP.TDF import TDF_Label, TDF_LabelSequence
from OCP.TDocStd import TDocStd_Document
from OCP.TopAbs import TopAbs_FACE
from OCP.TopExp import TopExp_Explorer
from OCP.TopLoc import TopLoc_Location
from OCP.TopoDS import TopoDS
from OCP.XCAFApp import XCAFApp_Application
from OCP.XCAFDoc import XCAFDoc_ColorType, XCAFDoc_DocumentTool, XCAFDoc_ShapeTool


DEFAULT_COLOR = (0.69, 0.69, 0.69, 1.0)


def _color_for_shape(color_tool: object, shape: object) -> tuple[float, float, float, float]:
    color = Quantity_Color()
    for color_type in (
        XCAFDoc_ColorType.XCAFDoc_ColorSurf,
        XCAFDoc_ColorType.XCAFDoc_ColorGen,
        XCAFDoc_ColorType.XCAFDoc_ColorCurv,
    ):
        if color_tool.GetColor(shape, color_type, color):
            return (float(color.Red()), float(color.Green()), float(color.Blue()), 1.0)
    return DEFAULT_COLOR


def _color_for_component(color_tool: object, component: object, definition: object) -> tuple[float, float, float, float]:
    instance_color = Quantity_Color()
    if color_tool.GetInstanceColor(component, XCAFDoc_ColorType.XCAFDoc_ColorSurf, instance_color):
        return (float(instance_color.Red()), float(instance_color.Green()), float(instance_color.Blue()), 1.0)
    color = _color_for_shape(color_tool, component)
    if color != DEFAULT_COLOR:
        return color
    return _color_for_shape(color_tool, definition)


def _colored_components(
    label: object,
    shape_tool: object,
    color_tool: object,
    parent_location: TopLoc_Location | None = None,
):
    parent_location = parent_location or TopLoc_Location()
    if not XCAFDoc_ShapeTool.IsAssembly_s(label):
        shape = shape_tool.GetShape_s(label)
        if not shape.IsNull():
            yield shape, [color for _, color in _walk_faces(shape, color_tool)]
        return

    components = TDF_LabelSequence()
    XCAFDoc_ShapeTool.GetComponents_s(label, components)
    for index in range(1, components.Length() + 1):
        component = components.Value(index)
        referred = TDF_Label()
        target = referred if XCAFDoc_ShapeTool.GetReferredShape_s(component, referred) else component
        component_shape = shape_tool.GetShape_s(component)
        location = parent_location.Multiplied(component_shape.Location())
        if XCAFDoc_ShapeTool.IsAssembly_s(target):
            yield from _colored_components(target, shape_tool, color_tool, location)
            continue
        target_shape = shape_tool.GetShape_s(target)
        if not target_shape.IsNull():
            shape = target_shape.Located(TopLoc_Location()).Located(location)
            colors = [color for _, color in _walk_faces(target_shape, color_tool)]
            component_color = _color_for_component(color_tool, component_shape, target_shape)
            if all(color == DEFAULT_COLOR for color in colors):
                colors = [component_color] * len(colors)
            yield shape, colors


def _read_step(path: Path) -> tuple[object, list[tuple[object, list[tuple[float, float, float, float]]]]]:
    reader = STEPCAFControl_Reader()
    reader.SetColorMode(True)
    reader.SetNameMode(True)
    reader.SetLayerMode(True)
    if reader.ReadFile(str(path)) != IFSelect_ReturnStatus.IFSelect_RetDone:
        raise RuntimeError(f"Could not read STEP file: {path}")
    XCAFApp_Application.GetApplication_s()
    document = TDocStd_Document(TCollection_ExtendedString("step-to-gltf"))
    if not reader.Transfer(document):
        raise RuntimeError(f"Could not transfer STEP file: {path}")

    shape_tool = XCAFDoc_DocumentTool.ShapeTool_s(document.Main())
    color_tool = XCAFDoc_DocumentTool.ColorTool_s(document.Main())
    roots = TDF_LabelSequence()
    shape_tool.GetFreeShapes(roots)
    if roots.Length() == 0:
        raise RuntimeError("The STEP file does not contain a transferable shape")
    colored_shapes = []
    for index in range(1, roots.Length() + 1):
        colored_shapes.extend(_colored_components(roots.Value(index), shape_tool, color_tool))
    if not colored_shapes:
        raise RuntimeError("The STEP file does not contain a transferable colored shape")
    return document, colored_shapes


def _color_for_face(color_tool: object, face: object) -> tuple[float, float, float, float]:
    color = Quantity_Color()
    for color_type in (
        XCAFDoc_ColorType.XCAFDoc_ColorSurf,
        XCAFDoc_ColorType.XCAFDoc_ColorGen,
        XCAFDoc_ColorType.XCAFDoc_ColorCurv,
    ):
        if color_tool.GetColor(face, color_type, color):
            return (float(color.Red()), float(color.Green()), float(color.Blue()), 1.0)
    return DEFAULT_COLOR


def _walk_faces(shape: object, color_tool: object | None = None):
    explorer = TopExp_Explorer(shape, TopAbs_FACE)
    while explorer.More():
        face = TopoDS.Face_s(explorer.Current())
        yield face, _color_for_face(color_tool, face) if color_tool else DEFAULT_COLOR
        explorer.Next()


def _normal(a: tuple[float, float, float], b: tuple[float, float, float], c: tuple[float, float, float]):
    ux, uy, uz = b[0] - a[0], b[1] - a[1], b[2] - a[2]
    vx, vy, vz = c[0] - a[0], c[1] - a[1], c[2] - a[2]
    nx, ny, nz = uy * vz - uz * vy, uz * vx - ux * vz, ux * vy - uy * vx
    length = math.sqrt(nx * nx + ny * ny + nz * nz) or 1.0
    return nx / length, ny / length, nz / length


def _tessellate(colored_shapes: list[tuple[object, list[tuple[float, float, float, float]]]], deflection: float):
    groups: dict[tuple[int, tuple[float, float, float, float]], tuple[list[float], list[float], list[int]]] = defaultdict(
        lambda: ([], [], [])
    )
    for shape_index, (shape, colors) in enumerate(colored_shapes):
        BRepMesh_IncrementalMesh(shape, deflection, False, math.radians(12), False)
        for face_index, (face, _) in enumerate(_walk_faces(shape)):
            color = colors[face_index] if face_index < len(colors) else DEFAULT_COLOR
            location = TopLoc_Location()
            triangulation = BRep_Tool.Triangulation_s(face, location)
            if triangulation is None:
                continue
            positions, normals, indices = groups[(shape_index, color)]
            offset = len(positions) // 3
            transformation = location.Transformation()
            for node_index in range(1, triangulation.NbNodes() + 1):
                point = triangulation.Node(node_index).Transformed(transformation)
                positions.extend((point.X() * 0.001, point.Y() * 0.001, point.Z() * 0.001))
                normals.extend((0.0, 0.0, 0.0))
            for triangle_index in range(1, triangulation.NbTriangles() + 1):
                triangle = triangulation.Triangle(triangle_index)
                first, second, third = triangle.Get()
                a = tuple(positions[(offset + first - 1) * 3 : (offset + first) * 3])
                b = tuple(positions[(offset + second - 1) * 3 : (offset + second) * 3])
                c = tuple(positions[(offset + third - 1) * 3 : (offset + third) * 3])
                face_normal = _normal(a, b, c)
                for vertex in (first, second, third):
                    normal_offset = (offset + vertex - 1) * 3
                    normals[normal_offset : normal_offset + 3] = face_normal
                indices.extend((offset + first - 1, offset + second - 1, offset + third - 1))
    return groups


def _append_f32(data: bytearray, values: list[float]) -> tuple[int, int]:
    while len(data) % 4:
        data.append(0)
    offset = len(data)
    data.extend(struct.pack(f"<{len(values)}f", *values))
    return offset, len(values) * 4


def write_gltf(groups: dict, output: Path) -> None:
    binary = bytearray()
    accessors = []
    buffer_views = []
    meshes = []
    nodes = []
    materials = []
    primitives_by_shape: dict[int, list[dict]] = defaultdict(list)

    for material_index, ((shape_index, color), (positions, normals, indices)) in enumerate(groups.items()):
        position_offset, position_bytes = _append_f32(binary, positions)
        normal_offset, normal_bytes = _append_f32(binary, normals)
        while len(binary) % 4:
            binary.append(0)
        index_offset = len(binary)
        binary.extend(struct.pack(f"<{len(indices)}I", *indices))
        index_bytes = len(indices) * 4

        position_view = len(buffer_views)
        buffer_views.extend([
            {"buffer": 0, "byteOffset": position_offset, "byteLength": position_bytes},
            {"buffer": 0, "byteOffset": normal_offset, "byteLength": normal_bytes},
            {"buffer": 0, "byteOffset": index_offset, "byteLength": index_bytes},
        ])
        position_accessor = len(accessors)
        points = list(zip(*(iter(positions),) * 3))
        accessors.append({"type": "VEC3", "componentType": 5126, "count": len(points), "max": [max(v[i] for v in points) for i in range(3)], "min": [min(v[i] for v in points) for i in range(3)], "bufferView": position_view})
        normal_accessor = len(accessors)
        accessors.append({"type": "VEC3", "componentType": 5126, "count": len(points), "bufferView": position_view + 1})
        index_accessor = len(accessors)
        accessors.append({"type": "SCALAR", "componentType": 5125, "count": len(indices), "bufferView": position_view + 2})
        materials.append({"doubleSided": True, "pbrMetallicRoughness": {"baseColorFactor": list(color), "metallicFactor": 0}})
        primitives_by_shape[shape_index].append({"attributes": {"POSITION": position_accessor, "NORMAL": normal_accessor}, "indices": index_accessor, "material": material_index, "mode": 4})

    for part_index, primitives in enumerate(primitives_by_shape.values(), start=1):
        name = f"Part {part_index}"
        meshes.append({"name": name, "primitives": primitives})
        nodes.append({"mesh": len(meshes) - 1, "name": name})

    document = {
        "asset": {"generator": "step-to-gltf", "version": "2.0"},
        "scene": 0,
        "scenes": [{"nodes": list(range(len(nodes)))}],
        "nodes": nodes,
        "meshes": meshes,
        "materials": materials,
        "accessors": accessors,
        "bufferViews": buffer_views,
        "buffers": [{"byteLength": len(binary), "uri": output.with_suffix(".bin").name}],
    }
    output.write_text(json.dumps(document, separators=(",", ":"), ensure_ascii=True) + "\n", encoding="utf-8")
    output.with_suffix(".bin").write_bytes(binary)


def main() -> None:
    parser = argparse.ArgumentParser(description="Convert a STEP file to meter-scaled glTF with STEP colors")
    parser.add_argument("input", type=Path, help="Input .stp or .step file")
    parser.add_argument("output", type=Path, nargs="?", help="Output .gltf path (defaults beside input)")
    parser.add_argument("--deflection", type=float, default=0.1, help="Mesh tolerance in source units (default: 0.1 mm)")
    args = parser.parse_args()
    output = args.output or args.input.with_suffix(".gltf")
    if args.deflection <= 0:
        parser.error("--deflection must be greater than zero")
    document, colored_shapes = _read_step(args.input)
    groups = _tessellate(colored_shapes, args.deflection)
    if not groups:
        raise RuntimeError("No tessellated faces were found")
    output.parent.mkdir(parents=True, exist_ok=True)
    write_gltf(groups, output)
    part_count = len({shape_index for shape_index, _ in groups})
    print(f"Wrote {output} and {output.with_suffix('.bin')} ({part_count} parts, {len(groups)} material groups)")


if __name__ == "__main__":
    main()