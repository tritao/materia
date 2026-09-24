"""Read-only geometry inventory for FreeCAD's AssemblyExample.FCStd.

Run this in FreeCAD's Python console after opening the reference file. It only
reads component-link shapes and prints JSON; it does not edit or save the file.
"""

import json

import FreeCAD as App


def shape_record(shape):
    if shape.isNull():
        return None
    bounds = shape.BoundBox
    return {
        "solids": len(shape.Solids),
        "faces": len(shape.Faces),
        "edges": len(shape.Edges),
        "volume": shape.Volume,
        "surfaceArea": shape.Area,
        "bounds": [
            bounds.XMin,
            bounds.YMin,
            bounds.ZMin,
            bounds.XMax,
            bounds.YMax,
            bounds.ZMax,
        ],
    }


def component_record(obj):
    shape = obj.Shape
    source = obj.LinkedObject
    return {
        "name": obj.Label,
        "source": source.Label if source is not None else None,
        "geometry": shape_record(shape),
        "definitionGeometry": shape_record(source.Shape) if source is not None else None,
    }


doc = App.ActiveDocument
if doc is None:
    raise RuntimeError("Open AssemblyExample.FCStd before running this script")

components = []
for obj in doc.Objects:
    if obj.TypeId != "App::Link":
        continue
    record = component_record(obj)
    if record is not None:
        components.append(record)

components.sort(key=lambda item: item["name"])
print(json.dumps({"document": doc.Name, "components": components}, indent=2))
