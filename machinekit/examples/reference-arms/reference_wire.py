"""Encode typed preparation data using the Haxe consumer's @:wire field IDs."""
import re


def encode(value, root_type, schema_paths):
    schemas = {}
    for path in schema_paths:
        for name, body in re.findall(r"@:wire typedef (\w+) = \{(.*?)\n\}", path.read_text(), re.S):
            schemas[name] = re.findall(r"@:id\((\d+)\)\s+(@:optional\s+)?var (\w+):([^;]+);", body)
    if root_type not in schemas:
        raise ValueError(f"Unknown wire root: {root_type}")

    def convert(item, kind):
        if item is None:
            return None
        if kind.startswith("Null<"):
            return convert(item, kind[5:-1])
        if kind.startswith("Array<"):
            return [convert(v, kind[6:-1]) for v in item]
        if kind not in schemas:
            return item
        return [[f"{index}:{name}", convert(item[name], field_type)]
                for index, optional, name, field_type in schemas[kind] if not optional or name in item]

    return {"version": 1, "value": convert(value, root_type)}
