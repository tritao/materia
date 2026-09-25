"""SensorKit HMPK interop vectors and generated Haxe checks."""

from __future__ import annotations

import json
import struct
from pathlib import Path
from typing import Any

from tools.wire.backends.common import lower_camel

EXAMPLES = Path(__file__).resolve().parent / "schema/sensor_wire_examples.json"

def _mp_uint(value: int) -> bytes:
    if value < 0:
        raise ValueError("wire examples only support non-negative values")
    if value < 128:
        return bytes([value])
    if value <= 0xFF:
        return b"\xcc" + struct.pack(">B", value)
    if value <= 0xFFFF:
        return b"\xcd" + struct.pack(">H", value)
    if value <= 0xFFFFFFFF:
        return b"\xce" + struct.pack(">I", value)
    return b"\xcf" + struct.pack(">Q", value)


def _mp_int(value: int) -> bytes:
    if value >= 0:
        return _mp_uint(value)
    if value >= -32:
        return bytes([value & 0xFF])
    if value >= -(2**7):
        return b"\xd0" + struct.pack(">b", value)
    if value >= -(2**15):
        return b"\xd1" + struct.pack(">h", value)
    if value >= -(2**31):
        return b"\xd2" + struct.pack(">i", value)
    return b"\xd3" + struct.pack(">q", value)


def _mp_binary(value: bytes) -> bytes:
    size = len(value)
    if size <= 0xFF:
        return b"\xc4" + struct.pack(">B", size) + value
    if size <= 0xFFFF:
        return b"\xc5" + struct.pack(">H", size) + value
    return b"\xc6" + struct.pack(">I", size) + value


def _mp_float(value: float) -> bytes:
    return b"\xcb" + struct.pack(">d", value)


def hmpk_message(fields: list[dict[str, Any]], example: dict[str, Any], *,
                 overrides: dict[int, Any] | None = None,
                 extra_fields: list[tuple[int, int | None]] | None = None,
                 omit_ids: set[int] | None = None,
                 trailing_data: bytes = b"") -> bytes:
    overrides = overrides or {}
    extra_fields = extra_fields or []
    omit_ids = omit_ids or set()
    encoded_fields = [field for field in fields if field["id"] not in omit_ids]
    count = len(encoded_fields) + len(extra_fields)
    if count <= 15:
        payload = bytes([0x80 | count])
    else:
        payload = b"\xde" + struct.pack(">H", count)
    for field in encoded_fields:
        payload += _mp_uint(field["id"])
        name = field["name"]
        if name == "data" and field["id"] in overrides:
            payload += _mp_binary(bytes(overrides[field["id"]]))
            continue
        if name == "data" and "data_f64" in example:
            value = b"".join(struct.pack("<d", float(item)) for item in example["data_f64"])
            if len(value) != 24 * 8:
                raise ValueError("IMU vector must contain exactly 24 values")
            payload += _mp_binary(value)
            continue
        elif name == "data" and "data_hex" in example:
            value = bytes.fromhex(example["data_hex"])
            payload += _mp_binary(value)
            continue
        value = overrides.get(field["id"], example[name])
        if field["type"] == "f64":
            payload += _mp_float(float(value))
        else:
            payload += _mp_int(int(value))
    for field_id, value in extra_fields:
        payload += _mp_uint(field_id)
        payload += b"\xc0" if value is None else _mp_int(value)
    payload += trailing_data
    return b"HMPK\x01\x00" + struct.pack(">I", len(payload)) + payload


def vector_fixture(norm: dict[str, Any]) -> str:
    examples = json.loads(EXAMPLES.read_text(encoding="utf-8"))
    lines = ["# Generated from schema/sensor_wire.nkw and sensor_wire_examples.json; complete HMPK frames."]
    for message_name, example in examples.items():
        message = norm["messages"][message_name]
        fields = message["fields"]
        lines.append(f"{message_name}\t{hmpk_message(fields, example).hex()}")
        sensor = next(field for field in fields if field["name"] == "sensor")
        lines.append(f"{message_name}.invalid.duplicate_field\t"
                     f"{hmpk_message(fields, example, extra_fields=[(sensor['id'], example['sensor'] + 1)]).hex()}")
        data = next(field for field in fields if field["name"] == "data")
        lines.append(f"{message_name}.invalid.missing_field\t"
                     f"{hmpk_message(fields, example, omit_ids={data['id']}).hex()}")
        lines.append(f"{message_name}.invalid.trailing_data\t"
                     f"{hmpk_message(fields, example, trailing_data=bytes([0xC0])).hex()}")
        for field in fields:
            if field["constant"]:
                lines.append(f"{message_name}.invalid.{field['name']}\t"
                             f"{hmpk_message(fields, example, overrides={field['id']: field['value'] + 1}).hex()}")
                lines.append(f"{message_name}.invalid.missing_{field['name']}\t"
                             f"{hmpk_message(fields, example, omit_ids={field['id']}).hex()}")
            elif field["nonnegative"]:
                lines.append(f"{message_name}.invalid.{field['name']}\t"
                             f"{hmpk_message(fields, example, overrides={field['id']: -1}).hex()}")
        if message_name == "PackedFrameMessage":
            lines.append(f"{message_name}.unknown_field\t"
                         f"{hmpk_message(fields, example, extra_fields=[(1000, None)]).hex()}")
            lines.append(f"{message_name}.uint32_max_width\t"
                         f"{hmpk_message(fields, example, overrides={8: 2**32 - 1}).hex()}")
            lines.append(f"{message_name}.invalid.width_range\t"
                         f"{hmpk_message(fields, example, overrides={8: 2**32}).hex()}")
        elif message_name == "LidarScanMessage":
            packed_data = bytes.fromhex(example["data_hex"])
            lines.append(f"{message_name}.invalid.packed_size\t"
                         f"{hmpk_message(fields, example, overrides={data['id']: packed_data[:-1]}).hex()}")
    return "\n".join(lines) + "\n"


def haxe_vector_check(norm: dict[str, Any]) -> str:
    examples = json.loads(EXAMPLES.read_text(encoding="utf-8"))
    fields_by_message = norm["messages"]
    enum_values = norm["enums"]
    used_enums = sorted({
        field["type"]
        for message in fields_by_message.values()
        for field in message["fields"]
        if field["type"] in enum_values
    })
    lines = [
        "import haxe.Int64;",
        "import haxe.io.Bytes;",
        "import haxeon.wire.MessagePackFrame;",
        "import materia.sensor.wire.ImuSampleMessage;",
        "import materia.sensor.wire.LidarScanMessage;",
        "import materia.sensor.wire.PackedFrameMessage;",
        "import materia.sensor.wire.SensorWireCodec;",
    ]
    lines.extend(f"import materia.sensor.wire.{enum_name};" for enum_name in used_enums)
    lines.extend([
        "import sys.io.File;",
        "",
        "/** Generated interop check. Compile with Haxeon and pass the generated TSV path. */",
        "class SensorWireVectorCheck {",
        "\tstatic function main():Int {",
        "\t\tvar expected = readVectors(Sys.args()[0]);",
    ])
    for message_name, example in examples.items():
        var_name = {"PackedFrameMessage": "packed", "ImuSampleMessage": "imu", "LidarScanMessage": "lidar"}[message_name]
        lines.append(f"\t\tvar {var_name} = new {message_name}();")
        for field in fields_by_message[message_name]["fields"]:
            name = field["name"]
            if name == "data":
                if "data_f64" in example:
                    raw = b"".join(struct.pack("<d", float(value)) for value in example["data_f64"])
                    literal = raw.hex()
                else:
                    literal = example["data_hex"]
                value_expr = f'bytesFromHex("{literal}")'
            elif field["type"] in {"u32", "u64", "i64"}:
                value_expr = f'Int64.parseString("{example[name]}")'
            elif field["type"] in enum_values:
                item_name = next(key for key, value in enum_values[field["type"]]["values"].items()
                                 if value == example[name])
                value_expr = f"{field['type']}.{lower_camel(item_name)}"
            else:
                value_expr = str(example[name])
            lines.append(f"\t\t{var_name}.{lower_camel(name)} = {value_expr};")
        vector_key = message_name
        lines.append(f'\t\tvar {var_name}Bytes = SensorWireCodec.encode{message_name}({var_name});')
        lines.append(f'\t\tverify(expected, "{vector_key}", MessagePackFrame.pack({var_name}Bytes));')
        lines.append(f'\t\tSensorWireCodec.decode{message_name}(MessagePackFrame.unpack(getVector(expected, "{vector_key}")));')
        for field in fields_by_message[message_name]["fields"]:
            if field["constant"] or field["nonnegative"]:
                key = f"{message_name}.invalid.{field['name']}"
                lines.append(f'\t\texpectRejected(expected, "{key}", "{message_name}");')
            if field["constant"]:
                key = f"{message_name}.invalid.missing_{field['name']}"
                lines.append(f'\t\texpectRejected(expected, "{key}", "{message_name}");')
        lines.append(f'\t\texpectRejected(expected, "{message_name}.invalid.duplicate_field", "{message_name}");')
        lines.append(f'\t\texpectRejected(expected, "{message_name}.invalid.missing_field", "{message_name}");')
        lines.append(f'\t\texpectRejected(expected, "{message_name}.invalid.trailing_data", "{message_name}");')
        if message_name == "PackedFrameMessage":
            lines.append('\t\tSensorWireCodec.decodePackedFrameMessage(MessagePackFrame.unpack(getVector(expected, "PackedFrameMessage.unknown_field")));')
            lines.append('\t\tSensorWireCodec.decodePackedFrameMessage(MessagePackFrame.unpack(getVector(expected, "PackedFrameMessage.uint32_max_width")));')
            lines.append('\t\texpectRejected(expected, "PackedFrameMessage.invalid.width_range", "PackedFrameMessage");')
    lines.extend([
        "\t\treturn 42;",
        "\t}",
        "",
        "\tstatic function readVectors(path:String):Map<String, Bytes> {",
        "\t\tvar result:Map<String, Bytes> = [];",
        "\t\tfor (line in File.getContent(path).split(\"\\n\")) {",
        "\t\t\tvar text = StringTools.trim(line);",
        "\t\t\tif (text == \"\" || StringTools.startsWith(text, \"#\")) continue;",
        "\t\t\tvar pair = text.split(\"\\t\");",
        "\t\t\tresult.set(pair[0], bytesFromHex(pair[1]));",
        "\t\t}",
        "\t\treturn result;",
        "\t}",
        "",
        "\tstatic function verify(expected:Map<String, Bytes>, name:String, actual:Bytes):Void {",
        "\t\tvar vector = expected.get(name);",
        "\t\tif (vector == null || vector.compare(actual) != 0) throw 'Sensor wire vector differs: $name';",
        "\t}",
        "",
        "\tstatic function getVector(expected:Map<String, Bytes>, name:String):Bytes {",
        "\t\tvar value = expected.get(name);",
        "\t\tif (value == null) throw 'Missing Sensor wire vector: $name';",
        "\t\treturn value;",
        "\t}",
        "",
        "\tstatic function expectRejected(expected:Map<String, Bytes>, name:String, message:String):Void {",
        "\t\tvar bytes = MessagePackFrame.unpack(getVector(expected, name));",
        "\t\tvar rejected = false;",
        "\t\ttry {",
        "\t\t\tswitch (message) {",
        "\t\t\t\tcase \"PackedFrameMessage\": SensorWireCodec.decodePackedFrameMessage(bytes);",
        "\t\t\t\tcase \"ImuSampleMessage\": SensorWireCodec.decodeImuSampleMessage(bytes);",
        "\t\t\t\tcase \"LidarScanMessage\": SensorWireCodec.decodeLidarScanMessage(bytes);",
        "\t\t\t\tdefault: throw 'Unknown Sensor wire message: $message';",
        "\t\t\t}",
        "\t\t} catch (error:Dynamic) {",
        "\t\t\trejected = true;",
        "\t\t}",
        "\t\tif (!rejected) throw 'Sensor wire decoder accepted invalid vector: $name';",
        "\t}",
        "",
        "\tstatic function bytesFromHex(text:String):Bytes {",
        "\t\tvar bytes = Bytes.alloc(text.length >> 1);",
        "\t\tfor (index in 0...bytes.length)",
        "\t\t\tbytes.set(index, (hexDigit(text.charCodeAt(index * 2)) << 4) | hexDigit(text.charCodeAt(index * 2 + 1)));",
        "\t\treturn bytes;",
        "\t}",
        "",
        "\tstatic function hexDigit(code:Int):Int {",
        "\t\tif (code >= 48 && code <= 57) return code - 48;",
        "\t\tif (code >= 65 && code <= 70) return code - 55;",
        "\t\treturn code - 87;",
        "\t}",
        "}",
        "",
    ])
    return "\n".join(lines)
