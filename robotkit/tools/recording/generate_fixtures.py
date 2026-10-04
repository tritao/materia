"""Regenerate intentionally foreign and malformed-schema MCAP reader fixtures.

Run with: uv run --with mcap==1.5.0 python robotkit/tools/recording/generate_fixtures.py
The historical recording-v5 fixture stays as an unsupported-version input.
"""
from pathlib import Path
import re
from mcap.writer import CompressionType, Writer

ROOT = Path(__file__).resolve().parents[3]
source = (ROOT / 'robotkit/haxe/robotkit/recording/RobotRecordingEntry.hx').read_text()
version = re.search(r'VERSION:Int = (\d+)', source).group(1)
fixtures = ROOT / 'robotkit/tests/fixtures'
for filename, name, encoding, topic, message_encoding, payload in [
    ('recording-schema-mismatch.mcap', 'WrongSnapshot', 'robotkit-wire',
     'robotkit/snapshot', 'msgpack', b'\x80'),
    ('recording-foreign.mcap', 'Foreign', 'jsonschema',
     'foreign/diagnostic', 'json', b'{}'),
]:
    with (fixtures / filename).open('wb') as stream:
        writer = Writer(stream, compression=CompressionType.NONE)
        writer.start()
        schema_id = writer.register_schema(name, encoding, b'{}')
        channel_id = writer.register_channel(topic, message_encoding, schema_id)
        writer.add_metadata('robotkit', {'robotkit.schema_version': version})
        writer.add_message(channel_id, log_time=1, publish_time=0, data=payload)
        writer.finish()
