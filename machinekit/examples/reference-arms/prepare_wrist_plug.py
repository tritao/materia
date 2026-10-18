"""Fetch the exact supplier candidate assets; never substitute newer unreviewed CAD."""
import hashlib
import json
import io
import zipfile
from pathlib import Path
from urllib.request import urlopen


def prepare(root, spec_name="wrist-plug-candidate.json"):
    root = Path(root)
    spec = json.loads((root / spec_name).read_text())
    for entry in spec['files']:
        target = root / 'build' / entry['path']
        if target.exists():
            data = target.read_bytes()
        else:
            data = urlopen(entry['url'], timeout=30).read()
            if 'archiveMember' in entry:
                if hashlib.sha256(data).hexdigest() != entry['archiveSha256']:
                    raise ValueError('Supplier candidate archive changed')
                data = zipfile.ZipFile(io.BytesIO(data)).read(entry['archiveMember'])
        if hashlib.sha256(data).hexdigest() != entry['sha256']:
            raise ValueError('Supplier candidate asset changed: ' + entry['path'])
        target.parent.mkdir(parents=True, exist_ok=True)
        if not target.exists():
            target.write_bytes(data)
    print(spec['id'] + ': supplier STEP and drawing SHA256 verified')


if __name__ == '__main__':
    for spec_name in ("wrist-plug-candidate.json", "wrist-right-angle-candidate.json"):
        prepare(Path(__file__).resolve().parent, spec_name)
