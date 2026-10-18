#!/usr/bin/env python3
"""Download the supplier CAD reference without vendoring it in project history."""
import hashlib
from pathlib import Path
import urllib.request

URL = 'https://dh-robotics-tr.com/media/docs/f47f5d03ad.step'
SHA256 = '8803e2416e133a9ec4766dc77e85595859bfa27d91081fb9154e1e0cebbc4a7c'
path = Path(__file__).resolve().parent / 'build/vendor/PGC-140-50.step'
data = path.read_bytes() if path.exists() else urllib.request.urlopen(URL, timeout=60).read()
if hashlib.sha256(data).hexdigest() != SHA256:
    raise SystemExit('Supplier CAD checksum changed; inspect the revision before updating the pin.')
path.parent.mkdir(parents=True, exist_ok=True)
path.write_bytes(data)
print(path)
