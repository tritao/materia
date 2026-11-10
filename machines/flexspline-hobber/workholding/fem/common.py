import os, sys, json, math
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np
import cupfem as c
from concurrent.futures import ThreadPoolExecutor
RES = os.path.join(c.HERE, "results"); os.makedirs(RES, exist_ok=True)

def pmap(fn, items, workers=2):
    with ThreadPoolExecutor(workers) as ex: return list(ex.map(fn, items))

def save(name, obj):
    json.dump(obj, open(os.path.join(RES, name + ".json"), "w"), indent=1, default=lambda o: o.tolist() if hasattr(o, "tolist") else float(o))

def um(x): return float(x) * 1e3

# sleeve variants (assumed lands; see README): land_lo/land_hi = rigidly fixed inner lands at the open end / far end
VARIANTS = {
    "V1 lands 1.5/1.5, L16": dict(land_lo=1.5, land_hi=1.5, length=16.0),
    "V2 lands 0.5/1.5, L16": dict(land_lo=0.5, land_hi=1.5, length=16.0),
    "V3 free open end, far land 1.5, L16": dict(land_lo=0.0, land_hi=1.5, length=16.0),
    "V4 lands 1.5/1.5, L24": dict(land_lo=1.5, land_hi=1.5, length=24.0),
}
