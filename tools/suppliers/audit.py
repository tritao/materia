#!/usr/bin/env python3
"""Classify imported Vallder listings and prepare evidence-backed fastener mappings."""
import argparse
from collections import Counter
import csv
from html import unescape
import json
from pathlib import Path
import re
from urllib.parse import urljoin, urlsplit

from vallder import DEFAULT_OUTPUT, now, write_json


ROOT = Path(__file__).resolve().parents[2]
COMPONENTS = ROOT / "machinekit/haxe/src/machinekit"
# Candidate families only: matching a category never qualifies a supplier part.
FAMILIES = {
    "aluminum-profiles": ["structural/TSlotExtrusion"],
    "linear-guideways": ["motion/LinearRail", "motion/LinearRailBlock"],
    "linear-bearings": ["motion/LinearBearing"],
    "ball-screws": ["motion/BallScrew", "motion/LeadScrewNut"],
    "ball-screw-support": ["motion/ScrewSupportUnit"],
    "linear-shafts-supports": ["motion/SteppedShaft", "motion/GuideSupport"],
    "couplings": ["motion/ShaftCoupling"],
    "ac-servo": ["motion/ServoMotor"],
    "stepper-motor-drivers": ["motion/NemaStepper"],
    "levelling-feets-casters": ["motion/CasterWheel", "milling/MillLevelingFoot"],
    "bolts-nuts": ["standard/SocketHeadCapScrew", "standard/HexNut"],
    "pulleys-belts": ["transmission/TimingPulley"],
    "bearings-housing": ["standard/DeepGrooveBearing", "motion/PillowBlock"],
}


def audit_product(product, variant_count):
    name = unescape(product["name"])
    candidates = sorted({family for category in product.get("categories", [])
                         for family in FAMILIES.get(category["slug"], [])
                         if (COMPONENTS / (family + ".hx")).exists()})
    links = {"step": [], "archive": [], "drawing": [], "document": []}
    for link in product.get("description_links", []):
        url = urljoin(product["url"], link)
        extension = Path(urlsplit(url).path).suffix.lower()
        kind = {".step": "step", ".stp": "step", ".zip": "archive", ".rar": "archive",
                ".7z": "archive", ".dxf": "drawing", ".dwg": "drawing", ".iges": "drawing",
                ".igs": "drawing", ".pdf": "document"}.get(extension)
        if kind and url not in links[kind]:
            links[kind].append(url)
    if name in ("End Machining", "Wooden Package", "test"):
        status = "service_packaging_or_test"
    elif links["step"] or links["archive"]:
        status = "inspect_linked_cad"
    elif candidates:
        status = "existing_family_candidate"
    elif product.get("specification_tables"):
        status = "new_family_or_import_needed"
    else:
        status = "dimensional_evidence_needed"
    return {"supplier_product_id": product["supplier_product_id"], "name": name,
            "url": product["url"], "variant_count": variant_count, "status": status,
            "candidate_families": ["machinekit." + c.replace("/", ".") for c in candidates],
            "has_specification_tables": bool(product.get("specification_tables")),
            "links": links, "cad_download_verified": False, "compatibility": "unverified"}


def fastener_mappings(catalog, family_sku="1131500"):
    """Join exact SKU/table rows for the identified socket-head screw families."""
    stainless = family_sku == "1131500"
    family_name = "Socket Bolts INOX A2" if stainless else "Socket Bolts"
    parents = [p for p in catalog["products"] if p["sku"] == family_sku
               and unescape(p["name"]) == family_name]
    if len(parents) != 1:
        raise ValueError("Expected one Vallder " + family_name + " family")
    parent = parents[0]
    if not stainless and not re.search(r"carbon steel\s+8\.8\s+grade", parent["short_description_html"], re.I):
        raise ValueError("Carbon steel property-class evidence changed")
    header = ["Product", "d", "L", "dk", "k", "Material / Class", "Standard", "SKU"] if stainless else [
        "Product", "dk", "k", "b", "L", "d", "SKU", "Drawing"]
    tables = [t for t in parent["specification_tables"]
              if t and t[0][:8] == header]
    if len(tables) != 1:
        raise ValueError("Fastener table layout changed; review before mapping")
    rows = {}
    for row in tables[0][1:]:
        if len(row) < (8 if stainless else 7):
            continue
        sku = row[7 if stainless else 6]
        if sku in rows:
            raise ValueError("Duplicate fastener table SKU")
        rows[sku] = row
    mappings = []
    reference = json.loads(Path(__file__).with_name("fastener-reference.json").read_text())
    for variant in catalog["variants"]:
        if variant["parent_id"] != parent["supplier_product_id"]:
            continue
        row = rows.get(variant["sku"])
        if row is None or (stainless and (row[5] != "A2 (304) / A2-70" or row[6] != "EN ISO 4762")):
            raise ValueError(f"Missing or changed dimensional evidence for {variant['sku']}")
        size, length, head_diameter, head_height = [row[i] for i in ((1,2,3,4) if stainless else (5,4,1,2))]
        match = re.fullmatch(r"Socket-Bolt-(M[0-9]+)x([0-9]+)" + ("-INOX-A2" if stainless else ""), row[0])
        if not match or match[1] != size or float(match[2]) != float(length):
            raise ValueError("Fastener name and dimensional row disagree")
        terms = [term for attribute in parent["attributes"] for term in attribute.get("terms", [])
                 if term["name"] == row[0]]
        if len(terms) != 1 or not any(a["value"] == terms[0]["slug"]
                                   for a in variant["option_attributes"]):
            raise ValueError("Variant options do not match dimensional row")
        limits = reference["sizes"][size]
        head_check = {"source": limits["source"], "scope": reference["scope"],
                      "reference_head_diameter_mm": limits["head_diameter_mm"],
                      "reference_head_height_mm": limits["head_height_mm"],
                      "diameter_within_reference": limits["head_diameter_mm"][0] <= float(head_diameter) <= limits["head_diameter_mm"][1],
                      "height_within_reference": limits["head_height_mm"][0] <= float(head_height) <= limits["head_height_mm"][1]}
        mappings.append({"type_id": "machinekit.standard.socket-head-cap-screw",
            "values": {"size": size, "length": float(length), "propertyClass": "A2-70" if stainless else "8.8"},
            "supplier_id": "vallder", "supplier_product_id": variant["supplier_product_id"],
            "sku": variant["sku"], "url": variant["url"], "price": variant["price"],
            "source": variant["source"],
            "availability": variant["availability"], "source_url": parent["url"],
            "head_dimension_check": head_check,
            "evidence": {"table_header": tables[0][0], "table_row": row,
                         "head_diameter_mm": float(head_diameter), "head_height_mm": float(head_height),
                         "standard_claimed_by_supplier": stainless,
                         "property_class_source": row[5] if stainless else parent["short_description_html"]},
            "compatibility": "review_required", "geometry": "generic_nominal"})
    if len(mappings) != len(rows):
        raise ValueError("Fastener table and variant coverage differ")
    return mappings


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--catalog", type=Path, default=DEFAULT_OUTPUT / "catalog.json")
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()
    catalog = json.loads(args.catalog.read_text())
    counts = Counter(v["parent_id"] for v in catalog["variants"])
    entries = [audit_product(p, counts[p["supplier_product_id"]]) for p in catalog["products"]]
    summary = dict(Counter(p["status"] for p in entries))
    report = {"schema_version": 1, "generated_at": now(), "catalog_generated_at": catalog["generated_at"],
              "catalog_sources": catalog["sources"], "summary": summary, "products": entries}
    write_json(args.output / "audit.json", report)
    mappings = fastener_mappings(catalog) + fastener_mappings(catalog,"1131000")
    write_json(args.output / "fasteners.json", {"schema_version": 1,
               "catalog_sources": catalog["sources"], "mappings": mappings})
    with (args.output / "audit.csv").open("w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(["product_id", "name", "variants", "status", "candidate_families", "cad_links", "url"])
        for p in entries:
            writer.writerow([p["supplier_product_id"], p["name"], p["variant_count"], p["status"],
                             "; ".join(p["candidate_families"]),
                             "; ".join(p["links"]["step"] + p["links"]["archive"]), p["url"]])
    print(json.dumps({"summary": summary, "fastener_mappings": len(mappings),
                      "output": str(args.output)}, indent=2))


if __name__ == "__main__":
    main()
