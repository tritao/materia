#!/usr/bin/env python3
"""Join CAD validation, generator recipes and unresolved geometry/interface requirements."""
from collections import Counter
import csv
import json
from pathlib import Path

from vallder import DEFAULT_OUTPUT, now, write_json
from hardware import build as hardware_review
from motor_mounts import build as motor_mount_review


def build(output):
    catalog = json.loads((output / "catalog.json").read_text())
    audit = json.loads((output / "audit.json").read_text())
    inventory = json.loads((output / "cad/inventory.json").read_text())
    validation = json.loads((output / "cad/validation.json").read_text())
    fasteners = json.loads((output / "fasteners.json").read_text())["mappings"]
    checked = {a["asset_id"]: a for a in validation["assets"]}
    recipes = [{"supplier_id": m["supplier_id"], "sku": m["sku"], "supplier_product_id": m["supplier_product_id"],
        "parent_id": next(v["parent_id"] for v in catalog["variants"] if v["supplier_product_id"] == m["supplier_product_id"]),
        "type_id": m["type_id"], "values": m["values"], "required_inputs": [],
        "geometry_source": "generator", "compatibility": "review_required",
        "head_dimension_check": m["head_dimension_check"], "source_url": m["source_url"]} for m in fasteners]
    # Rail-only SKU: length is a design input, not an invented supplier variant.
    parent = next(p for p in catalog["products"] if p["supplier_product_id"] == 39276)
    rail_row = next(row for table in parent["specification_tables"] for row in table if row and row[0] == "HGR_15")
    rail_variants = [v for v in catalog["variants"] if v["parent_id"] == parent["supplier_product_id"] and v["sku"] == rail_row[-1]]
    if len(rail_variants) != 1 or [rail_row[i] for i in (4,5,9,10)] != ["15", "15", "60", "20"]:
        raise ValueError("HGR15 evidence changed; review binding")
    rail = rail_variants[0]
    recipes.append({"supplier_id": "vallder", "sku": rail["sku"], "supplier_product_id": rail["supplier_product_id"],
        "parent_id": parent["supplier_product_id"], "type_id": "machinekit.motion.linear-rail",
        "values": {"profile": "HGR15"}, "required_inputs": ["length"], "geometry_source": "generator",
        "compatibility": "review_required", "source_url": parent["url"], "table_row": rail_row,
        "limits": ["Default first hole is 20 mm from the end; custom/symmetrical cutting is not represented",
                   "Cut length and cutting charges require a quote; per-metre price is not a per-piece price",
                   "Generic running section is not a manufacturer-matched raceway"]})
    hardware = hardware_review(catalog)
    recipes.extend(hardware["recipes"])
    write_json(output / "hardware.json", hardware)
    write_json(output / "recipes.json", {"schema_version": 1, "recipes": recipes})
    by_parent = {}
    for recipe in recipes:
        by_parent.setdefault(recipe["parent_id"], []).append(recipe)
    profile_path, profile_validation = output / "profiles.json", output / "profile-validation.json"
    sections = json.loads(profile_path.read_text())["profiles"] if profile_path.exists() else []
    section_checks = json.loads(profile_validation.read_text()) if profile_validation.exists() else []
    ready_sections = {section["product_id"]: section for section in sections if any(
        check["productId"] == section["product_id"] and check["sourceSha256"] == section["source_sha256"] and
        check.get("familyId") == section.get("family_id") and check.get("systemId") == section.get("system_id") and
        check["status"] == "valid_parametric_geometry" for check in section_checks)}
    library = []
    for download in inventory["downloads"]:
        for asset in download.get("assets", []):
            result = checked.get(asset["id"], {"status": "not_validated"})
            usable = result["status"] == "valid_solid_geometry" and asset["unit_status"] == "declared_mm"
            for reference in download["references"]:
                variant_ids = sorted({v for context in reference["table_contexts"] for v in context["variant_ids"]})
                library.append({"asset_id": asset["id"], "file": asset["file"], "sha256": asset["sha256"],
                    "source_url": download["url"], "archive_sha256": download["sha256"],
                    "archive_member": asset["archive_member"], "fetched_at": download["fetched_at"],
                    "product_id": reference["product_id"], "variant_ids": variant_ids,
                    "identity_status": "table_linked" if variant_ids else "product_reference_only",
                    "geometry_status": result["status"], "usable_geometry": usable,
                    "length_unit": result.get("lengthUnit"), "declared_units": asset["declared_si_length_prefixes"],
                    "dimensions_mm": result.get("dimensions"), "solids": result.get("solids"),
                    "assembly_occurrences": asset["assembly_occurrences"],
                    "mounting_frame_status": "unverified", "compatibility_verified": False,
                    "table_evidence": reference["table_contexts"]})
    # RBK downloads join through exact columns in its transposed specification table.
    motor_mounts = motor_mount_review(catalog, library)
    write_json(output / "motor-mounts.json", motor_mounts)
    # BK/BF files use the established X-across/Y-up/Z-shaft reference frame.
    # Validate the base plane against h and envelope against B/H; do not infer bores from splines.
    support_interfaces = []
    for parent_id in (2348, 2378):
        support_parent = next(p for p in catalog["products"] if p["supplier_product_id"] == parent_id)
        table = support_parent["specification_tables"][0]
        width_column, height_column = table[0].index("B"), table[0].index("H")
        h_column = next(i for i, name in enumerate(table[0]) if name.startswith("h"))
        for row in table[1:]:
            if len(row) < len(table[0])-1:
                continue
            variant = next((v for v in catalog["variants"] if v["parent_id"] == parent_id and v["sku"] == row[-1]), None)
            if variant is None:
                continue
            matching = [a for a in library if a["product_id"] == parent_id and variant["supplier_product_id"] in a["variant_ids"]]
            for asset in matching:
                result = checked.get(asset["asset_id"], {})
                if result.get("status") != "valid_solid_geometry":
                    continue
                width, height, shaft_height = [float(row[i].replace(",", ".")) for i in (width_column,height_column,h_column)]
                planes = [p for p in result.get("planes", []) if abs(p["centre"][1]+shaft_height)<0.02 and
                          abs(p["normal"][0])<1e-6 and abs(p["normal"][1]+1)<1e-6 and abs(p["normal"][2])<1e-6]
                dimensions = result["dimensions"]
                entry = {"sku": variant["sku"], "type": row[0], "asset_id": asset["asset_id"],
                    "table_row": row, "table_url": support_parent["url"],
                    "base_plane_status": "matches_table" if planes else "unverified",
                    "base_plane_y_mm": -shaft_height, "base_plane_face_indices": [p["face"] for p in planes],
                    "width_comparison": {"table_mm": width, "cad_mm": dimensions[0], "matches": abs(width-dimensions[0])<0.02},
                    "height_comparison": {"table_mm": height, "cad_mm": dimensions[1], "matches": abs(height-dimensions[1])<0.02},
                    "mounting_bores_status": "unverified", "shaft_fit_status": "unverified", "compatibility_verified": False}
                support_interfaces.append(entry)
                asset["interface_audit"] = entry
    write_json(output / "cad/library.json", {"schema_version": 1, "generated_at": now(), "assets": library})
    write_json(output / "support-interface-review.json", {"scope": "Base-plane and envelope checks only; mounting bores and shaft fit unqualified",
               "supports": support_interfaces})
    products = []
    for entry in audit["products"]:
        product_id = entry["supplier_product_id"]
        assets = [a for a in library if a["product_id"] == product_id]
        valid = [a for a in assets if a["usable_geometry"]]
        family_recipes = by_parent.get(product_id, [])
        drawing_section = ready_sections.get(product_id)
        gaps = []
        if family_recipes or drawing_section:
            status = "generator_available"
            gaps.append("Supplier compatibility remains unverified")
        elif valid:
            status = "supplier_geometry_available"
            gaps.append("Mounting frames and mechanical properties need qualification")
        elif entry["status"] == "service_packaging_or_test":
            status = entry["status"]
        else:
            status = "geometry_gap"
            gaps.append("No validated STEP or supplier-bound generator recipe")
        if entry["name"].startswith("Alu Profile") and not valid and not drawing_section:
            gaps.append("Linked files are DXF sections; extract measured slot/bore dimensions before using TSlotExtrusion")
        if drawing_section:
            gaps.append("Drawing-derived section: explicit cut length and material/alloy qualification required; price per metre needs a quote")
        if product_id in (2348, 2378):
            gaps.append("Existing ScrewSupportUnit assumes BK12 height 43 / BF12 bore 12; table states BK12 height 42 / BF12 bore 10")
        if assets and not valid:
            gaps.append("See native validation results before using linked CAD")
        variants = [v for v in catalog["variants"] if v["parent_id"] == product_id]
        variant_coverage = []
        for variant in variants:
            recipe = next((r for r in family_recipes if r["supplier_product_id"] == variant["supplier_product_id"]), None)
            matched_assets = sorted({a["asset_id"] for a in valid if variant["supplier_product_id"] in a["variant_ids"]})
            variant_coverage.append({"id": variant["supplier_product_id"], "sku": variant["sku"],
                "geometry_source": "generator" if recipe else "supplier_cad" if matched_assets else "unresolved",
                "asset_ids": matched_assets, "compatibility_verified": False})
        products.append({"product_id": product_id, "name": entry["name"], "url": entry["url"], "status": status,
            "generator_skus": [r["sku"] for r in family_recipes] + ([drawing_section["sku"]] if drawing_section else []),
            "geometry_source": ("tslot_section_variant" if drawing_section.get("system_id") else "drawing_extrusion") if drawing_section else "generator" if family_recipes else None,
            "profile_family_id": drawing_section.get("family_id") if drawing_section else None,
            "validated_asset_ids": sorted({a["asset_id"] for a in valid}),
            "candidate_families": entry["candidate_families"], "gaps": gaps, "variants": variant_coverage})
    summary = {"products": dict(Counter(p["status"] for p in products)),
        "variants": dict(Counter(v["geometry_source"] for p in products for v in p["variants"])),
        "downloads": dict(Counter(d["status"] for d in inventory["downloads"])),
        "unique_step_assets": len({a["asset_id"] for a in library}),
        "validated_unique_assets": len({a["asset_id"] for a in library if a["usable_geometry"]}),
        "registered_generator_bindings": len(recipes), "validated_drawing_generators": len(ready_sections),
        "shared_tslot_families": len({s["family_id"] for s in ready_sections.values() if s.get("system_id")}),
        "all_inventory_complete": inventory["complete"] and validation["complete"] and
                                  {a["id"] for d in inventory["downloads"] for a in d.get("assets", [])} == set(checked)}
    write_json(output / "coverage.json", {"schema_version": 1, "generated_at": now(), "summary": summary, "products": products})
    with (output / "coverage.csv").open("w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(["product_id", "name", "status", "generator_skus", "valid_step_assets", "unresolved_variants", "gaps"])
        for p in products:
            writer.writerow([p["product_id"], p["name"], p["status"], ";".join(p["generator_skus"]),
                             len(p["validated_asset_ids"]), sum(v["geometry_source"] == "unresolved" for v in p["variants"]),
                             "; ".join(p["gaps"])])
    write_json(output / "fastener-dimension-review.json", {"scope": "Head dimensions only; other interfaces remain unverified",
        "summary": dict(Counter("within_reference" if m["head_dimension_check"]["diameter_within_reference"] and
                               m["head_dimension_check"]["height_within_reference"] else "conflict" for m in fasteners)),
        "variants": [{"sku": m["sku"], "values": m["values"], "supplier_evidence": m["evidence"],
                      "check": m["head_dimension_check"]} for m in fasteners]})
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    build(parser.parse_args().output)
