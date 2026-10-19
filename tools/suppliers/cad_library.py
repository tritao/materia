#!/usr/bin/env python3
"""Download supplier CAD references and inventory STEP assets without executing archives."""
import argparse
from collections import Counter
import hashlib
from html.parser import HTMLParser
import io
import json
from pathlib import Path
import re
import time
from urllib.error import HTTPError, URLError
from urllib.parse import urljoin, urlsplit
from urllib.request import Request, urlopen
import zipfile

from vallder import DEFAULT_OUTPUT, now, write_json

MAX_DOWNLOAD = 64 * 1024 * 1024
MAX_MEMBER = 320 * 1024 * 1024
MAX_EXPANDED = 512 * 1024 * 1024


class LinkRows(HTMLParser):
    """Retain the exact table row surrounding a download link for SKU joins."""
    def __init__(self):
        super().__init__(convert_charrefs=True)
        self.rows, self.text, self.links = [], None, []

    def handle_starttag(self, tag, attrs):
        if tag == "tr":
            self.text, self.links = [], []
        if tag == "a" and self.text is not None:
            url = dict(attrs).get("href")
            if url:
                self.links.append(url)

    def handle_data(self, data):
        if self.text is not None:
            self.text.append(data)

    def handle_endtag(self, tag):
        if tag == "tr" and self.text is not None:
            self.rows.append({"text": " ".join(" ".join(self.text).split()), "links": self.links})
            self.text = None


def link_contexts(product, variants):
    parser = LinkRows()
    parser.feed(product.get("description_html", ""))
    contexts = {}
    for row in parser.rows:
        # Whole numeric/text SKU boundaries, never a substring of another SKU.
        matching = [v for v in variants if v.get("sku") and
                    re.search(r"(?<![\w-])" + re.escape(v["sku"]) + r"(?![\w-])", row["text"])]
        for url in row["links"]:
            contexts.setdefault(urljoin(product["url"], url), []).append({
                "table_row": row["text"], "variant_ids": [v["supplier_product_id"] for v in matching],
                "supplier_skus": [v["sku"] for v in matching]})
    return contexts


def download(url, directory, refresh=False, offline=False):
    key = hashlib.sha256(url.encode()).hexdigest()
    meta_path, data_path = directory / (key + ".json"), directory / (key + ".bin")
    if not refresh and meta_path.exists() and data_path.exists():
        meta, data = json.loads(meta_path.read_text()), data_path.read_bytes()
        if meta["url"] == url and hashlib.sha256(data).hexdigest() == meta["sha256"]:
            return meta, data
        raise ValueError("Downloaded cache hash mismatch: " + url)
    if offline:
        raise ValueError("Missing cached download: " + url)
    for attempt in range(3):
        try:
            request = Request(url, headers={"User-Agent": "MateriaSupplierImporter/1.0"})
            with urlopen(request, timeout=40) as response:
                if int(response.headers.get("Content-Length", 0)) > MAX_DOWNLOAD:
                    raise ValueError("Download exceeds 64 MiB limit")
                data = response.read(MAX_DOWNLOAD + 1)
                if len(data) > MAX_DOWNLOAD:
                    raise ValueError("Download exceeds 64 MiB limit")
                meta = {"url": url, "resolved_url": response.url, "fetched_at": now(),
                        "sha256": hashlib.sha256(data).hexdigest(), "bytes": len(data),
                        "content_type": response.headers.get("Content-Type"), "cache_file": data_path.name}
            directory.mkdir(parents=True, exist_ok=True)
            temporary = data_path.with_suffix(".tmp")
            temporary.write_bytes(data)
            temporary.replace(data_path)
            write_json(meta_path, meta)
            return meta, data
        except (HTTPError, URLError, TimeoutError) as error:
            if isinstance(error, HTTPError) and error.code < 500 and error.code != 429:
                raise
            if attempt == 2:
                raise
            time.sleep(2 ** attempt)


def step_metadata(data):
    text = data.decode("latin-1")
    if not text.lstrip().startswith("ISO-10303-21;"):
        raise ValueError("STEP member does not have a Part 21 header")
    # Record declared SI length units; native validation reports geometry in mm.
    units = sorted(set(re.findall(r"SI_UNIT\s*\(\s*(\$|\.[A-Z]+\.)\s*,\s*\.METRE\.\s*\)", text, re.I)))
    return {"declared_si_length_prefixes": units,
            "has_conversion_based_units": "CONVERSION_BASED_UNIT" in text.upper(),
            "assembly_occurrences": len(re.findall(r"\bNEXT_ASSEMBLY_USAGE_OCCURRENCE\s*\(", text, re.I)),
            "unit_status": "declared_mm" if units == [".MILLI."] else "review_required"}


def inventory(data, url, asset_directory):
    """Only STEP bytes are materialized, under content hashes, never archive paths."""
    members, assets = [], []
    def member(name, payload):
        extension = Path(name).suffix.lower()
        row = {"name": name, "bytes": len(payload), "format": extension.lstrip(".")}
        if extension in (".step", ".stp"):
            digest = hashlib.sha256(payload).hexdigest()
            meta = step_metadata(payload)
            asset_directory.mkdir(parents=True, exist_ok=True)
            path = asset_directory / (digest + ".step")
            path.write_bytes(payload)
            asset = {"id": digest, "sha256": digest, "file": "cad/assets/" + path.name,
                     "archive_member": name, **meta}
            assets.append(asset)
            row["asset_id"] = digest
        members.append(row)
    if zipfile.is_zipfile(io.BytesIO(data)):
        with zipfile.ZipFile(io.BytesIO(data)) as archive:
            infos = [info for info in archive.infolist() if not info.is_dir()]
            if sum(info.file_size for info in infos) > MAX_EXPANDED:
                raise ValueError("Archive exceeds 512 MiB expanded limit")
            if len(infos) > 10000:
                raise ValueError("Archive has too many entries")
            for info in infos:
                if info.file_size > MAX_MEMBER:
                    members.append({"name": info.filename, "bytes": info.file_size, "status": "too_large"})
                elif Path(info.filename).suffix.lower() in (".step", ".stp"):
                    member(info.filename, archive.read(info))
                else:
                    members.append({"name": info.filename, "bytes": info.file_size,
                                    "format": Path(info.filename).suffix.lower().lstrip(".")})
        kind = "zip"
    elif data.lstrip().startswith(b"ISO-10303-21;"):
        member(Path(urlsplit(url).path).name, data)
        kind = "step"
    else:
        raise ValueError("Download is neither a ZIP nor a STEP file")
    return {"format": kind, "members": members, "assets": assets}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--refresh", action="store_true")
    mode.add_argument("--offline", action="store_true")
    args = parser.parse_args()
    catalog = json.loads((args.output / "catalog.json").read_text())
    audit = json.loads((args.output / "audit.json").read_text())
    products = {p["supplier_product_id"]: p for p in catalog["products"]}
    downloads = {}
    for product in audit["products"]:
        product_id = product["supplier_product_id"]
        variants = [v for v in catalog["variants"] if v["parent_id"] == product_id]
        contexts = link_contexts(products[product_id], variants)
        for url in product["links"]["step"] + product["links"]["archive"]:
            downloads.setdefault(url, []).append({"product_id": product_id,
                "product_name": product["name"], "product_url": product["url"],
                "table_contexts": contexts.get(url, [])})
    results = []
    for i, (url, references) in enumerate(downloads.items()):
        result = {"url": url, "references": references}
        try:
            meta, data = download(url, args.output / "cad/downloads", args.refresh, args.offline)
            result.update(meta)
            result.update(inventory(data, url, args.output / "cad/assets"))
            limited = any(m.get("status") == "too_large" for m in result["members"])
            result["status"] = ("partial_step_available" if limited else "step_available") if result["assets"] else (
                "member_size_limit" if limited else "no_step_members")
        except (OSError, ValueError, zipfile.BadZipFile, RuntimeError) as error:
            result.update({"status": "failed", "error": str(error)})
        results.append(result)
        # Checkpoint each completed source so interrupted runs remain inspectable.
        write_json(args.output / "cad/inventory.json", {"schema_version": 1,
            "generated_at": now(), "complete": len(results) == len(downloads),
            "source_count": len(downloads), "downloads": results})
        print(f"[{i+1}/{len(downloads)}] {result['status']}: {url}", flush=True)
        if not args.offline:
            time.sleep(0.3)
    print(json.dumps(dict(Counter(r["status"] for r in results)), indent=2))


if __name__ == "__main__":
    main()
