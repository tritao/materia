#!/usr/bin/env python3
"""Read-only Vallder Store API importer; Python standard library only."""

import argparse
from datetime import datetime, timezone
from decimal import Decimal
import hashlib
from html.parser import HTMLParser
import json
from pathlib import Path
import time
from urllib.error import HTTPError, URLError
from urllib.parse import urlencode
from urllib.request import Request, urlopen
import xml.etree.ElementTree as ET


BASE = "https://vallder.com"
DEFAULT_OUTPUT = Path(__file__).resolve().parents[2] / "build/suppliers/vallder"


def now():
    return datetime.now(timezone.utc).isoformat()


def write_json(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2) + "\n")
    temporary.replace(path)


class Client:
    def __init__(self, directory, refresh=False, offline=False, delay=0.5):
        self.directory = directory
        self.refresh = refresh
        self.offline = offline
        self.delay = delay
        self.last_request = 0

    def get(self, url):
        path = self.directory / (hashlib.sha256(url.encode()).hexdigest() + ".json")
        if path.exists() and not self.refresh:
            result = json.loads(path.read_text())
            if result["url"] != url:
                raise ValueError("Cache URL mismatch")
            return result
        if self.offline:
            raise ValueError(f"Missing cached response: {url}")
        for attempt in range(4):
            time.sleep(max(0, self.delay - (time.monotonic() - self.last_request)))
            self.last_request = time.monotonic()
            try:
                request = Request(url, headers={"User-Agent": "MateriaSupplierImporter/1.0",
                                               "Accept": "application/json, application/xml"})
                with urlopen(request, timeout=45) as response:
                    result = {"url": url, "fetched_at": now(),
                              "headers": {k.lower(): v for k, v in response.headers.items()},
                              "body": response.read().decode("utf-8")}
                write_json(path, result)
                return result
            except (HTTPError, URLError, TimeoutError) as error:
                if isinstance(error, HTTPError) and error.code != 429 and error.code < 500:
                    raise
                if attempt == 3:
                    raise
                retry_after = error.headers.get("Retry-After", "") if isinstance(error, HTTPError) else ""
                time.sleep(max(2 ** attempt, int(retry_after) if retry_after.isdigit() else 0))


def collection(client, kind=None):
    rows, sources = [], []
    page, total, pages = 1, None, None
    while True:
        params = {"per_page": 100, "page": page, "orderby": "id", "order": "asc"}
        if kind:
            params["type"] = kind
        response = client.get(BASE + "/wp-json/wc/store/v1/products?" + urlencode(params))
        data = json.loads(response["body"])
        if not isinstance(data, list):
            raise ValueError("Expected a product array")
        current_total = int(response["headers"]["x-wp-total"])
        current_pages = int(response["headers"]["x-wp-totalpages"])
        if total is None:
            total, pages = current_total, current_pages
        if (current_total, current_pages) != (total, pages):
            raise ValueError("Catalogue changed during pagination; rerun with --refresh")
        sources.append({"url": response["url"], "fetched_at": response["fetched_at"]})
        for product in data:
            product["_import_source"] = sources[-1]
        rows.extend(data)
        if page >= pages:
            break
        page += 1
    if len(rows) != total or len({p["id"] for p in rows}) != total:
        raise ValueError("Incomplete or duplicate product pagination")
    return rows, sources


class DescriptionParser(HTMLParser):
    """Preserve table rows as text, without guessing engineering field semantics."""
    def __init__(self):
        super().__init__(convert_charrefs=True)
        self.tables, self.links = [], []
        self.table = self.row = self.cell = None

    def handle_starttag(self, tag, attrs):
        attrs = dict(attrs)
        if tag == "table":
            self.table = []
        elif tag == "tr" and self.table is not None:
            self.row = []
        elif tag in ("td", "th") and self.row is not None:
            self.cell = []
        elif tag == "br" and self.cell is not None:
            self.cell.append(" ")
        if tag in ("a", "img"):
            url = attrs.get("href" if tag == "a" else "src")
            if url:
                self.links.append(url)

    def handle_data(self, data):
        if self.cell is not None:
            self.cell.append(data)

    def handle_endtag(self, tag):
        if tag in ("td", "th") and self.cell is not None:
            self.row.append(" ".join("".join(self.cell).split()))
            self.cell = None
        elif tag == "tr" and self.row is not None:
            self.table.append(self.row)
            self.row = None
        elif tag == "table" and self.table is not None:
            self.tables.append(self.table)
            self.table = None


def normalize(product, option_attributes=None):
    parser = DescriptionParser()
    parser.feed(product.get("description", ""))
    prices = product.get("prices", {})
    minor_unit = prices.get("currency_minor_unit")
    def amount(value):
        if value in (None, "") or minor_unit is None:
            return None
        return format(Decimal(value).scaleb(-minor_unit), "f")
    return {
        "supplier_id": "vallder", "supplier_product_id": product["id"],
        "source": product.get("_import_source"),
        "parent_id": product.get("parent", 0), "type": product["type"],
        "name": product["name"], "sku": product.get("sku"), "url": product["permalink"],
        "variation_label": product.get("variation"),
        "option_attributes": option_attributes or [],
        "attributes": product.get("attributes", []), "categories": product.get("categories", []),
        "price": {"currency": prices.get("currency_code"), "amount": amount(prices.get("price")),
                  "regular_amount": amount(prices.get("regular_price")),
                  "range": {k: amount(v) for k, v in (prices.get("price_range") or {}).items()},
                  "tax_basis": "unspecified", "raw": prices},
        "availability": {k: product.get(k) for k in
                         ("is_in_stock", "is_on_backorder", "is_purchasable", "stock_availability", "low_stock_remaining")},
        "description_html": product.get("description", ""),
        "short_description_html": product.get("short_description", ""),
        "specification_tables": parser.tables,
        "description_links": sorted(set(parser.links)), "images": product.get("images", []),
        "reported_dimensions": product.get("dimensions"), "reported_weight": product.get("weight"),
        "engineering_verification": "unverified", "component_id": None,
    }


def import_catalog(client):
    products, sources = collection(client)
    variants, variant_sources = collection(client, "variation")
    parents = {p["id"] for p in products}
    options = {v["id"]: v.get("attributes", []) for p in products for v in p.get("variations", [])}
    if set(options) != {v["id"] for v in variants}:
        raise ValueError("Variant collection does not match parent variant references")
    if any(v.get("parent") not in parents for v in variants):
        raise ValueError("Variant references an unknown parent")
    sitemap = client.get(BASE + "/product-sitemap.xml")
    root = ET.fromstring(sitemap["body"])
    sitemap_urls = {node.text.rstrip("/") for node in root.findall(".//{*}loc")}
    product_urls = {p["permalink"].rstrip("/") for p in products}
    coverage = {"sitemap_count": len(sitemap_urls),
                "missing_from_api": sorted(sitemap_urls - product_urls),
                "missing_from_sitemap": sorted(product_urls - sitemap_urls)}
    return {"schema_version": 1, "supplier": {"id": "vallder", "name": "Vallder",
            "shop_url": BASE + "/pt/shop/", "api_url": BASE + "/wp-json/wc/store/v1/products"},
            "generated_at": now(), "sources": sources + variant_sources + [
                {"url": sitemap["url"], "fetched_at": sitemap["fetched_at"]}],
            "coverage": coverage, "counts": {"products": len(products), "variants": len(variants)},
            "products": [normalize(p) for p in products],
            "variants": [normalize(v, options.get(v["id"])) for v in variants]}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--refresh", action="store_true", help="Refetch cached responses")
    mode.add_argument("--offline", action="store_true", help="Use cached responses only")
    parser.add_argument("--delay", type=float, default=0.5, help="Minimum seconds between requests")
    args = parser.parse_args()
    if args.delay < 0:
        parser.error("--delay must be nonnegative")
    try:
        catalog = import_catalog(Client(args.output / "raw", args.refresh, args.offline, args.delay))
        write_json(args.output / "catalog.json", catalog)
    except (OSError, ValueError, KeyError, ET.ParseError) as error:
        parser.exit(1, f"Import failed; previous catalog retained: {error}\n")
    print(json.dumps({"output": str(args.output / "catalog.json"),
                      "counts": catalog["counts"], "coverage": catalog["coverage"]}, indent=2))


if __name__ == "__main__":
    main()
