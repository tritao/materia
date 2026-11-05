package machinekit.cover;

import machinekit.component.BomItem;
import machinekit.component.BomRevision;
import machinekit.component.Procurement.ProcurementCodec;

/** What a wiper does for a guide block. */
enum WayWiperKind {
  /** The block's own double-lip end seal (what most catalogue blocks already carry, bought as a spare or added to a block without). */
  EndSeal;
  /** A metal scraper plate in front of the end seal: it takes the swarf off the rail before the seal sees it. */
  MetalScraper;
}

/**
 * A wiper bought for the blocks of a profile rail: a row says which carriage profile it fits, what it adds to the block's length at each
 * end, and where the values come from. The values are the family's typical ones (`basis`), labelled assumed until a design buys against a
 * maker's drawing.
 */
typedef WayWiperRow = {
  var id:String;
  var kind:WayWiperKind;
  var description:String;
  /** The carriage designation it fits (`LinearRailBlock` profile). */
  var profile:String;
  /** Length one wiper adds to the block at its end, mm. */
  var added:Float;
  var basis:String;
}

/** The block wipers a design can choose from, and the BOM line for a set of them. */
class WayWiperCatalog {
  static var cached:Null<Array<WayWiperRow>>;

  public static function rows():Array<WayWiperRow> {
    if (cached == null) cached = [
      {
        id: "WIPER-HG15-SCRAPER", kind: MetalScraper, description: "Metal scraper plate with felt wiper for an HG15 carriage, one per block end",
        profile: "HGH15CA", added: 2.5,
        basis: "HIWIN HG accessory family (end seal, double seal, metal scraper); added length assumed, check against the HIWIN catalogue before buying"
      },
      {
        id: "WIPER-HG15-DOUBLE-SEAL", kind: EndSeal, description: "Double-lip end seal for an HG15 carriage, one per block end",
        profile: "HGH15CA", added: 3.0,
        basis: "HIWIN HG accessory family (double seal DD); added length assumed, check against the HIWIN catalogue before buying"
      }
    ];
    return cached.copy();
  }

  public static function ids():Array<String> return [for (row in rows()) row.id];

  public static function get(id:String):WayWiperRow {
    for (row in rows()) if (row.id == id) return row;
    throw 'No wiper row "$id"; rows: ${ids().join(", ")}';
  }

  /** One BOM line for `ends` block ends fitted with wiper `id`. */
  public static function bomItem(id:String, ends:Int):BomItem {
    var row = get(id);
    return {
      partNumber: row.id, description: row.description, quantity: ends, material: null,
      fingerprint: BomRevision.fingerprint('way-wiper|${row.id}|${row.profile}', 1),
      procurement: Buy(null, null, null), maturity: "Assumed",
      requirements: [ProcurementCodec.choice("profile", row.profile), ProcurementCodec.choice("kind", Std.string(row.kind))]
    };
  }
}
