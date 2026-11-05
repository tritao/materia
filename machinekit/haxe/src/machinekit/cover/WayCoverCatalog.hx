package machinekit.cover;

enum WayCoverKind {
  /** Folded bellows: compressed length per fold, extended up to `ratio` times the compressed length. */
  Bellows;
  /** Nested rigid sections that slide out of each other. */
  Telescopic;
}

/**
 * One way cover (bellows or telescopic) as a maker lists it: how long it is folded and extended, the thickness of its skin and the
 * flange it bolts down by. The values are typical catalogue values of the family (`basis`), not one maker's datasheet; a
 * design that buys against a maker's row replaces the row.
 */
typedef WayCoverRow = {
  var id:String;
  /** Compact family code for readable purchase numbers; custom rows may retain their id. */
  @:optional var purchaseCode:String;
  var kind:WayCoverKind;
  var description:String;
  /** The skin: a `MaterialLibrary` spec. */
  var material:String;
  /** Bellows: compressed length per fold, mm. */
  var foldCompressed:Float;
  /** Bellows: extended length over compressed length, at most. */
  var ratio:Float;
  /** Telescopic: length of one section, mm, and the overlap two neighbours keep when extended. */
  var sectionLength:Float;
  var overlap:Float;
  /** Telescopic: how much longer each further section makes the folded stack (its end fold and the stop that holds the next), mm. */
  var stackStep:Float;
  /** Skin thickness, mm. */
  var wall:Float;
  /** Radial clearance between a skin and the next one that moves relative to it, mm. */
  var gap:Float;
  /** Width of the mounting lip of the end flanges, mm, and their thickness. */
  var lip:Float;
  var flangeThickness:Float;
  /** Mounting screw and the clearance hole of its flange, and the most the flange holes are apart, mm. */
  var screw:String;
  var clearance:Float;
  var holePitch:Float;
  /** Thickness of the wiper lip at the moving end, mm. */
  var wiper:Float;
  /** Fastest travel speed the maker allows, mm/s. */
  var maxSpeed:Float;
  var basis:String;
}

/** The way covers a design can choose from. */
class WayCoverCatalog {
  static var cached:Null<Array<WayCoverRow>>;

  public static function rows():Array<WayCoverRow> {
    if (cached == null) cached = [
      {
        id: "BELLOWS-PU-6x8", purchaseCode: "P6x8", kind: Bellows, description: "Flat bellows, polyurethane-coated polyester, 6 mm per fold, 1:8", material: "polyurethane PU",
        foldCompressed: 6, ratio: 8, sectionLength: 0, overlap: 0, stackStep: 0, wall: 0.8, gap: 1.2, lip: 14, flangeThickness: 3, screw: "M5", clearance: 5.5,
        holePitch: 150, wiper: 6, maxSpeed: 1000, basis: "Typical flat-bellows family values (assumed)"
      },
      {
        id: "BELLOWS-PU-8x10", purchaseCode: "P8x10", kind: Bellows, description: "Flat bellows, polyurethane-coated polyester, 8 mm per fold, 1:10", material: "polyurethane PU",
        foldCompressed: 8, ratio: 10, sectionLength: 0, overlap: 0, stackStep: 0, wall: 1.0, gap: 1.2, lip: 16, flangeThickness: 3, screw: "M5", clearance: 5.5,
        holePitch: 150, wiper: 6, maxSpeed: 1000, basis: "Typical flat-bellows family values (assumed)"
      },
      {
        id: "TELESCOPIC-SS-250", purchaseCode: "TSS250", kind: Telescopic, description: "Telescopic steel cover, 250 mm sections, 1.5 mm stainless", material: "stainless A2-70",
        foldCompressed: 0, ratio: 0, sectionLength: 250, overlap: 30, stackStep: 10, wall: 1.5, gap: 1.5, lip: 18, flangeThickness: 4, screw: "M6", clearance: 6.6,
        holePitch: 200, wiper: 8, maxSpeed: 1500, basis: "Typical telescopic-cover family values (assumed)"
      },
      {
        id: "BELLOWS-LAMELLA-8x8", purchaseCode: "L8x8", kind: Bellows, description: "Flat bellows with steel lamellas, polyester-PVC fabric, 8 mm per fold, 1:8, chip-proof",
        material: "polyurethane PU", foldCompressed: 8, ratio: 8, sectionLength: 0, overlap: 0, stackStep: 0, wall: 1.0, gap: 1.1, lip: 12, flangeThickness: 3,
        screw: "M5", clearance: 5.5, holePitch: 120, wiper: 6, maxSpeed: 1500,
        basis: "Typical lamella-bellows family values for machine-tool ways (assumed): steel lamellas sewn into the fabric every fold"
      },
      {
        id: "TELESCOPIC-SS-150", purchaseCode: "TSS150", kind: Telescopic, description: "Telescopic steel cover, 150 mm sections, 1.5 mm stainless, brush wipers",
        material: "stainless A2-70", foldCompressed: 0, ratio: 0, sectionLength: 150, overlap: 20, stackStep: 12, wall: 1.5, gap: 1.5, lip: 18,
        flangeThickness: 4, screw: "M6", clearance: 6.6, holePitch: 150, wiper: 8, maxSpeed: 1500,
        basis: "Typical telescopic-cover family values (assumed): shorter sections than the 250 mm row for shorter strokes"
      }
    ];
    return cached.copy();
  }

  public static function ids():Array<String> return [for (row in rows()) row.id];

  public static function get(id:String):WayCoverRow {
    for (row in rows()) if (row.id == id) return row;
    throw 'No way cover row "$id"; rows: ${ids().join(", ")}';
  }
}
