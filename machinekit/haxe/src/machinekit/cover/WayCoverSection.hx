package machinekit.cover;

import cadkit.modeling.Part;
import machinekit.component.BoltedFace;
import machinekit.component.ComponentDetail;
import machinekit.component.ComponentRecipeSupport;
import machinekit.component.ComponentType;
import machinekit.component.ComponentValues;
import machinekit.component.Dimension;
import machinekit.component.MachineComponent;
import machinekit.component.Procurement.ProcurementCodec;
import machinekit.component.Procurement.ProcurementRequirement;
import machinekit.component.Solids;
import machinekit.cover.WayCoverCatalog.WayCoverRow;
import machinekit.cover.WayCoverPlan.CoverShape;

/** Which piece of a way cover a `WayCoverSection` is. */
enum WayCoverRole {
  /** The flange that bolts to the fixed member, with the first skin. */
  Fixed;
  /** A loose skin between the two. */
  Sleeve;
  /** The flange that bolts to the moving member, with the last skin and the wiper. */
  Moving;
}

/**
 * One piece of a way cover (`WayCover.install` places them): an end flange with its skin, or a loose skin. All pieces of one
 * cover are one purchase: the fixed flange carries the BOM line and the rest are included in it.
 *
 * Frame of a flange: the mounting plane at z = 0 with the cover's axis through the origin, +Z into the plate and on along the
 * skin, +X across the opening and +Y up; the moving flange is the fixed one turned to face the other way. A loose skin starts
 * at z = 0 and runs along +Z. The flange's holes are a clamped `BoltedFace` drilled to suit the host.
 */
class WayCoverSection extends MachineComponent {
  public final plan:WayCoverPlan;
  public final role:WayCoverRole;
  public final level:Int;

  public function new(row:WayCoverRow, role:WayCoverRole, level:Int, width:Float, height:Float, stroke:Float, space:Float, ?shape:CoverShape) {
    var plan = new WayCoverPlan(row, width, height, stroke, space, shape);
    if (level < 0 || level > plan.levels) throw 'Way cover ${row.id} has skins 0 to ${plan.levels}, not $level';
    if (role == Fixed && level != plan.fixedLevel()) throw 'The fixed flange carries skin ${plan.fixedLevel()}';
    if (role == Moving && level != plan.movingLevel()) throw 'The moving flange carries skin ${plan.movingLevel()}';
    if (role == Sleeve && (level < 1 || level >= plan.levels)) throw "A loose skin is neither the first nor the last";
    super(
      baseNumber(row, width, height, stroke, space, plan.shape) + (role == Fixed ? "" : role == Moving ? "-M" : '-S$level'),
      row.description + ", " + Dimension.format(width) + " x " + Dimension.format(height) + " opening, " + Dimension.format(stroke) + " mm stroke"
      + (plan.shape.open ? ", box cover" : "") + (role == Fixed ? "" : role == Moving ? " (moving flange and skin)" : ' (skin $level)'),
      row.material,
      true
    );
    this.plan = plan;
    this.role = role;
    this.level = level;
    addFacet(new machinekit.component.ProcurementIntent(Buy(null, null, null), selectionRequirements(
      row.id, width, height, stroke, space, plan.shape.open, plan.shape.reversed
    )));
    addFacet(new machinekit.component.EngineeringMetadata(new ComponentValues()
      .setToken("ratingsBasis", "Assumed").setToken("errorsBasis", "Assumed").setToken("source", row.basis)));
    if (role != Fixed) includedInPurchase(purchaseNumber(row, width, height, stroke, space, plan.shape));
  }

  /** Descriptive model designation. The shorter purchaseNumber is the BOM identity. */
  public static function baseNumber(row:WayCoverRow, width:Float, height:Float, stroke:Float, space:Float, ?shape:CoverShape):String
    return 'WAYCOVER-${row.id}-${Dimension.format(width)}x${Dimension.format(height)}-S${Dimension.format(stroke)}-L${Dimension.format(space)}'
      + (shape != null && shape.open ? "-BOX" : "") + (shape != null && shape.reversed ? "-R" : "");

  /** Readable purchase identity; full family and dimensional selection remain structured BOM requirements. */
  public static function purchaseNumber(row:WayCoverRow, width:Float, height:Float, stroke:Float, space:Float, ?shape:CoverShape):String {
    var code = row.purchaseCode;
    if (code == null) code = row.id;
    return 'WAYCOVER-$code-${Dimension.format(width)}x${Dimension.format(height)}-S${Dimension.format(stroke)}-L${Dimension.format(space)}'
      + (shape != null && shape.open ? "-B" : "") + (shape != null && shape.reversed ? "-R" : "");
  }

  /** The direct component's intent and its recipe describe the same purchased cover, including its family and section. */
  static function selectionRequirements(row:String, width:Float, height:Float, stroke:Float, space:Float, open:Bool,
      reversed:Bool):Array<ProcurementRequirement> return [
    ProcurementCodec.choice("row", row),
    ProcurementCodec.choice("section", open ? "Box" : "Ring"),
    ProcurementCodec.choice("nesting", reversed ? "GrowsToFixed" : "GrowsToMoving"),
    ProcurementCodec.requirement("width", "mm", null, null, width),
    ProcurementCodec.requirement("height", "mm", null, null, height),
    ProcurementCodec.requirement("stroke", "mm", null, null, stroke),
    ProcurementCodec.requirement("space", "mm", null, null, space)
  ];

  /** The face the flange bolts down by. */
  override public function fasteningFaces():Array<BoltedFace> {
    if (role == Sleeve) return [];
    var row = plan.row;
    if (holes().length < 3) throw 'Way cover flange ${designation} has ${holes().length} holes on its host: the host faces cover heights ${plan.shape.hostMin} to ${plan.shape.hostMax} mm, '
      + 'half width ${plan.shape.hostHalfWidth}; the plate is ${plan.plateHalf() * 2} wide and its roof lip is at ${plan.plateHigh() - row.lip / 2} mm';
    return [BoltedFace.clamped(
      role == Fixed ? "fixed" : "moving",
      BoltedFace.frameOf({x: 0, y: 0, z: 0}, {x: 0, y: 0, z: 1}, {x: 1, y: 0, z: 0}),
      holes(),
      row.clearance,
      row.flangeThickness,
      row.screw,
      null,
      false,
      true
    )];
  }

  /** Half the width of this flange's plate: the largest skin and its lip; the moving flange is cut back to its host face if that is narrower. */
  function plateHalf():Float return role == Moving ? plan.plateHalf() : plan.fullPlateHalf();

  /**
   * The flange holes in its face frame: along the middle of the lip, at most `holePitch` apart. An open cover has no bottom row; its
   * holes are only where the host face exists (`CoverShape.hostMin`/`hostMax`, heights above the surface), at least an edge distance in.
   */
  public function holes():Array<{x:Float, y:Float}> {
    var hx = plateHalf() - plan.row.lip / 2;
    var out:Array<{x:Float, y:Float}> = [];
    if (!plan.shape.open) {
      var hy = plan.outerHeight() / 2 - plan.row.lip / 2;
      var nx = Math.ceil(2 * hx / plan.row.holePitch), ny = Math.ceil(2 * hy / plan.row.holePitch);
      for (i in 0...nx + 1) for (sy in [-1, 1]) out.push({x: -hx + 2 * hx * i / nx, y: sy * hy});
      for (j in 1...ny) for (sx in [-1, 1]) out.push({x: sx * hx, y: -hy + 2 * hy * j / ny});
      return out;
    }
    var edge = 1.5 * Std.parseFloat(plan.row.screw.substr(1)) + 0.5, hostLow = plan.shape.hostMin + edge, hostHigh = plan.shape.hostMax - edge;
    var roof = plan.plateHigh() - plan.row.lip / 2;
    // Across the roof lip, as far as the host face goes; walls' lips only when the plate's walls are over the host too.
    var hxRoof = role == Moving ? Math.min(hx, plan.shape.hostHalfWidth - edge) : hx;
    if (roof >= hostLow - 1e-9 && roof <= hostHigh + 1e-9) {
      var nx = Std.int(Math.max(3, Math.ceil(2 * hxRoof / plan.row.holePitch)));
      for (i in 0...nx + 1) out.push({x: -hxRoof + 2 * hxRoof * i / nx, y: roof});
    }
    // A plate cut back to its host has no lips on its walls.
    if (plateHalf() < plan.fullPlateHalf() - 1e-9) return out;
    // The walls' lips: from the roof down (or from the top of the host face) to the lowest host height, at most a pitch apart.
    var top = Math.min(roof - (out.length > 0 ? 10 : 0), hostHigh), bottom = Math.max(hostLow, WayCoverPlan.FLOOR_GAP + edge);
    if (top >= bottom - 1e-9) {
      var ny = Std.int(Math.max(1, Math.ceil((top - bottom) / plan.row.holePitch)));
      for (j in 0...ny + 1) {
        var y = top - (top - bottom) * j / ny;
        for (sx in [-1, 1]) out.push({x: sx * hx, y: y});
      }
    }
    return out;
  }

  override public function machinable():Bool return role != Sleeve;

  override public function hasGeometry():Bool return true;

  /** A box centred across the frame (x), `y0` to `y1` up and `z0` to `z1` along. */
  static function slab(halfWidth:Float, y0:Float, y1:Float, z0:Float, z1:Float):Part {
    var box = Part.box(2 * halfWidth, y1 - y0, z1 - z0), moved = box.translated(new cadkit.modeling.Vector(0, (y0 + y1) / 2, z0));
    box.close();
    return moved;
  }

  /** The solid between an outer and an inner section: both rectangles are given by their half width and height range. */
  static function ring(outerHalf:Float, oy0:Float, oy1:Float, innerHalf:Float, iy0:Float, iy1:Float, z0:Float, z1:Float):Part
    return Solids.cut(slab(outerHalf, oy0, oy1, z0, z1), [slab(innerHalf, iy0, iy1, z0 - 1, z1 + 1)]);

  function skin(z0:Float, z1:Float):Part {
    var row = plan.row, w = plan.innerHalfWidth(level);
    return ring(w + row.wall, plan.outerLow(level), plan.outerHigh(level), w, plan.innerLow(level), plan.innerHigh(level), z0, z1);
  }

  override public function geometry(detail:ComponentDetail = Preview):Part {
    var row = plan.row;
    if (role == Sleeve) return Solids.building([], owned -> {
      // The slide lip at the start end rides on the next smaller skin, which is how a loose skin is held and touches its parent.
      var w = plan.innerHalfWidth(level), g = row.gap;
      var shell = skin(0, plan.looseSkin());
      owned.push(shell);
      var open = plan.shape.open;
      var lip = plan.shape.reversed
        // Nesting that grows toward the fixed end: the parent is the larger skin, so the lip is on the outside of this one.
        ? ring(w + row.wall + g, open ? WayCoverPlan.FLOOR_GAP : plan.outerLow(level) - g, plan.outerHigh(level) + g, w + row.wall,
          open ? WayCoverPlan.FLOOR_GAP - 1 : plan.outerLow(level), plan.outerHigh(level), 0, WayCoverPlan.LIP)
        : ring(w, open ? WayCoverPlan.FLOOR_GAP : plan.innerLow(level), plan.innerHigh(level), w - g, open ? WayCoverPlan.FLOOR_GAP - 1 : plan.innerLow(level) + g,
          plan.innerHigh(level) - g, 0, WayCoverPlan.LIP);
      owned.push(lip);
      return Solids.union([shell, lip]);
    });
    return Solids.building([], owned -> {
      var plate = ring(plateHalf(), plan.plateLow(), plan.plateHigh(), plan.width / 2, plan.openingLow(), plan.openingHigh(), 0, row.flangeThickness);
      owned.push(plate);
      var shell = skin(row.flangeThickness, row.flangeThickness + plan.flangeSkin());
      owned.push(shell);
      var parts = [plate, shell];
      if (role == Moving) {
        // The wiper pad: a lip hugging the protected profile, from the opening out to the first skin, standing on the plate's inner face.
        var pad = ring(plan.innerHalfWidth(0), plan.shape.open ? WayCoverPlan.FLOOR_GAP : plan.innerLow(0), plan.innerHigh(0), plan.width / 2, plan.openingLow(), plan.openingHigh(),
          row.flangeThickness, row.flangeThickness + row.wiper);
        owned.push(pad);
        parts.push(pad);
      }
      var body = Solids.union(parts);
      owned.splice(0, owned.length);
      return machined(body);
    });
  }

  static var recipe:Null<ComponentType>;

  public static function recipeType():ComponentType {
    if (recipe == null) recipe = new ComponentType(
      "machinekit.cover.way-cover-section",
      [
        ComponentRecipeSupport.choice("row", WayCoverCatalog.ids(), WayCoverCatalog.ids()[0]),
        ComponentRecipeSupport.choice("role", ["Fixed", "Sleeve", "Moving"], "Fixed"),
        ComponentRecipeSupport.count("level", 0),
        ComponentRecipeSupport.length("width", 40),
        ComponentRecipeSupport.length("height", 40),
        ComponentRecipeSupport.length("stroke", 300),
        ComponentRecipeSupport.length("space", 120),
        ComponentRecipeSupport.choice("section", ["Ring", "Box"], "Ring"),
        ComponentRecipeSupport.length("hostMin", 0),
        ComponentRecipeSupport.length("hostMax", 100000),
        ComponentRecipeSupport.length("hostHalfWidth", 100000),
        ComponentRecipeSupport.choice("nesting", ["GrowsToMoving", "GrowsToFixed"], "GrowsToMoving")
      ],
      v -> {
        var shape:CoverShape = {open: v.token("section") == "Box", hostMin: v.number("hostMin"), hostMax: v.number("hostMax"),
          hostHalfWidth: v.number("hostHalfWidth"), reversed: v.token("nesting") == "GrowsToFixed"};
        var row = WayCoverCatalog.get(v.token("row")), plan = new WayCoverPlan(row, v.number("width"), v.number("height"), v.number("stroke"), v.number("space"), shape);
        var role = switch v.token("role") {
          case "Sleeve": Sleeve;
          case "Moving": Moving;
          default: Fixed;
        };
        // The defaults describe the fixed flange; the other roles name the skin they are.
        return new WayCoverSection(row, role, role == Fixed ? plan.fixedLevel() : role == Moving ? plan.movingLevel() : v.integer("level"), v.number("width"), v.number("height"),
          v.number("stroke"), v.number("space"), shape);
      },
      true
    ).numbered(v -> {
      var row = WayCoverCatalog.get(v.token("row"));
      var base = purchaseNumber(row, v.number("width"), v.number("height"), v.number("stroke"), v.number("space"), {open: v.token("section") == "Box", hostMin: 0, hostMax: 0, hostHalfWidth: 0, reversed: v.token("nesting") == "GrowsToFixed"});
      return v.token("role") == "Fixed" ? base : v.token("role") == "Moving" ? base + "-M" : base + "-S" + v.integer("level");
    }).procured(_ -> Buy(null, null, null)).requiring(v -> selectionRequirements(
      v.token("row"), v.number("width"), v.number("height"), v.number("stroke"), v.number("space"),
      v.token("section") == "Box", v.token("nesting") == "GrowsToFixed"
    ));
    return recipe;
  }

  override public function componentType():Null<ComponentType> return recipeType();

  override public function values():ComponentValues return new ComponentValues()
    .setToken("row", plan.row.id)
    .setToken("role", Std.string(role))
    .setInteger("level", level)
    .setNumber("width", plan.width)
    .setNumber("height", plan.height)
    .setNumber("stroke", plan.stroke)
    .setNumber("space", plan.space)
    .setToken("section", plan.shape.open ? "Box" : "Ring")
    .setNumber("hostMin", plan.shape.hostMin)
    .setNumber("hostMax", Math.min(plan.shape.hostMax, 100000))
    .setNumber("hostHalfWidth", Math.min(plan.shape.hostHalfWidth, 100000))
    .setToken("nesting", plan.shape.reversed ? "GrowsToFixed" : "GrowsToMoving")
    .setToken("material", materialSpec());
}
