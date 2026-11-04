package machinekit.cover;

import cadkit.modeling.Part;
import machinekit.component.BoltedFace;
import machinekit.component.ComponentDetail;
import machinekit.component.ComponentRecipeSupport;
import machinekit.component.ComponentType;
import machinekit.component.ComponentValues;
import machinekit.component.Dimension;
import machinekit.component.MachineComponent;
import machinekit.component.Solids;
import machinekit.cover.WayCoverCatalog.WayCoverRow;

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

  public function new(row:WayCoverRow, role:WayCoverRole, level:Int, width:Float, height:Float, stroke:Float, space:Float) {
    var plan = new WayCoverPlan(row, width, height, stroke, space);
    if (level < 0 || level > plan.levels) throw 'Way cover ${row.id} has skins 0 to ${plan.levels}, not $level';
    if (role == Fixed && level != 0) throw "The fixed flange carries skin 0";
    if (role == Moving && level != plan.levels) throw 'The moving flange carries skin ${plan.levels}';
    if (role == Sleeve && (level < 1 || level >= plan.levels)) throw "A loose skin is neither the first nor the last";
    super(
      baseNumber(row, width, height, stroke, space) + (role == Fixed ? "" : role == Moving ? "-M" : '-S$level'),
      row.description + ", " + Dimension.format(width) + " x " + Dimension.format(height) + " opening, " + Dimension.format(stroke) + " mm stroke"
      + (role == Fixed ? "" : role == Moving ? " (moving flange and skin)" : ' (skin $level)'),
      row.material,
      true
    );
    this.plan = plan;
    this.role = role;
    this.level = level;
    addFacet(new machinekit.component.ProcurementIntent(Buy(null, null, null)));
    addFacet(new machinekit.component.EngineeringMetadata(new ComponentValues()
      .setToken("ratingsBasis", "Assumed").setToken("errorsBasis", "Assumed").setToken("source", row.basis)));
    if (role != Fixed) includedInPurchase(baseNumber(row, width, height, stroke, space));
  }

  public static function baseNumber(row:WayCoverRow, width:Float, height:Float, stroke:Float, space:Float):String
    return 'WAYCOVER-${row.id}-${Dimension.format(width)}x${Dimension.format(height)}-S${Dimension.format(stroke)}-L${Dimension.format(space)}';

  /** The face the flange bolts down by. */
  override public function fasteningFaces():Array<BoltedFace> {
    if (role == Sleeve) return [];
    var row = plan.row;
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

  /** The flange holes in its face frame: along the middle of the lip, at most `holePitch` apart. */
  public function holes():Array<{x:Float, y:Float}> {
    var hx = plan.outerWidth() / 2 - plan.row.lip / 2, hy = plan.outerHeight() / 2 - plan.row.lip / 2;
    var nx = Math.ceil(2 * hx / plan.row.holePitch), ny = Math.ceil(2 * hy / plan.row.holePitch);
    var out:Array<{x:Float, y:Float}> = [];
    for (i in 0...nx + 1) for (sy in [-1, 1]) out.push({x: -hx + 2 * hx * i / nx, y: sy * hy});
    for (j in 1...ny) for (sx in [-1, 1]) out.push({x: sx * hx, y: -hy + 2 * hy * j / ny});
    return out;
  }

  override public function machinable():Bool return role != Sleeve;

  override public function hasGeometry():Bool return true;

  static function slab(width:Float, height:Float, z0:Float, z1:Float):Part {
    var box = Part.box(width, height, z1 - z0), moved = box.translated(new cadkit.modeling.Vector(0, 0, z0));
    box.close();
    return moved;
  }

  static function ring(outerW:Float, outerH:Float, innerW:Float, innerH:Float, z0:Float, z1:Float):Part
    return Solids.cut(slab(outerW, outerH, z0, z1), [slab(innerW, innerH, z0 - 1, z1 + 1)]);

  function skin(z0:Float, z1:Float):Part {
    var row = plan.row, w = 2 * plan.innerHalfWidth(level), h = 2 * plan.innerHalfHeight(level);
    return ring(w + 2 * row.wall, h + 2 * row.wall, w, h, z0, z1);
  }

  override public function geometry(detail:ComponentDetail = Preview):Part {
    var row = plan.row;
    if (role == Sleeve) return Solids.building([], owned -> {
      // The slide lip at the start end rides on the next smaller skin, which is how a loose skin is held and touches its parent.
      var w = 2 * plan.innerHalfWidth(level), h = 2 * plan.innerHalfHeight(level), g = 2 * row.gap;
      var shell = skin(0, plan.looseSkin());
      owned.push(shell);
      var lip = ring(w, h, w - g, h - g, 0, WayCoverPlan.LIP);
      owned.push(lip);
      return Solids.union([shell, lip]);
    });
    return Solids.building([], owned -> {
      var plate = ring(plan.outerWidth(), plan.outerHeight(), plan.width, plan.height, 0, row.flangeThickness);
      owned.push(plate);
      var shell = skin(row.flangeThickness, row.flangeThickness + plan.flangeSkin());
      owned.push(shell);
      var parts = [plate, shell];
      if (role == Moving) {
        // The wiper pad: from the opening out to the skin, standing on the plate's inner face.
        var pad = ring(2 * plan.innerHalfWidth(level), 2 * plan.innerHalfHeight(level), plan.width, plan.height, row.flangeThickness,
          row.flangeThickness + row.wiper);
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
        ComponentRecipeSupport.length("space", 120)
      ],
      v -> {
        var row = WayCoverCatalog.get(v.token("row")), plan = new WayCoverPlan(row, v.number("width"), v.number("height"), v.number("stroke"), v.number("space"));
        var role = switch v.token("role") {
          case "Sleeve": Sleeve;
          case "Moving": Moving;
          default: Fixed;
        };
        // The defaults describe the fixed flange; the other roles name the skin they are.
        return new WayCoverSection(row, role, role == Fixed ? 0 : role == Moving ? plan.levels : v.integer("level"), v.number("width"), v.number("height"),
          v.number("stroke"), v.number("space"));
      },
      true
    ).numbered(v -> {
      var row = WayCoverCatalog.get(v.token("row"));
      var base = baseNumber(row, v.number("width"), v.number("height"), v.number("stroke"), v.number("space"));
      return v.token("role") == "Fixed" ? base : v.token("role") == "Moving" ? base + "-M" : base + "-S" + v.integer("level");
    }).procured(_ -> Buy(null, null, null)).requiringInputs(["width", "height", "stroke", "space"]);
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
    .setToken("material", materialSpec());
}
