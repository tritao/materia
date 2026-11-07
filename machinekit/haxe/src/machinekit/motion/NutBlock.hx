package machinekit.motion;

import cadkit.modeling.Part;
import machinekit.component.ComponentDetail;
import machinekit.component.ComponentRecipeSupport;
import machinekit.component.ComponentType;
import machinekit.component.ComponentValues;
import machinekit.component.Dimension;
import machinekit.component.MachineComponent;
import machinekit.component.ManufacturingRoute;
import machinekit.component.Solids;
import machinekit.standard.SocketHeadCapScrew;
import machinekit.structural.GroundStock;
import machinekit.structural.GroundStock.GroundStockRow;
import materia.assembly.AssemblyFrames;
import materia.assembly.AssemblyRecord.AssemblyFrame;

/**
 * A flanged ball or lead-screw nut's carrier: a piece of ground steel flat bar, sawn to length, with the nut's barrel
 * bore and its flange bolts tapped in one broad face, and two tapped holes in the base end for the carriage.
 *
 * Frame: origin on the screw axis at the broad face the nut flange seats on, local +Z into the bar (the bar is `thickness`
 * thick), local +X across its `width`, local +Y toward the top; the base end is at y = -`centreHeight`. This is the
 * nut's `MountingFace` frame, so `nutPose()` is the nut's pose in the block's frame.
 *
 * The broad faces are ground stock and stay as ground; the base end and the bore are the machined faces (see the route).
 * The base holes sit near the width's edges, clear of the flange bolts; the constructor refuses a centre height that
 * leaves them no room.
 */
class NutBlock extends MachineComponent {
  /** Material kept beyond the flange rim at the top, and between a flange hole and the bar edge, mm. */
  public static inline var EDGE:Float = 6;
  /** Thread and depth of the carriage holes in the base end. */
  public static inline var BASE_THREAD:String = "M6";
  public static inline var BASE_DEPTH:Float = 12;
  /** Least thickness, mm: the pilot engagement, and the base holes' edge distance. */
  public static inline var MIN_THICKNESS:Float = 16;
  /** Web kept between a flange hole and a base hole, mm. */
  static inline var WEB:Float = 1;
  /**
	 * The dowels of a pinned block (`pinned`): two Ø6 H7 on the base end, 10 mm either side of the screw axis, `PIN_DEPTH` deep in the block (one diameter
	 * of engagement plus the 1 mm relief of a blind hole). They carry the thrust the screws' friction cannot (a crash), so the block is located against it.
	 */
  public static inline var PIN_DIAMETER:Float = 6;
  public static inline var PIN_OFFSET:Float = 10;
  public static inline var PIN_DEPTH:Float = 7;
  /** Distance from the block's side to the base holes' centres, mm, unless a machine asks for another (`baseEdge`). */
  public static inline var BASE_EDGE:Float = 10;

  public final nut:LeadScrewNut;
  public final centreHeight:Float;
  public final width:Float;
  public final thickness:Float;
  public final length:Float;
  /**
	 * Distance from the block's side to each base hole's centre: the holes stand `width - 2 * baseEdge` apart. A machine that must keep both
	 * clear of something beside the block (the Y nut block's bolts between the saddle and the swivel plate) draws them closer together.
	 */
  public final baseEdge:Float;
  /** Depth of the base holes' thread, mm (`BASE_DEPTH` unless a machine shortens it to clear the flange bolts). */
  public final baseDepth:Float;
  /** Whether the base end carries the two locating dowels (`PIN_*`). */
  public final pinned:Bool;
  public final stock:GroundStockRow;
  final seat:AssemblyFrame;

  public function new(drawing:String, nut:LeadScrewNut, centreHeight:Float, ?width:Float, ?thickness:Float, baseEdge:Float = BASE_EDGE, baseDepth:Float = BASE_DEPTH, pinned:Bool = false) {
    var face = nut.mountingFace();
    if (!nut.flanged || face == null) throw "A nut block carries a flanged nut";
    if (!Math.isFinite(centreHeight) || centreHeight <= 0) throw "A nut block needs a positive screw centre height";
    var wide = width ?? findWidth(nut.flangeDiameter + 2 * EDGE);
    var thick = thickness ?? GroundStock.atLeast("flat bar", MIN_THICKNESS);
    var base = SocketHeadCapScrew.catalog().get(BASE_THREAD);
    if (base == null) throw 'No ${BASE_THREAD} screw row';
    if (centreHeight < nut.flangeDiameter / 2 + 1) throw "Nut block: the screw centre height leaves the flange rim no clearance from the carriage";
    var long = centreHeight + nut.flangeDiameter / 2 + EDGE;
    if (baseEdge < 1.5 * base.diameter - 1e-9 || width != null && wide - 2 * baseEdge < 2.5 * base.diameter - 1e-9 || wide - 2 * baseEdge < 0)
      throw "Nut block: the base holes need 1.5 diameters to the side and 2.5 between them";
    var row = GroundStock.bar(wide, thick);
    if (wide < nut.flangeDiameter + 2 * EDGE - 1e-9) throw "Nut block is too narrow for the nut flange";
    if (thick < MIN_THICKNESS - 1e-9) throw "Nut block is too thin for the nut pilot and the base holes";
    super(
      'NUT-BLOCK-${Dimension.format(wide)}x${Dimension.format(thick)}x${Dimension.format(long)}-C${Dimension.format(centreHeight)}-F${Dimension.format(nut.flangeDiameter)}x${Dimension.format(nut.flangeThickness)}${baseEdge == BASE_EDGE ? "" : "-E" + Dimension.format(baseEdge)}${baseDepth == BASE_DEPTH ? "" : "-D" + Dimension.format(baseDepth)}${pinned ? "-P" : ""}',
      'Nut block for ${nut.description}, screw ${Dimension.format(centreHeight)} mm above the base',
      "steel",
      true
    );
    this.nut = nut;
    this.centreHeight = centreHeight;
    this.width = wide;
    this.thickness = thick;
    this.baseEdge = baseEdge;
    this.baseDepth = baseDepth;
    this.pinned = pinned;
    this.length = long;
    this.stock = row;
    seat = face.pose;
    declareDrawingNumber(drawing);
    var route = ManufacturingRoute.groundStock(row, [
      {name: pinned ? "base end; dowels match-drilled and reamed only after both-end screw indication, never pre-drilled" : "base end (ground after cutting)", tolerance: 0.02, roughnessRa: 1.6},
      {name: "nut bore", tolerance: 0.02, roughnessRa: 1.6}
    ]);
    addFacet(route);
    addFacet(route.intent());
    addFacet(new machinekit.component.EngineeringMetadata(new machinekit.component.ComponentValues()
      .setToken("ratingsBasis", "Stated").setToken("errorsBasis", "Assumed").setToken("source", "")));
    // The flange holes must leave room for the base holes.
    var tap = nut.mountingTapDrill() / 2;
    for (hole in nut.mountingHoles()) for (x in baseHoleXs()) {
      var rect = {x0: x - base.tapDrill / 2, x1: x + base.tapDrill / 2, y0: -centreHeight, y1: -centreHeight + baseDepth};
      var nearestX = Math.max(rect.x0, Math.min(hole.x, rect.x1)), nearestY = Math.max(rect.y0, Math.min(hole.y, rect.y1));
      var gap = Math.sqrt((hole.x - nearestX) * (hole.x - nearestX) + (hole.y - nearestY) * (hole.y - nearestY));
      if (gap < tap + WEB) throw "Nut block: the screw centre height leaves no room between the flange bolts and the base holes";
    }
    if (pinned) {
      // The dowels sit between the base screws and clear of the flange bolts and the barrel's bore.
      for (x in pinXs()) {
        for (sx in baseHoleXs()) if (Math.abs(sx - x) < PIN_DIAMETER / 2 + base.diameter / 2 + 2 * WEB) throw "Nut block: the base dowels need room between the base screws";
        var rect = {x0: x - PIN_DIAMETER / 2, x1: x + PIN_DIAMETER / 2, y0: -centreHeight, y1: -centreHeight + PIN_DEPTH};
        for (hole in nut.mountingHoles()) {
          var nearestX = Math.max(rect.x0, Math.min(hole.x, rect.x1)), nearestY = Math.max(rect.y0, Math.min(hole.y, rect.y1));
          if (Math.sqrt((hole.x - nearestX) * (hole.x - nearestX) + (hole.y - nearestY) * (hole.y - nearestY)) < tap + WEB)
            throw "Nut block: the screw centre height leaves no room between the flange bolts and the base dowels";
        }
        var nx = Math.max(rect.x0, Math.min(0.0, rect.x1)), ny = Math.max(rect.y0, Math.min(0.0, rect.y1));
        if (Math.sqrt(nx * nx + ny * ny) < nut.bodyDiameter / 2 + 0.2 + WEB) throw "Nut block: the screw centre height leaves no room between the barrel's bore and the base dowels";
      }
    }
    addConnector("nut", Mount, Solids.axial(0, 0, 0));
  }

  static function findWidth(least:Float):Float {
    for (candidate in GroundStock.barWidths()) if (candidate >= least - 1e-9) return candidate;
    throw 'No ground flat bar ${Dimension.format(least)} mm wide or more';
  }

  /** The base holes' centres across the width. */
  public function baseHoleXs():Array<Float> return [-(width / 2 - baseEdge), width / 2 - baseEdge];

  /** The base dowels' centres across the width. */
  public function pinXs():Array<Float> return [-PIN_OFFSET, PIN_OFFSET];

  override public function machinable():Bool return pinned;

  /** The pose of `nut` in this block's frame: the nut's flange seat against the broad face, its axis along the block's. */
  public function nutPose():AssemblyFrame return AssemblyFrames.inverse(seat);

  /** The holes of the finished block in its own frame: the barrel bore, the flange's tapped holes (through) and the two base holes. */
  public function holes():Array<ManufacturingRoute.RouteHole> {
    var flange = SocketHeadCapScrew.catalog().get(nut.mountScrew), base = SocketHeadCapScrew.catalog().get(BASE_THREAD);
    if (flange == null || base == null) throw "Nut block lost a screw row";
    var out:Array<ManufacturingRoute.RouteHole> = [{
      id: "N1", kind: "bore", thread: null, diameter: nut.bodyDiameter + 0.4, nominal: nut.bodyDiameter, fit: null, depth: thickness,
      position: [0, 0, 0], direction: [0, 0, 1]
    }];
    var n = 0;
    for (h in nut.mountingHoles()) out.push({
      id: "F" + (++n), kind: "tapped", thread: nut.mountScrew, diameter: flange.tapDrill, nominal: flange.diameter, fit: null,
      depth: thickness, position: [h.x, h.y, 0], direction: [0, 0, 1]
    });
    n = 0;
    for (x in baseHoleXs()) out.push({
      id: "T" + (++n), kind: "tapped", thread: BASE_THREAD, diameter: base.tapDrill, nominal: base.diameter, fit: null,
      depth: baseDepth, position: [x, -centreHeight, thickness / 2], direction: [0, 1, 0]
    });
    return out.concat(derivedRouteHoles());
  }

  static final recipeCache = new machinekit.cache.Once<ComponentType>();

  /** The recipe for a block on a ball nut (SFU1605 flange); a block on another nut is code-only. */
  public static function recipeType():ComponentType {
    return recipeCache.get(() -> new ComponentType(
      "machinekit.motion.nut-block",
      [
        ComponentRecipeSupport.text("drawing", "NUT-BLOCK"),
        ComponentRecipeSupport.flag("preloaded", true),
        ComponentRecipeSupport.length("centreHeight", 50),
        ComponentRecipeSupport.length("width", 60),
        ComponentRecipeSupport.length("thickness", 16),
        ComponentRecipeSupport.length("baseEdge", BASE_EDGE),
        ComponentRecipeSupport.length("baseDepth", BASE_DEPTH),
        ComponentRecipeSupport.flag("pinned", false)
      ],
      v -> new NutBlock(v.token("drawing"), new BallNut(v.boolean("preloaded")), v.number("centreHeight"), v.number("width"), v.number("thickness"), v.number("baseEdge"), v.number("baseDepth"), v.boolean("pinned")),
      true
    ).numbered(v -> v.token("drawing"))
      .procured(_ -> Make("ground steel flat bar, cut and drilled"))
      .requiringInputs(["centreHeight", "width", "thickness"]));
  }

  override public function componentType():Null<ComponentType> return Std.isOfType(nut, BallNut) ? recipeType() : null;

  override public function values():ComponentValues {
    if (!Std.isOfType(nut, BallNut)) return super.values();
    var values = new ComponentValues()
    .setToken("drawing", drawingNumber ?? "NUT-BLOCK")
    .setBoolean("preloaded", cast(nut, BallNut).preloaded)
    .setNumber("centreHeight", centreHeight)
    .setNumber("width", width)
    .setNumber("thickness", thickness)
    .setToken("material", materialSpec());
    if (baseEdge != BASE_EDGE) values.setNumber("baseEdge", baseEdge);
    if (baseDepth != BASE_DEPTH) values.setNumber("baseDepth", baseDepth);
    if (pinned) values.setBoolean("pinned", true);
    return values;
  }

  override public function revisionFingerprint():Null<String> {
    var number = machinekit.component.ComponentNumberKey.encode;
    return machinekit.component.BomRevision.fingerprint([drawingNumber, materialSpec(), number(width), number(thickness),
      number(length), number(centreHeight), number(nut.flangeDiameter), number(nut.flangeThickness), number(nut.bodyDiameter),
      number(nut.boltCircleDiameter), nut.mountScrew, Std.string(nut.boltCount)].concat(baseEdge == BASE_EDGE ? [] : [number(baseEdge)]).concat(baseDepth == BASE_DEPTH ? [] : [number(baseDepth)]).concat(pinned ? ["pinned"] : []).join("|"), 1);
  }

  /**
   * The faces the block is bolted by: the nut's flange seats on `nut` (its bolts thread the tap-drill holes `geometry` cuts, through
   * the bar), and the block's base end is the tapped `base` the carriage or seat is bolted to (`BASE_THREAD` holes `BASE_DEPTH`
   * deep, in the middle of the thickness).
   */
  override public function fasteningFaces():Array<machinekit.component.BoltedFace> {
    var base = SocketHeadCapScrew.catalog().get(BASE_THREAD);
    return [
      machinekit.component.BoltedFace.tapped(
        "nut",
        machinekit.component.BoltedFace.frameOf({x: 0, y: 0, z: 0}, {x: 0, y: 0, z: 1}, {x: 1, y: 0, z: 0}),
        [for (p in nut.mountingHoles()) {x: p.x, y: p.y}],
        nut.mountScrew,
        thickness,
        thickness,
        true
      ),
      machinekit.component.BoltedFace.tapped(
        "base",
        machinekit.component.BoltedFace.frameOf({x: 0, y: -centreHeight, z: thickness / 2}, {x: 0, y: 1, z: 0}, {x: 1, y: 0, z: 0}),
        [for (x in baseHoleXs()) {x: x, y: 0.0}],
        BASE_THREAD,
        baseDepth,
        length
      )
    ];
  }

  /** The base end's two locating dowels, where the block is pinned (the carriage declares the same holes). */
  override public function pinnedFaces():Array<machinekit.component.BoltedFace.PinnedFace> {
    if (!pinned) return [];
    return [new machinekit.component.BoltedFace.PinnedFace(
      "base-pins",
      machinekit.component.BoltedFace.frameOf({x: 0, y: -centreHeight, z: thickness / 2}, {x: 0, y: 1, z: 0}, {x: 1, y: 0, z: 0}),
      [for (x in pinXs()) {x: x, y: 0.0}],
      PIN_DIAMETER,
      PIN_DEPTH
    )];
  }

  override public function hasGeometry():Bool return true;

  override public function geometry(detail:ComponentDetail = Preview):Part {
    return Solids.building([], owned -> {
      var bar = Part.box(width, length, thickness);
      owned.push(bar);
      var placed = bar.translated(new cadkit.modeling.Vector(0, length / 2 - centreHeight, 0));
      owned.push(placed);
      var body = Solids.named(placed, "nut-block");
      if (detail == Envelope) return body;
      var cutout = nut.mountingCutout(thickness);
      owned.push(cutout);
      var tools = [cutout];
      var base = SocketHeadCapScrew.catalog().get(BASE_THREAD);
      if (base == null) throw 'No ${BASE_THREAD} screw row';
      var baseHole = 0;
      for (x in baseHoleXs()) {
        // Along +Y from the base end, in the middle of the thickness.
        tools.push(Solids.named(
          Part.cylinderAlongY(base.tapDrill / 2, -centreHeight - 1, -centreHeight + baseDepth, x, thickness / 2),
          "base-tap" + (++baseHole)
        ));
      }
      return machined(Solids.cut(body, tools));
    });
  }
}
