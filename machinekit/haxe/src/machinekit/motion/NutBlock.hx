package machinekit.motion;

import cadkit.modeling.Part;
import machinekit.component.ComponentDetail;
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

  public final nut:LeadScrewNut;
  public final centreHeight:Float;
  public final width:Float;
  public final thickness:Float;
  public final length:Float;
  public final stock:GroundStockRow;
  final seat:AssemblyFrame;

  public function new(drawing:String, nut:LeadScrewNut, centreHeight:Float, ?width:Float, ?thickness:Float) {
    var face = nut.mountingFace();
    if (!nut.flanged || face == null) throw "A nut block carries a flanged nut";
    if (!Math.isFinite(centreHeight) || centreHeight <= 0) throw "A nut block needs a positive screw centre height";
    var wide = width ?? findWidth(nut.flangeDiameter + 2 * EDGE);
    var thick = thickness ?? GroundStock.atLeast("flat bar", MIN_THICKNESS);
    var base = SocketHeadCapScrew.catalog().get(BASE_THREAD);
    if (base == null) throw 'No ${BASE_THREAD} screw row';
    if (centreHeight < nut.flangeDiameter / 2 + 1) throw "Nut block: the screw centre height leaves the flange rim no clearance from the carriage";
    var long = centreHeight + nut.flangeDiameter / 2 + EDGE;
    var row = GroundStock.bar(wide, thick);
    if (wide < nut.flangeDiameter + 2 * EDGE - 1e-9) throw "Nut block is too narrow for the nut flange";
    if (thick < MIN_THICKNESS - 1e-9) throw "Nut block is too thin for the nut pilot and the base holes";
    super(
      'NUT-BLOCK-${Dimension.format(wide)}x${Dimension.format(thick)}x${Dimension.format(long)}-C${Dimension.format(centreHeight)}-F${Dimension.format(nut.flangeDiameter)}x${Dimension.format(nut.flangeThickness)}',
      'Nut block for ${nut.description}, screw ${Dimension.format(centreHeight)} mm above the base',
      "steel",
      true
    );
    this.nut = nut;
    this.centreHeight = centreHeight;
    this.width = wide;
    this.thickness = thick;
    this.length = long;
    this.stock = row;
    seat = face.pose;
    declareDrawingNumber(drawing);
    var route = ManufacturingRoute.groundStock(row, [
      {name: "base end", tolerance: 0.02, roughnessRa: 1.6},
      {name: "nut bore", tolerance: 0.02, roughnessRa: 1.6}
    ]);
    addFacet(route);
    addFacet(new machinekit.component.ProcurementIntent(Make(route.process())));
    addFacet(new machinekit.component.EngineeringMetadata(new machinekit.component.ComponentValues()
      .setToken("ratingsBasis", "Stated").setToken("errorsBasis", "Assumed").setToken("source", "")));
    // The flange holes must leave room for the base holes.
    var tap = nut.mountingTapDrill() / 2;
    for (hole in nut.mountingHoles()) for (x in baseHoleXs()) {
      var rect = {x0: x - base.tapDrill / 2, x1: x + base.tapDrill / 2, y0: -centreHeight, y1: -centreHeight + BASE_DEPTH};
      var nearestX = Math.max(rect.x0, Math.min(hole.x, rect.x1)), nearestY = Math.max(rect.y0, Math.min(hole.y, rect.y1));
      var gap = Math.sqrt((hole.x - nearestX) * (hole.x - nearestX) + (hole.y - nearestY) * (hole.y - nearestY));
      if (gap < tap + WEB) throw "Nut block: the screw centre height leaves no room between the flange bolts and the base holes";
    }
    addConnector("nut", Mount, Solids.axial(0, 0, 0));
  }

  static function findWidth(least:Float):Float {
    for (candidate in GroundStock.barWidths()) if (candidate >= least - 1e-9) return candidate;
    throw 'No ground flat bar ${Dimension.format(least)} mm wide or more';
  }

  /** The base holes' centres across the width. */
  public function baseHoleXs():Array<Float> return [-(width / 2 - 10), width / 2 - 10];

  /** The pose of `nut` in this block's frame: the nut's flange seat against the broad face, its axis along the block's. */
  public function nutPose():AssemblyFrame return AssemblyFrames.inverse(seat);

  override public function revisionFingerprint():Null<String> {
    var number = machinekit.component.ComponentNumberKey.encode;
    return machinekit.component.BomRevision.fingerprint([drawingNumber, materialSpec(), number(width), number(thickness),
      number(length), number(centreHeight), number(nut.flangeDiameter), number(nut.flangeThickness), number(nut.bodyDiameter),
      number(nut.boltCircleDiameter), nut.mountScrew, Std.string(nut.boltCount)].join("|"), 1);
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
      for (x in baseHoleXs()) {
        // Along +Y from the base end, in the middle of the thickness.
        tools.push(Part.cylinderAlongY(base.tapDrill / 2, -centreHeight - 1, -centreHeight + BASE_DEPTH, x, thickness / 2));
      }
      return Solids.cut(body, tools);
    });
  }
}
