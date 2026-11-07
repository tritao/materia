package machinekit.hobbing;

import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.component.*;
import machinekit.component.ComponentRecipeSupport as P;
import machinekit.component.BoltedFace.PinnedFace;
import materia.assembly.AssemblyFrames as F;

/** Separate unwelded X-carriage rib, sawn from bright drawn steel flat stock.
 * The scale carrier is a deliberate height interface derived from its head seat.
 * Hole geometry and purchase hardware come only from the declared joints. */
class HobberCarriageRib extends MachineComponent {
  public static final ROLES = ["leftOuter", "leftInner", "rightInner", "scale"];
  public final role:String;
  static final recipe = new machinekit.cache.Once<ComponentType>();
  public static function recipeType():ComponentType return recipe.get(() -> new ComponentType(
    "machinekit.hobbing.carriage-rib", [P.choice("role", ROLES, "leftInner")], v -> new HobberCarriageRib(v))
    .numbered(v -> "HOBBER-X-RIB-" + v.token("role").toUpperCase())
    .procured(v -> Make(routeFor(v.token("role")).process()))
    .requiring(v -> routeFor(v.token("role")).requirements()));
  public function new(?overrides:ComponentValues) {
    var v = P.resolve([P.choice("role", ROLES, "leftInner")], overrides);
    super("HOBBER-X-RIB-" + v.token("role").toUpperCase(), "Bolted bright drawn steel carriage rib", "steel", false, v);
    role = v.token("role");
    addConnector("datum", Mount, F.identity());
    // Bond the rib lands during assembly; stiffness and slip take no bond credit.
    addFacet(routeFor(role));
  }
  override public function componentType():Null<ComponentType> return recipeType();
  override public function hasGeometry():Bool return true;
  override public function machinable():Bool return true;
  public function band():{low:Float, high:Float, bottom:Float} return bandFor(role);
  static function bandFor(role:String):{low:Float, high:Float, bottom:Float} return switch role {
    case "leftOuter": {low: -180.0, high: -160.0, bottom: HobberSaddle.xCarriageBottom() - 40.0};
    case "leftInner": {low: -140.0, high: -120.0, bottom: HobberSaddle.xCarriageBottom() - 40.0};
    case "rightInner": {low: 120.0, high: 140.0, bottom: HobberSaddle.xCarriageBottom() - 40.0};
    case "scale":
      var box = HobberScales.carrierBox(HobberScales.x(), HobberScales.HEAD_CARRIER_MARGIN);
      {low: box[2], high: box[3], bottom: HobberScales.headFace(HobberScales.x())[2]};
    case _: throw "Unknown carriage rib";
  };
  static function routeFor(role:String):ManufacturingRoute {
    var b = bandFor(role);
    var width = HobberSaddle.xCarriageBottom() - b.bottom;
    var thickness = b.high - b.low;
    // The scale carrier is a deliberate height/width interface, milled from
    // the next five-mm nominal square. No stock tolerance is called vendor data.
    if (role == "scale") {
      var nominal = Math.ceil(Math.max(width, thickness) / 5) * 5;
      width = nominal;
      thickness = nominal;
    }
    var blank:machinekit.component.ManufacturingRoute.BarBlank = {width:width, thickness:thickness,
      length:246, source:"nominal bright drawn steel section; supplier quote and bond-face qualification pending", assumed:true};
    var faces:Array<machinekit.component.ManufacturingRoute.MachinedFace> = [{name:"bond face prepared and fixture-controlled; joint holes drilled, counterbored and match-reamed", tolerance:0.1, roughnessRa:null}];
    return role == "scale" ? ManufacturingRoute.milledFromBar(blank, faces) : ManufacturingRoute.sawnBar(blank, faces);
  }
  public function interfaceFrame(into:Float):materia.assembly.AssemblyRecord.AssemblyFrame {
    var b = band();
    return BoltedFace.frameOf({x: 263, y: (b.low + b.high) / 2, z: HobberSaddle.xCarriageBottom()},
      {x: 0, y: 0, z: into}, {x: 1, y: 0, z: 0});
  }
  public function boltSize():String return role == "scale" ? "M5" : "M6";
  public function boltXs():Array<Float> {
    var xs:Array<Float> = [155, 195, 235, 275, 315, 365];
    if (role == "scale") {
      // Keep the attachment counterbores clear of the head's upward tapped
      // bores, including their required edge-support annuli.
      xs[2] = 220;
      xs[4] = 325;
    }
    // Added central fixing for the installed Y module load. The retained
    // six-fixing FE is the baseline; the added bore needs final FE review.
    // No bond is credited.
    if(role!="scale")xs.push(255);
    return xs;
  }
  public function pinXs():Array<Float> return [175, 345];
  override public function fasteningFaces():Array<BoltedFace> {
    var b = band();
    var screw = machinekit.standard.SocketHeadCapScrew.catalog().get(boltSize());
    var recess = screw.headHeight + 0.5;
    var faces = [BoltedFace.clamped("plate", interfaceFrame(-1),
      [for (x in boltXs()) {x: x - 263, y: 0.0}], screw.clearanceMedium,
      HobberSaddle.xCarriageBottom() - b.bottom - recess, boltSize(), {diameter: screw.headDiameter + 2, depth: recess}, false)];
    if (role == "scale") faces.push(HobberScales.carrierFace(HobberScales.x(), [0.0, 0, 0], 8, HobberSaddle.xCarriageBottom() - b.bottom));
    return faces;
  }
  override public function pinnedFaces():Array<PinnedFace> return [new PinnedFace("plate-pins",
    interfaceFrame(-1), [for (x in pinXs()) {x: x - 263, y: 0.0}], 4, 8)];
  override public function geometry(detail:ComponentDetail = Preview):Part {
    var b = band();
    var raw = Part.box(246, b.high - b.low, HobberSaddle.xCarriageBottom() - b.bottom);
    var placed = raw.translated(new Vector(263, (b.low + b.high) / 2, b.bottom));
    raw.close();
    return machined(placed);
  }
}
