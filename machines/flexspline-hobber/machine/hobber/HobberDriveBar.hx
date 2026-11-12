package hobber;

import machinekit.hobbing.*;

import cadkit.modeling.Part;
import machinekit.motion.ErobActuatorPart;
import cadkit.modeling.Vector;
import cadkit.modeling.Plane;
import machinekit.assembly.PosedParts;
import machinekit.component.*;
import machinekit.component.BoltedFace.PinnedFace;
import machinekit.component.ManufacturingRoute.MachinedFace;
import machinekit.structural.GroundStock;
import machinekit.component.ComponentRecipeSupport as P;
import materia.assembly.AssemblyFrames as F;
import materia.assembly.AssemblyRecord.AssemblyFrame;

/** GE-X-fixed-drive-bar; owns its stock, interfaces and manufacturing route. */
/**
 * A ground pad bar under a ball-screw support (X or Z, fixed or floating end): bolted to its cast-in seat, carrying the support
 * on its far face. Recorded in its own frame, the pad face against the casting (`frameFor`): +X along the screw, +Z into the
 * bar towards the support.
 */
class HobberDriveBar extends HobberMadePart {
  static final barRecipe = new machinekit.cache.Once<ComponentType>();

  /** Its own recipe: which axis and which end; every dimension follows from the drive and its support. */
  public static function barType():ComponentType return barRecipe.get(() -> new ComponentType("machinekit.hobbing.drive-bar",
    [P.choice("axis", ["X", "Z"], "X"), P.choice("support", ["fixed", "floating"], "fixed")],
    v -> new HobberDriveBar(v.token("axis"), v.token("support")), false, false, null, false).versioned(1)
    .numbered(v -> 'HOBBER-GE-${v.token("axis")}-${v.token("support").toUpperCase()}-DRIVE-BAR')
    .procured(_ -> Make("steel fabrication and machining")).requiring(v -> route(v.token("axis") == "X" ? HobberDrives.x() : HobberDrives.z()).requirements()));

  /** The bar's own frame in the part datum frame: its pad face against the casting. */
  public static function frameFor(axis:HobberDriveAxis, support:String):materia.assembly.AssemblyRecord.AssemblyFrame
    return HobberPadBolts.face(axis, support).frame;
  /** Width of the ground flat bar a drive pad is sawn from. */
  public static inline final PAD_WIDTH = HobberDrives.PAD_WIDTH;
  // ---- Fastening faces (BOM_PLAN F1): the joints whose screws, holes and clearances `deriveFasteners` derives ------------------

  /** The mating plane of the parts bolted on a pad or bracket: the faces of `axis`'s `support` seat, drilled and tapped to suit what stands on it. */
  public static function seatFace(axis:HobberDriveAxis, support:String, thickness:Float, ?clip:Array<Float>):BoltedFace {
    var b = clip ?? HobberDrives.padBounds(axis, support, PAD_WIDTH, thickness), n = axis.axis.normal, a = axis.axis.direction, ni = 0, ai = 0;
    for (i in 0...3) {
      if (Math.abs(n[i]) > 0.5) ni = i;
      if (Math.abs(a[i]) > 0.5) ai = i;
    }
    var ci = 3 - ni - ai, origin = [0.0, 0, 0], into = [0.0, 0, 0], across = [0.0, 0, 0];
    origin[ni] = n[ni] > 0 ? b[ni + 3] : b[ni];
    origin[ai] = (b[ai] + b[ai + 3]) / 2;
    origin[ci] = (b[ci] + b[ci + 3]) / 2;
    into[ni] = n[ni] > 0 ? -1 : 1;
    across[ai] = 1;
    return BoltedFace.drilledToSuit("seat-" + support, BoltedFace.frameOf({x: origin[0], y: origin[1], z: origin[2]}, {x: into[0], y: into[1], z: into[2]},
      {x: across[0], y: across[1], z: across[2]}), b[ai + 3] - b[ai], b[ci + 3] - b[ci], Math.min(thickness, 20.0), thickness, BoltedFace.ThreadMaterial.Steel,
      null, thickness <= 15);
  }
  /** The nut block's base end on the carriage, saddle or seat that carries it: two clearance holes for the block's tapped base, `grip` of material, in this part's frame (`origin` is the part frame's origin in the work frame). */
  public static function nutBlockFace(axis:HobberDriveAxis, grip:Float, origin:Array<Float>, ?recess:machinekit.component.BoltedFace.BoltRecess):BoltedFace {
    var block = HobberDrives.nutBlock(axis, new machinekit.motion.BallNut());
    var screw = machinekit.standard.SocketHeadCapScrew.catalog().get(machinekit.motion.NutBlock.BASE_THREAD);
    return BoltedFace.clamped("nut-block", nutBlockFrame(axis, origin), [for (u in block.baseHoleXs()) {x: u, y: 0.0}], screw.clearanceMedium, grip,
      machinekit.motion.NutBlock.BASE_THREAD, recess, false);
  }
  /** The face frame of the nut block's base end on its carrier (this part's frame; `origin` is the part frame's origin in the work frame). */
  public static function nutBlockFrame(axis:HobberDriveAxis, origin:Array<Float>):materia.assembly.AssemblyRecord.AssemblyFrame {
    var block = HobberDrives.nutBlock(axis, new machinekit.motion.BallNut()), pose = HobberDrives.nutBlockPose(axis, new machinekit.motion.BallNut());
    var at = F.transformPoint(pose, 0, -block.centreHeight, block.thickness / 2), out = F.transformVector(pose, 0, -1, 0), x = F.transformVector(pose, 1, 0, 0);
    return BoltedFace.frameOf({x: at.x - origin[0], y: at.y - origin[1], z: at.z - origin[2]}, {x: out.x, y: out.y, z: out.z}, {x: x.x, y: x.y, z: x.z});
  }
  /** The dowels of a pinned nut block in its carrier (X deck, Z saddle plate): the block's two, blind `NUT_PIN_DEPTH` deep. */
  public static function nutBlockPins(axis:HobberDriveAxis, origin:Array<Float>):PinnedFace {
    var block = HobberDrives.nutBlock(axis, new machinekit.motion.BallNut());
    return new PinnedFace("nut-block-pins", nutBlockFrame(axis, origin), [for (u in block.pinXs()) {x: u, y: 0.0}], machinekit.motion.NutBlock.PIN_DIAMETER, NUT_PIN_DEPTH);
  }
  /** Depth of the carrier's dowel bores for the nut block, mm: one diameter of engagement plus the relief, with a millimetre over (the carrier is 20 and 13 thick). */
  public static inline final NUT_PIN_DEPTH = 8.0;
  /** The dowels of the supports on a pad or bracket: each support's two, the seat's half of `SupportedScrewDrive.dowelPoints` (the foot's half is the support's own). */
  public static function padPins(axis:HobberDriveAxis, supports:Array<String>, thickness:Float, ?clip:Array<Float>):Array<machinekit.component.BoltedFace.PinnedFace> {
    var faces = [];
    for (support in supports) {
      var holes = [for (h in HobberDrives.seatRouteHoles(axis, support)) if (h.kind == "dowel") h];
      if (holes.length == 0) continue;
      faces.push(PinnedFace.atPoints("pins-" + support, seatFace(axis, support, thickness, clip).frame,
        [for (h in holes) {x: h.position[0], y: h.position[1], z: h.position[2]}], holes[0].nominal, holes[0].depth, holes[0].depth >= thickness - 1e-6));
    }
    return faces;
  }
  public static function barAxis(design:String):HobberDriveAxis return design.indexOf("GE-X-")==0?HobberDrives.x():HobberDrives.z();
  public static function barSupport(design:String):String return design.indexOf("-fixed-")>=0?"fixed":"floating";
  public function new(axis:String="X", support:String="fixed") {
    if((axis!="X"&&axis!="Z")||(support!="fixed"&&support!="floating"))throw "Unknown drive bar";
    super("GE-"+axis+"-"+support+"-drive-bar", new ComponentValues().setToken("axis", axis).setToken("support", support));
    finish(true);
  }
  override public function componentType():Null<ComponentType> return barType();

  override public function record(b:cadkit.parametric.recording.DocumentBuilder, detail:ComponentDetail):Null<cadkit.parametric.Feature> {
    var axis = barAxis(designId), support = barSupport(designId), bounds = HobberDrives.barBounds(axis, support);
    var ni = 0, ai = 0;
    for (i in 0...3) {
      if (Math.abs(axis.axis.normal[i]) > 0.5) ni = i;
      if (Math.abs(axis.axis.direction[i]) > 0.5) ai = i;
    }
    var ci = 3 - ni - ai;
    var blank:Null<cadkit.parametric.Feature> = null;
    b.group("Stock", () -> {
      var length = b.length("bar.length", bounds[ai + 3] - bounds[ai])
        .derivedBy("tapped seat holes along the screw + 3 diameters, up to 5 mm" + (support == "floating" ? "; at least the support's foot" : ""));
      var width = b.length("bar.width", bounds[ci + 3] - bounds[ci]).derivedBy("tapped seat holes across + 3 diameters, up to 5 mm");
      var thickness = b.length("bar.thickness", bounds[ni + 3] - bounds[ni]).fixedBy("ground flat bar, matched pair for the axis");
      blank = b.label(b.block(length, width, thickness), "Ground bar");
    });
    var body = blank;
    if (axis.id == "Z" && support == "floating") {
      var tools = runningClearance(axis, support, b);
      body = b.label(b.cutAll(blank, tools, [for (i in 0...tools.length) 'swivel head sweep ${i + 1}']), "Z running clearance");
    }
    // Its two interfaces: the pad face on the casting (the frame itself) and the seat the support stands on (the far face).
    b.datum("pad", new Plane(new Vector(0, 0, 0), Vector.X(), Vector.Z()));
    // The seat's in-plane axis follows the drive's convention (the support's dowels are laid out in it), so it is the drive's
    // seat frame expressed in this bar's frame, not one written here.
    var seat = F.compose(F.inverse(frameFor(axis, support)), seatFace(axis, support, bounds[ni + 3] - bounds[ni], bounds).frame);
    var o = F.transformPoint(seat, 0, 0, 0), x = F.transformVector(seat, 1, 0, 0), z = F.transformVector(seat, 0, 0, 1);
    b.datum("seat", new Plane(new Vector(o.x, o.y, o.z), new Vector(x.x, x.y, x.z), new Vector(z.x, z.y, z.z)));
    return body;
  }

  /** The swivel head's swept envelope over Z's stroke and overtravel, with its running gap, as tools in this bar's frame. */
  function runningClearance(axis:HobberDriveAxis, support:String, b:cadkit.parametric.recording.DocumentBuilder):Array<cadkit.parametric.Feature> {
    var skeleton = HobberSkeleton.reference(), range = HobberScales.stroke("Z"), over = HobberScales.overtravel();
    var toOwn = F.inverse(frameFor(axis, support));
    var zLow = skeleton.nativeHobCentreZ + range.lower - over, zHigh = skeleton.nativeHobCentreZ + range.upper + over;
    var tools:Array<cadkit.parametric.Feature> = [];
    function own(x:Float, y:Float, z:Float):Vector {
      var p = F.transformPoint(toOwn, x, y, z);
      return new Vector(p.x, p.y, p.z);
    }
    var along = F.transformVector(toOwn, 1, 0, 0);
    for (s in skeleton.envelopeSegments(skeleton.zRunningGap)) {
      for (z in [zLow, zHigh]) tools.push(b.cylinderAlong(own(0, 0, z), new Vector(along.x, along.y, along.z), s.r, s.x0, s.x1));
      var a = own(s.x0, -s.r, zLow), c = own(s.x1, s.r, zHigh);
      tools.push(b.boxBetween(Math.min(a.x, c.x), Math.max(a.x, c.x), Math.min(a.y, c.y), Math.max(a.y, c.y), Math.min(a.z, c.z), Math.max(a.z, c.z)));
    }
    return tools;
  }
  override public function machinable():Bool return true;
  /** The supports' dowels, at the seat datum. */
  override public function pinnedFaces():Array<PinnedFace> return {
    var axis=barAxis(designId),support=barSupport(designId);
    [for (pins in padPins(axis,[support],HobberDrives.barThickness(axis),HobberDrives.barBounds(axis,support))) pins.at(recordedDatum("seat"))];
  };
  override public function holes():Array<machinekit.component.ManufacturingRoute.RouteHole> return {
    derivedRouteHoles();
  };
  override public function manufacturingRoute():Null<ManufacturingRoute> return route(barAxis(designId));

  static function route(axis:HobberDriveAxis):ManufacturingRoute return {
    ManufacturingRoute.groundStock(GroundStock.bar(80,HobberDrives.barThickness(axis)),[
      {name:"saw finished width from80 mm ground stock; preserve both broad faces; buy both supports from one matched-thickness parent bar, inspect pair within0.01 mm; no epoxy",tolerance:0.01,roughnessRa:0.8},
      {name:"joint-derived bores, flush bar-clamp counterbores and support match-pins after indication",tolerance:0.1,roughnessRa:null}
    ]);
  };
  override public function fasteningFaces():Array<BoltedFace> {
    var faces:Array<BoltedFace>=[];
    var axis=barAxis(designId),support=barSupport(designId);
    // The faces' holes and roles come from the drive; the recording's datums place them.
    faces.push(seatFace(axis,support,HobberDrives.barThickness(axis),HobberDrives.barBounds(axis,support)).at(recordedDatum("seat")));
    faces.push(HobberPadBolts.face(axis,support).at(recordedDatum("pad")));
    return faces;
  }
}
