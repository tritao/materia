import CadKit;
import machinekit.assembly.AxisBuilder;
import machinekit.assembly.MachineAssembly;
import machinekit.assembly.JointFasteners;
import machinekit.assembly.JointSlip;
import machinekit.assembly.JointSlip.JointLoad;
import machinekit.assembly.JointSlip.SlipPin;
import machinekit.assembly.JointSlip.SlipPoint;
import machinekit.assembly.JointSlip.SlipScrew;
import machinekit.assembly.JointFasteners.FastenerDerivation;
import machinekit.component.BoltedFace;
import machinekit.component.BoltedFace.BoltHole;
import machinekit.component.BoltedFace.ThreadMaterial;
import machinekit.component.BoltedFace.PinnedFace;
import machinekit.component.MachineComponent;
import machinekit.motion.LinearRail;
import cadkit.modeling.Part;
import machinekit.component.ComponentDetail;
import machinekit.component.Solids;
import materia.assembly.AssemblyFrames;

typedef Vec = {x:Float, y:Float, z:Float}

/** A part that only states its bolted faces, for exercising `JointFasteners`. */
class FaceBlock extends MachineComponent {
  final faces:Array<BoltedFace>;
  final thickness:Float;
  final low:Float;

  /** `thickness` > 0 gives the block a 100 x 100 body from z = `low` up, which cuts its derived holes. */
  public function new(name:String, faces:Array<BoltedFace>, thickness:Float = 0, low:Float = 0) {
    super("TEST-FACE-BLOCK-" + name, "Face block " + name, "steel", true);
    this.faces = faces;
    this.thickness = thickness;
    this.low = low;
  }

  override public function fasteningFaces():Array<BoltedFace> return faces;
  override public function machinable():Bool return true;
  override public function hasGeometry():Bool return thickness > 0;
  override public function geometry(detail:ComponentDetail = Preview):Part {
    if (!(thickness > 0)) throw new machinekit.component.NoGeometry(designation);
    return machined(Solids.named(Part.box(100, 100, thickness).translated(new cadkit.modeling.Vector(0, 0, low)), "body"));
  }
}

/** A physical masked shim; the declared thickness can be wrong for a negative control. */
class MaskedJointLayer extends FaceBlock {
  public function new(holes:Array<BoltHole>,diameter:Float,masked:Bool=true,thickness:Float=3,declared:Float=3) {
    super("LAYER",[],thickness);
    addFacet(new machinekit.component.InterfaceLayer(declared));
    if(masked)machine([for(i=>h in holes){frame:AssemblyFrames.identity(),x:h.x,y:h.y,diameter:diameter,from:-1.0,to:thickness+1,kind:"clearance",thread:null,name:"mask"+i}]);
  }
}

class NarrowJointLayer extends MaskedJointLayer {
  public function new(holes:Array<BoltHole>)super(holes,5.5);
  override public function geometry(detail:ComponentDetail=Preview):Part return machined(Part.box(50,100,3));
}

/** A part that only states its dowel faces, with a body (z from `low`, `thickness` thick) that cuts their reamed holes. */
class PinBlock extends MachineComponent {
  final faces:Array<machinekit.component.BoltedFace.PinnedFace>;
  final thickness:Float;
  final low:Float;

  public function new(name:String, faces:Array<machinekit.component.BoltedFace.PinnedFace>, thickness:Float, low:Float) {
    super("TEST-PIN-BLOCK-" + name, "Pin block " + name, "steel", true);
    this.faces = faces;
    this.thickness = thickness;
    this.low = low;
  }

  override public function pinnedFaces():Array<machinekit.component.BoltedFace.PinnedFace> return faces;
  override public function machinable():Bool return true;
  override public function hasGeometry():Bool return true;
  override public function geometry(detail:ComponentDetail = Preview):Part
    return machined(Solids.named(Part.box(100, 100, thickness).translated(new cadkit.modeling.Vector(0, 0, low)), "body"));
}

/** A part that states faces but does not cut holes. */
class PlainBlock extends MachineComponent {
  final faces:Array<BoltedFace>;

  public function new(faces:Array<BoltedFace>) {
    super("TEST-PLAIN-BLOCK", "Plain block", "steel", true);
    this.faces = faces;
  }

  override public function fasteningFaces():Array<BoltedFace> return faces;
}

/**
 * `JointFasteners` on small assemblies with one defect each, and on their sound versions, then on the bench mill: every
 * rail and motor screw of its BOM is derived from the faces its parts declare.
 */
class JointFastenerTests {
  static function check(condition:Bool, message:String):Void {
    if (!condition) throw 'Joint fasteners: $message';
  }

  /** The cover face looks down (-Z into the cover would be up): face frames are built per case below. */
  static function square(pitch:Float):Array<BoltHole> return [for (x in [-1, 1]) for (y in [-1, 1]) {x: x * pitch / 2, y: y * pitch / 2}];

  static var down:Vec = {x: 0.0, y: 0.0, z: -1.0};
  static var up:Vec = {x: 0.0, y: 0.0, z: 1.0};
  static var across:Vec = {x: 1.0, y: 0.0, z: 0.0};
  static var origin:Vec = {x: 0.0, y: 0.0, z: 0.0};

  /** A host top face at z = `top` looking down into the host. */
  static function hostFace(face:Float -> BoltedFace, top:Float = 10):FaceBlock return new FaceBlock("HOST", [face(top)]);

  static function frameAtTop(top:Float):materia.assembly.AssemblyRecord.AssemblyFrame return BoltedFace.frameOf({x: 0, y: 0, z: top}, down, across);

  /** A cover (bottom face at its origin, looking up into the cover) on a host with the given receiving face. */
  static function stack(receiving:BoltedFace, clamped:BoltedFace, lift:Float = 0, fixed:Bool = true):AxisBuilder {
    var builder = new AxisBuilder();
    builder.place("host", new FaceBlock("HOST", [receiving]), AssemblyFrames.translation(0, 0, 0));
    var cover = new FaceBlock("COVER", [clamped]);
    if (fixed) builder.attach("cover", cover, AssemblyFrames.translation(0, 0, 10 + lift), "host");
    else builder.place("cover", cover, AssemblyFrames.translation(0, 0, 10 + lift));
    return builder;
  }

  static function cover(holes:Array<BoltHole>, ?size:String, grip:Float = 6, clearance:Float = 5.5, ?recess:machinekit.component.BoltedFace.BoltRecess, drawn:Bool = true, washer:Bool = false):BoltedFace
    return BoltedFace.clamped("bottom", BoltedFace.frameOf(origin, up, across), holes, clearance, grip, size, recess, drawn, washer);

  static function tappedHost(holes:Array<BoltHole>, size:String = "M5", depth:Float = 8, thickness:Float = 8, through:Bool = true):BoltedFace
    return BoltedFace.tapped("top", frameAtTop(10), holes, size, depth, thickness, through);

  static function suitHost(width:Float = 100, height:Float = 100, thickness:Float = 21, ?material:ThreadMaterial):BoltedFace
    return BoltedFace.drilledToSuit("top", frameAtTop(10), width, height, Math.min(20, thickness), thickness, material ?? CastIron);

  static function expect(builder:AxisBuilder, code:String, label:String):Void {
    var result = JointFasteners.derive(builder);
    check(result.codes().indexOf(code) >= 0, '$label should report $code, found ${result.codes()}: ${[for (i in result.diagnostics.items) i.message].join("; ")}');
  }

  public static function main():Void run();

  static function layeredFixture(masked:Bool=true,shift:Float=0,solid:Float=3,fixed:Bool=true,recess:Bool=false,air:Float=0):AxisBuilder {
    var builder=new AxisBuilder(),pattern=square(40);
    builder.place("host",new FaceBlock("HOST",[BoltedFace.drilledToSuit("top",frameAtTop(10),100,100,8,20,Steel,"M5")],20,-10),AssemblyFrames.identity());
    var layer=new MaskedJointLayer(pattern,5.5,masked,solid);
    if(fixed)builder.attach("layer",layer,AssemblyFrames.translation(shift,0,10),"host");
    else builder.place("layer",layer,AssemblyFrames.translation(shift,0,10));
    builder.attach("cover",new FaceBlock("COVER",[cover(pattern,"M5",6,5.5,recess?{diameter:10.0,depth:6.0}:null,false)],recess?12:6),AssemblyFrames.translation(0,0,13+air),"host");
    return builder;
  }

  static function replicatedLayerJoints():Void {
    for(recess in [false,true]) {
      var builder=layeredFixture(true,0,3,true,recess),d=builder.deriveFasteners();d.require();
      check(d.joints.length==1&&d.count()==4,"one steel joint crosses its physical replication layer");
      var joint=d.joints[0];
      check(Math.abs(joint.grip-9)<1e-9&&joint.length==16,"layer adds three mm to screw grip and standard length");
      check(joint.engagement==7&&joint.minEngagement==5,"compound receives no thread-engagement credit");
      var report=machinekit.assembly.AssemblyIntegrity.fasteners(builder);
      check(report.errors().length==0,"layer masks, actual head/counterbore and steel thread geometry fit: "+[for(e in report.errors())e.message].join("; "));
    }
    var combined=new AxisBuilder(),pattern=square(40);
    combined.place("host",new FaceBlock("HOST",[BoltedFace.drilledToSuit("top",frameAtTop(10),100,100,12,20,Steel,"M5")],20,-10),AssemblyFrames.identity());
    combined.attach("spacer",new MaskedJointLayer(pattern,5.5,true,2,2),AssemblyFrames.translation(0,0,10),"host");
    combined.attach("resin",new MaskedJointLayer(pattern,5.5),AssemblyFrames.translation(0,0,12),"spacer");
    combined.attach("cover",new FaceBlock("COVER",[cover(pattern,"M5",6,5.5,null,false)],6),AssemblyFrames.translation(0,0,15),"spacer");
    var combinedResult=combined.deriveFasteners();combinedResult.require();
    check(combinedResult.joints[0].grip==11,"spacer and compound both contribute to one through-clamped stack");
    check(combinedResult.joints[0].engagement==5,"only the original steel host supplies the final thread");
    check(machinekit.assembly.AssemblyIntegrity.check(combined,{sweepStates:[],strict:true}).errors().length==0,"stack bolts and adjacent fixed contacts agree with the physical layers");
    var overlapping=layeredFixture();
    overlapping.attach("overlap",new MaskedJointLayer(pattern,5.5,true,1,1),AssemblyFrames.translation(0,0,11),"host");
    expect(overlapping,"fasteners.unmatched","an extra layer inside a complete stack cannot be ignored");
    expect(layeredFixture(false),"fasteners.unmatched","unmasked compound cannot pass a bolt");
    expect(layeredFixture(true,95),"fasteners.unmatched","a touching layer must support every bolt axis");
    expect(layeredFixture(true,0,4),"fasteners.unmatched","a layer overlapping the foot cannot bridge it");
    expect(layeredFixture(true,0,3,false),"fasteners.unmatched","a loose layer is not a structural bridge");
    expect(layeredFixture(true,0,3,true,false,0.5),"fasteners.unmatched","declared layers cannot hide an air gap");
    var narrow=new AxisBuilder(),narrowPattern=square(40);
    narrow.place("host",new FaceBlock("HOST",[BoltedFace.drilledToSuit("top",frameAtTop(10),100,100,8,20,Steel,"M5")],20,-10),AssemblyFrames.identity());
    narrow.attach("layer",new NarrowJointLayer(narrowPattern),AssemblyFrames.translation(0,0,10),"host");
    narrow.attach("cover",new FaceBlock("COVER",[cover(narrowPattern,"M5",6,5.5,null,false)],6),AssemblyFrames.translation(0,0,13),"host");
    var narrowResult=JointFasteners.derive(narrow);narrowResult.require();
    check([for(d in narrowResult.diagnostics.items)if(d.code=="fasteners.edge-support"&&d.message.indexOf("layer")>=0)d].length>0,"made intermediate spacers obey the same per-hole edge rule");
    var original=layeredFixture(),rotated=new AxisBuilder();
    rotated.include("stack",original,AssemblyFrames.fromRotationMatrix(0,0,0,[1.0,0,0,0,0,-1,0,1,0]));
    var rd=JointFasteners.derive(rotated);rd.require();check(rd.joints[0].grip==9,"physical layer matching is independent of machine orientation");
    var pins=new AxisBuilder(),pinPattern=[{x:-15.0,y:0.0},{x:15.0,y:0.0}];
    pins.place("host",new PinBlock("HOST",[new PinnedFace("top",frameAtTop(10),pinPattern,5,10,false,false)],20,-10),AssemblyFrames.identity());
    pins.attach("spacer",new MaskedJointLayer(pinPattern,5,true,35,35),AssemblyFrames.translation(0,0,10),"host");
    pins.attach("layer",new MaskedJointLayer(pinPattern,5,true,5,5),AssemblyFrames.translation(0,0,45),"spacer");
    pins.attach("cover",new PinBlock("COVER",[new PinnedFace("bottom",BoltedFace.frameOf(origin,up,across),pinPattern,5,8,true,false)],8,0),AssemblyFrames.translation(0,0,50),"spacer");
    var pd=pins.deriveFasteners();pd.require();check(pd.dowels.length==1,"match pins cross the physical layer");
    var pin=pd.dowels[0];
    check(Math.abs(pin.length-pin.engagementA-pin.engagementB-40)<1e-9,"pin length includes compound but steel engagement does not");
    check(pin.engagementA>=5&&pin.engagementB>=5,"both steel parts retain a full-diameter pin engagement");
    check(machinekit.assembly.AssemblyIntegrity.fasteners(pins).errors().length==0,"joint-derived dowel bores cross the masked layer");
    Sys.println("interface layers: physical contact, masks, screw grip, counterbores, steel engagement and match pins pass");
  }

  static function repeatedMachiningBom():Void {
    function mount(name:String,x:Float,dx:Float=0,thread:String="M3"):machinekit.cover.WayCoverMount {
      var part=new machinekit.cover.WayCoverMount("REPEATED-MOUNT-TEST",50,50,14);
      part.machine([{name:name,frame:AssemblyFrames.fromRotationMatrix(dx,25,0,[1.0,0,0,0,-1,0,0,0,-1]),
        x:x,y:7.0+dx*1e-15,diameter:2.5,from:-1.0,to:8.0,kind:"tapped",thread:thread}]);
      return part;
    }
    var a=mount("first-instance",12),b=mount("second-instance",2,10),bom=new machinekit.component.Bom();
    var first=a.geometry(),second=b.geometry(),stock=new machinekit.cover.WayCoverMount("REPEATED-MOUNT-TEST",50,50,14).geometry(),common=first.intersect(second);
    check(Math.abs(stock.volume()-first.volume()-Math.PI*1.25*1.25*8)<0.01,"identity fixture drills a real blind bore in steel");
    check(Math.abs(first.volume()-common.volume())<1e-6 && Math.abs(second.volume()-common.volume())<1e-6,"different joint frames produce the same finished solid");
    common.close();stock.close();first.close();second.close();
    check(a.bom.fingerprint==b.bom.fingerprint,"same physical machining survives joint naming, frame choice and rigid-transform roundoff");
    check(a.geometryKey()!=b.geometryKey(),"geometry cache preserves instance face labels");
    bom.addComponent(a);bom.addComponent(b);
    check(bom.lines().length==1 && bom.lines()[0].quantity==2,"repeated machined brackets aggregate in the BOM");
    check(a.bom.fingerprint!=mount("first-instance",13).bom.fingerprint,"moved hole changes manufacturing revision");
    check(a.bom.fingerprint!=mount("first-instance",12,0,"M4").bom.fingerprint,"thread callout changes manufacturing revision");
  }
  public static function run():Void {
    replicatedLayerJoints();
    repeatedMachiningBom();
    sound();
    selection();
    machining();
    washersAndNuts();
    suitSeats();
    fit();
    defects();
    dowels();
    slip();
    mill();
    Sys.println("joint fasteners: derivation, validation and the bench mill's rail and motor screws pass");
  }

  /** A host (z -10..10) and a cover (z 10..18) located by dowels; each pin face lists its own holes. */
  static function pinned(hostHoles:Array<BoltHole>, coverHoles:Array<BoltHole>, hostDiameter:Float = 5, coverDiameter:Float = 5, hostDepth:Float = 10,
      coverDepth:Float = 8, fixed:Bool = true, drawn:Bool = false):AxisBuilder {
    var builder = new AxisBuilder();
    builder.place("host", new PinBlock("HOST", [new PinnedFace("top", frameAtTop(10), hostHoles, hostDiameter, hostDepth, false, drawn)], 20, -10),
      AssemblyFrames.translation(0, 0, 0));
    var cover = new PinBlock("COVER", [new PinnedFace("bottom", BoltedFace.frameOf(origin, up, across), coverHoles, coverDiameter, coverDepth, true, drawn)], 8, 0);
    if (fixed) builder.attach("cover", cover, AssemblyFrames.translation(0, 0, 10), "host");
    else builder.place("cover", cover, AssemblyFrames.translation(0, 0, 10));
    return builder;
  }

  static function selection():Void {
    var builder = stack(tappedHost(square(50)), cover(square(50), "M5"));
    builder.place("uninstalled", new FaceBlock("UNINSTALLED", [cover(square(50), "M5")]), AssemblyFrames.translation(300, 0, 0));
    var all = JointFasteners.derive(builder);
    check(all.codes().indexOf("fasteners.unmatched") >= 0, "default derivation still rejects an unmatched clamped member");
    var selected = JointFasteners.derive(builder, {clampedMembers: ["cover"]});
    selected.require();
    check(selected.joints.length == 1 && selected.count() == 4, "selected clamped members still find unselected hosts");
    check(JointFasteners.derive(builder, {clampedMembers: ["missing"]}).codes().indexOf("fasteners.selection") >= 0,
      "unknown selected clamped members are errors");
    check(JointFasteners.derive(builder, {clampedMembers: ["uninstalled"]}).codes().indexOf("fasteners.unmatched") >= 0,
      "selected unmatched faces remain errors");
    var holes:Array<BoltHole> = [{x: -20.0, y: 0.0}, {x: 20.0, y: 0.0}];
    var pins = pinned(holes, holes);
    pins.place("uninstalled", new PinBlock("UNINSTALLED", [new PinnedFace("pin", frameAtTop(10), holes, 5, 10)], 20, -10),
      AssemblyFrames.translation(300, 0, 0));
    var scopedPins = JointFasteners.derive(pins, {clampedMembers: ["cover"]});
    scopedPins.require();
    check(scopedPins.dowelCount() == 2, "scoped pins pair with unselected hosts");
    check(JointFasteners.derive(pins).codes().indexOf("fasteners.pin-unmatched") >= 0,
      "default derivation still checks every pin face");
    check(JointFasteners.derive(pins, {clampedMembers: ["uninstalled"]}).codes().indexOf("fasteners.pin-unmatched") >= 0,
      "selected unmatched pins remain errors");
  }

  static function dowels():Void {
    var two:Array<BoltHole> = [{x: -20.0, y: 0.0}, {x: 20.0, y: 0.0}];
    var result = JointFasteners.derive(pinned(two, two));
    result.require();
    check(result.dowels.length == 1 && result.dowelCount() == 2 && result.joints.length == 0, "two matching pinned faces are one dowelled joint of two pins");
    var joint = result.dowels[0];
    check(joint.size == "5" && joint.length == 10 && joint.item.partNumber == "ISO8734-5x10", 'a 5 mm dowel takes the shortest standard length that engages one diameter each side: ${joint.item.partNumber}');
    check(Math.abs(joint.engagementA + joint.engagementB - joint.length) < 1e-9 && joint.engagementA >= 5 - 1e-9 && joint.engagementB >= 5 - 1e-9, "the pin stands in both parts by a diameter or more");
    var lines = result.lines();
    check(lines.length == 1 && lines[0].quantity == 2, "the BOM line is the pin, twice");
    var cuts = [for (member in ["host", "cover"]) result.machining.get(member)];
    check(cuts[0] != null && cuts[0].length == 2 && cuts[0][0].kind == "dowel" && cuts[0][0].fit == "H7" && cuts[0][0].through != true, "the host is reamed blind: " + cuts[0]);
    check(cuts[1] != null && cuts[1].length == 2 && cuts[1][0].through == true, "the cover is reamed through");
    // Holes the parts draw are not cut again.
    var drawn = JointFasteners.derive(pinned(two, two, 5, 5, 10, 8, true, true));
    drawn.require();
    check(drawn.dowels.length == 1 && drawn.machining.get("host") == null, "a part that draws its dowel holes gets no derived cuts");
    // A deeper pair takes the longer pin only as far as the holes allow; 8 mm needs 16.
    var big = JointFasteners.derive(pinned(two, two, 8, 8, 12, 12));
    big.require();
    check(big.dowels[0].length == 16 && big.dowels[0].item.partNumber == "ISO8734-8x16", "an 8 mm dowel in 12 mm holes is 8 x 16: " + big.dowels[0].item.partNumber);
    // The derivation gives the machine its pins in the BOM and the cuts in the parts' geometry.
    var builder = pinned(two, two);
    builder.deriveFasteners();
    var kept = builder.derivedFasteners();
    check(kept != null && kept.dowels.length == 1, "the assembly keeps its derived dowels");
    var host = builder.components()[0].component;
    check(host.machinedCuts().length == 2, "the host machines its reamed holes");
    // Defects.
    expect(pinned(two, two, 5, 6), "fasteners.pin-diameter", "dowel holes of different diameters");
    expect(pinned(two, [{x: -20.0, y: 0.0}, {x: 20.0, y: 3.0}]), "fasteners.pin-pattern", "dowel holes that differ in one place");
    expect(pinned(two, [{x: -20.0, y: 0.0}]), "fasteners.pin-pattern", "dowel faces with different hole counts");
    expect(pinned(two, [{x: -30.0, y: 0.0}, {x: 30.0, y: 0.0}]), "fasteners.pin-unmatched", "dowel faces with no common hole");
    expect(pinned(two, two, 5, 5, 4, 8), "fasteners.dowel-depth", "a dowel hole shallower than one diameter");
    expect(pinned(two, two, 5, 5, 10, 8, false), "fasteners.not-fixed", "dowelled members that are not fixed together");
  }

  /** Slip of bolted and dowelled joints (`JointSlip`): friction grip without pins, pins alone with them, declared loads checked in the derivation. */
  static function slip():Void {
    function near(actual:Float, expected:Float, tolerance:Float, what:String):Void check(Math.abs(actual - expected) <= tolerance, '$what: $actual, expected $expected');
    near(JointSlip.stressArea("M6"), 20.12, 0.05, "M6 stress area (ISO 898-1: 20.1)");
    near(JointSlip.stressArea("M8"), 36.6, 0.2, "M8 stress area (ISO 898-1: 36.6)");
    near(JointSlip.preload("M6", machinekit.standard.PropertyClass.Steel129, 0.7), 13661, 30, "M6 12.9 at 70 % of proof");
    // The ring's four M6 on a 47.5 mm radius: mu F sum(r), no pins.
    function ring(rad:Float):Array<SlipPoint> return [for (i in 0...4) {x: 0.0, y: rad * Math.cos(Math.PI / 4 + i * Math.PI / 2), z: rad * Math.sin(Math.PI / 4 + i * Math.PI / 2)}];
    var screws:Array<SlipScrew> = [{id: "ring", size: "M6", propertyClass: machinekit.standard.PropertyClass.Steel129, points: ring(47.5)}];
    var load:JointLoad = {label: "A axis", between: ["a", "b"], torque: {peak: 204, momentary: 369}};
    var friction = JointSlip.check(load, screws, [])[0];
    near(friction.capacity, 0.15 * 4 * 13661 * 0.0475, 3.0, "four M6 at 47.5 mm");
    check(friction.primary == "friction" && friction.pinCapacity == 0, "no pins: friction is the capacity");
    var momentaryFactor:Float = cast friction.momentaryFactor;
    check(friction.peakFactor > 1.5 && momentaryFactor < 1.2 && !friction.passes, 'friction alone passes the peak (x${friction.peakFactor}) and misses the momentary load (x$momentaryFactor)');
    // Two 8 mm pins 40 mm out, 12 mm in the weaker part: bearing (240 x 8 x 12 = 23 kN) governs the pin; the torque is the farthest pin's force over sum(r^2)/r_max.
    var pins:Array<SlipPin> = [{id: "pins", diameter: 8, engagement: 12, points: [{x: 0.0, y: 40.0, z: 0.0}, {x: 0.0, y: -40.0, z: 0.0}]}];
    var pinned = JointSlip.check(load, screws, pins)[0];
    near(pinned.pinCapacity, 240 * 8 * 12 * 0.08, 1.0, "two 8 mm pins at 40 mm");
    check(pinned.primary == "pins" && pinned.capacity == pinned.pinCapacity && pinned.frictionCapacity > 0 && pinned.passes, "with pins the pins are the capacity and friction is only reported");
    // A locating key (two ribs either side of a block's base): the lesser of its bearing (240 x 3 x 40 = 28.8 kN) and the root shear (140 x 3.5 x 40 = 19.6 kN); no torque; friction not counted.
    var keyed = JointSlip.check({label: "thrust", between: ["a", "b"], shear: {peak: 1000, momentary: 2000}}, screws, [], [{id: "seat#base", height: 3, length: 40, thickness: 3.5}])[0];
    near(keyed.keyCapacity, 140 * 3.5 * 40, 1.0, "a key's root shear");
    check(keyed.primary == "key" && keyed.capacity == keyed.keyCapacity && keyed.pinCapacity == 0, "with a key the key is the capacity and friction is only reported");
    check(JointSlip.check(load, screws, [], [{id: "k", height: 3, length: 40, thickness: 3.5}])[0].primary == "friction", "a key carries no torque");
    check(JointSlip.check({label: "thrust", between: ["a", "b"], shear: {peak: 1000}}, screws, pins, [{id: "k", height: 3, length: 40, thickness: 3.5}])[0].primary == "pins+key", "pins and a key add");
    // A shear is mu F n, and the pins share it equally.
    var shear = JointSlip.check({label: "thrust", between: ["a", "b"], shear: {peak: 1000}}, screws, [])[0];
    near(shear.capacity, 0.15 * 4 * 13661, 5.0, "shear by friction");
    near(JointSlip.check({label: "thrust", between: ["a", "b"], shear: {peak: 1000}, friction: 0.12}, screws, [])[0].capacity, 0.12 * 4 * 13661, 5.0, "a declared friction coefficient");
    // In an assembly: four M5 12.9 on a square of 50 mm clamp 4 x 0.15 x 9627 N = 5776 N against sliding.
    function loaded(peak:Float, momentary:Null<Float>, between:Array<String>):FastenerDerivation
      return JointFasteners.derive(stack(tappedHost(square(50)), cover(square(50))), {loads: [{label: "load", between: between, shear: {peak: peak, momentary: momentary}}]});
    var sound = loaded(3000, 4000, ["cover", "host"]);
    sound.require();
    check(sound.slips.length == 1 && sound.slips[0].passes && Math.abs(sound.slips[0].capacity - 5776) < 20, 'the declared load is checked: ${sound.slips[0].capacity}');
    check(loaded(4000, null, ["host", "cover"]).codes().indexOf("fasteners.slip") >= 0, "a peak load of 4000 N on 5776 N of friction (x1.44) is a finding");
    check(loaded(3000, 5000, ["host", "cover"]).codes().indexOf("fasteners.slip") >= 0, "a momentary 5000 N on 5776 N (x1.16) is a finding");
    check(loaded(1000, null, ["host", "nowhere"]).codes().indexOf("fasteners.slip") >= 0, "a load between members that share no joint is a finding");
  }

  static function sound():Void {
    // A tapped host: the pattern is the host's, the screw the shortest standard one that engages one diameter.
    var tapped = JointFasteners.derive(stack(tappedHost(square(50)), cover(square(50))));
    tapped.require();
    check(tapped.joints.length == 1 && tapped.count() == 4, "a tapped four-hole joint is one joint of four screws");
    var joint = tapped.joints[0];
    check(joint.size == "M5" && joint.length == 12 && Math.abs(joint.engagement - 6) < 1e-9, 'M5 on grip 6 is x12, got ${joint.size}x${joint.length}');
    check(!joint.drilledToSuit && joint.item.partNumber == "ISO4762-M5x12", "the line is the 12.9 catalogue screw " + joint.item.partNumber);
    // A host drilled to suit in cast iron: 1.5 diameters, so a longer screw, and the host gets its holes.
    var suit = JointFasteners.derive(stack(suitHost(), cover(square(50), "M5")));
    suit.require();
    var drilled = suit.joints[0];
    check(drilled.length == 16 && drilled.drilledToSuit && drilled.hostHoles.length == 4, 'cast iron takes M5x16, got ${drilled.length}');
    check(Math.abs(drilled.tapDrill - 4.2) < 1e-9 && drilled.drillDepth > drilled.threadDepth && drilled.threadDepth >= drilled.engagement, "the host drilling clears the thread");
    var lines = suit.lines();
    check(lines.length == 1 && lines[0].quantity == 4, "the joint is one purchase line of four");
    // The bolt sizes meet: the size comes from either side.
    JointFasteners.derive(stack(tappedHost(square(50)), cover(square(50), "M5"))).require();
    // A property class is carried into the part number.
    var strong = JointFasteners.derive(stack(tappedHost(square(50)), cover(square(50))), {propertyClass: machinekit.standard.PropertyClass.Steel88});
    check(strong.joints[0].item.partNumber == "ISO4762-M5x12-8.8", "the class is in the part number: " + strong.joints[0].item.partNumber);
  }

  /** The derived holes are cut into the parts: the host's tap-drill holes, and the cover's clearance holes and counterbores. */
  static function machining():Void {
    var builder = new AxisBuilder();
    var host = new FaceBlock("HOST", [suitHost()], 21, -11);
    var recess = {diameter: 9.0, depth: 5.0};
    var plate = new FaceBlock("COVER", [cover(square(50), "M5", 6, 5.5, recess, false)], 11, 0);
    builder.place("host", host, AssemblyFrames.translation(0, 0, 0));
    builder.attach("cover", plate, AssemblyFrames.translation(0, 0, 10), "host");
    var before = [host.geometryKey(), plate.geometryKey()];
    var solidHost = host.geometry(), solidPlate = plate.geometry();
    var hostVolume = solidHost.volume(), plateVolume = solidPlate.volume();
    solidHost.close();
    solidPlate.close();
    var derivation = builder.deriveFasteners();
    var joint = derivation.joints[0];
    check(host.geometryKey() != before[0] && plate.geometryKey() != before[1], "machining changes the geometry key, so a cache cannot serve the undrilled solid");
    var drilled = host.geometry(), counterbored = plate.geometry();
    var tap = 4 * Math.PI * Math.pow(joint.tapDrill / 2, 2) * joint.drillDepth;
    check(Math.abs((hostVolume - drilled.volume()) - tap) < 0.01 * tap, 'the host loses ${hostVolume - drilled.volume()} mm3, the four tap-drill holes are $tap');
    var clearance = 4 * (Math.PI * Math.pow(5.5 / 2, 2) * 6 + Math.PI * Math.pow(4.5, 2) * 5);
    check(Math.abs((plateVolume - counterbored.volume()) - clearance) < 0.01 * clearance, 'the cover loses ${plateVolume - counterbored.volume()} mm3, the holes and counterbores are $clearance');
    // The derived cuts name the faces they leave: strong, unique, and under the joint's hole ids.
    for (pair in [{part: drilled, role: "tap"}, {part: counterbored, role: "clearance"}, {part: counterbored, role: "counterbore"}]) {
      var names = pair.part.shape.elementNames(CadKit.ShapeKind.Face), seen:Map<String, Bool> = [], found = 0;
      for (name in names) {
        check(cadkit.ElementNames.isStrong(name) && !seen.exists(name), 'a derived hole face is named strongly and uniquely: $name');
        seen.set(name, true);
        if (name.indexOf(JointFasteners.cutName(joint.id, 1, pair.role) + ":") >= 0 || name.indexOf(JointFasteners.cutName(joint.id, 2, pair.role) + ":") >= 0) found++;
      }
      check(found >= 2, 'the ${pair.role} holes of ${joint.id} are named by hole: $found faces of ${names.length}');
    }
    drilled.close();
    counterbored.close();
    // A part that does not cut derived holes is refused as a host drilled to suit.
    var plain = new AxisBuilder();
    plain.place("host", new PlainBlock([suitHost()]), AssemblyFrames.translation(0, 0, 0));
    plain.attach("cover", new FaceBlock("COVER", [cover(square(50), "M5")]), AssemblyFrames.translation(0, 0, 10), "host");
    expect(plain, "fasteners.machining", "a host that cannot be cut");
  }

  /** A plate with clearance holes and a nut: the washers and the nut come from the derivation, and set the length. */
  static function washersAndNuts():Void {
    var plate = BoltedFace.through("top", frameAtTop(10), square(50), 5.5, 10);
    var builder = stack(plate, cover(square(50), "M5", 6, 5.5, null, true, true));
    var derived = JointFasteners.derive(builder);
    derived.require();
    var joint = derived.joints[0];
    // 6 grip + 1 washer + 10 plate + 1 washer + 4.7 nut + 2 pitches (1.6) = 24.3: the next standard length is 25.
    check(joint.nutted && joint.length == 25 && Math.abs(joint.stack - 18) < 1e-9, 'a bolt through a plate: x${joint.length}, stack ${joint.stack}');
    var counts = new Map<String, Int>();
    for (line in derived.lines()) counts.set(line.item.partNumber, line.quantity);
    check(counts.get("ISO4762-M5x25") == 4 && counts.get("ISO7089-M5") == 8 && counts.get("ISO4032-M5") == 4, 'bolt lines: ${[for (k in counts.keys()) k + "=" + counts.get(k)]}');
    // The nut side is a plate too: its holes must pass the screw.
    expect(stack(BoltedFace.through("top", frameAtTop(10), square(50), 4.0, 10), cover(square(50), "M5")), "fasteners.clearance", "4 mm holes in the plate");
    expect(stack(BoltedFace.through("top", frameAtTop(10), square(40), 5.5, 10), cover(square(50), "M5")), "fasteners.pattern", "a plate drilled on another pitch");
    // A washer under the head lifts it: it must still fit the recess.
    expect(stack(tappedHost(square(50)), cover(square(50), null, 6, 5.5, {diameter: 9.0, depth: 5.5}, true, true)), "fasteners.head-clearance", "an M5 head on a washer (5.0 + 1.0) in a 5.5 mm recess");
    var washed = JointFasteners.derive(stack(tappedHost(square(50)), cover(square(50), null, 6, 5.5, null, true, true)));
    washed.require();
    check(washed.joints[0].length == 12 && washed.joints[0].extras.length == 1 && washed.joints[0].extras[0].count == 4, 'a washer under the head: x${washed.joints[0].length}');
  }

  /** A clamped footprint drilled to suit takes the pattern of each tapped block face under it. */
  static function suitSeats():Void {
    function seat(width:Float, height:Float):BoltedFace
      return BoltedFace.clampedToSuit("seat", BoltedFace.frameOf(origin, up, across), width, height, 5.5, 6, "M5");
    function two(footprint:BoltedFace, tapped:Array<BoltedFace>):AxisBuilder {
      var builder = new AxisBuilder();
      builder.place("cover", new FaceBlock("COVER", [footprint], 6, 0), AssemblyFrames.translation(0, 0, 10));
      var index = 0;
      for (face in tapped) {
        builder.attach("host" + index, new FaceBlock("HOST" + index, [face], 8, 2), AssemblyFrames.translation(index * 70, 0, 0), "cover");
        index++;
      }
      return builder;
    }
    function tappedAt(x:Float):BoltedFace
      return BoltedFace.tapped("top", frameAtTop(10), square(30), "M5", 8, 8, true);
    // One 200 x 60 seat takes two blocks, each with its own four holes; the pattern comes from the blocks.
    var good = JointFasteners.derive(two(seat(200, 60), [tappedAt(0), tappedAt(70)]));
    good.require();
    check(good.joints.length == 2 && good.count() == 8, "a footprint pairs with both blocks: " + good.joints.length + " joints");
    var cuts = good.machining.get("cover");
    check(cuts != null && cuts.length == 8, "the seat is cut by the derivation");
    // A hole close to the footprint's edge, and a footprint with no block under it.
    check(JointFasteners.derive(two(seat(200, 34), [tappedAt(0), tappedAt(70)])).codes().indexOf("fasteners.edge") >= 0, "a hole 2 mm from the footprint's edge");
    check(JointFasteners.derive(two(seat(200, 60), [])).codes().indexOf("fasteners.unmatched") >= 0, "a footprint with no block under it");
    var both = new AxisBuilder();
    both.place("cover", new FaceBlock("COVER", [seat(60, 60)], 6, 0), AssemblyFrames.translation(0, 0, 10));
    both.place("host", new FaceBlock("HOST", [BoltedFace.drilledToSuit("top", frameAtTop(10), 60, 60, 8, 8, CastIron)], 8, 2), AssemblyFrames.translation(0, 0, 0));
    check(JointFasteners.derive(both).codes().indexOf("fasteners.suit") >= 0, "two footprints: neither knows the pattern");
  }

  /** The derived fasteners fit: the holes exist in the solids, the heads stand clear and a driver reaches them. */
  static function fit():Void {
    // A cover (z 0..6 over the host top) bolted down with M5: the host is drilled, the cover's holes are drawn only if cut.
    function fitting(cap:Float, washer:Bool = false, lidAt:Null<Float> = null, lidFixed:Bool = true, drawn:Bool = false):AxisBuilder {
      var builder = new AxisBuilder();
      builder.place("host", new FaceBlock("HOST", [suitHost()], 21, -11), AssemblyFrames.translation(0, 0, 0));
      builder.attach("cover", new FaceBlock("COVER", [cover(square(50), "M5", 6, 5.5, null, drawn, washer)], 6, 0), AssemblyFrames.translation(0, 0, 10), "host");
      if (lidAt != null) {
        var lid = new FaceBlock("LID", [], 4, 0);
        if (lidFixed) builder.attach("lid", lid, AssemblyFrames.translation(0, 0, 10 + lidAt), "cover");
        else builder.place("lid", lid, AssemblyFrames.translation(0, 0, 10 + lidAt));
      }
      builder.deriveFasteners();
      return builder;
    }
    function codes(builder:AxisBuilder, ?fittedBefore:Array<machinekit.assembly.AssemblyIntegrity.FastenerFitOrder>):Array<String> {
      var report = machinekit.assembly.AssemblyIntegrity.check(builder, {fasteners: true, fittedBefore: fittedBefore});
      return [for (item in report.diagnostics.items) if (StringTools.startsWith(item.code, "fasteners.")) item.code];
    }
    check(codes(fitting(0)).length == 0, "bolted down with its holes cut, a clear cover fits: " + codes(fitting(0)));
    // The M5 head stands 5 mm proud of a 6 mm cover (z 6..11): a lid 8 mm above the cover (z 8) sits on it.
    check(codes(fitting(0, false, 8)).indexOf("fasteners.head-clearance") >= 0, "a lid on the heads is reported: " + codes(fitting(0, false, 8)));
    // A lid 40 mm up clears the head and the 25 mm driver reach (to z 36), and one fixed at 30 mm blocks the driver only.
    check(codes(fitting(0, false, 40)).length == 0, "a lid clear of head and driver: " + codes(fitting(0, false, 40)));
    check(codes(fitting(0, false, 30)).indexOf("fasteners.access") >= 0 && codes(fitting(0, false, 30)).indexOf("fasteners.head-clearance") < 0, "a lid in the driver's way is an access finding: " + codes(fitting(0, false, 30)));
    // The same lid, not held with the cover (it could be moved away), is not in the way.
    check(codes(fitting(0, false, 30, false)).length == 0, "a separate body is no obstacle to the driver: " + codes(fitting(0, false, 30, false)));
    // The same lid fixed with the cover but installed after the cover's screws are fitted: the driver reaches them first. Per fastener: the lid still
    // counts for the holes and heads (a lid on the heads is an error whatever the order), and it cannot be installed after a joint it belongs to.
    check(codes(fitting(0, false, 30), [{member: "lid", fastened: ["cover"]}]).length == 0, "a lid installed after the screws is no obstacle: " + codes(fitting(0, false, 30), [{member: "lid", fastened: ["cover"]}]));
    check(codes(fitting(0, false, 30), [{member: "lid", fastened: ["host"]}]).length == 0, "the rule names either member of the joint");
    check(codes(fitting(0, false, 30), [{member: "lid", fastened: ["elsewhere"]}]).indexOf("fasteners.access") >= 0, "an order that does not name the joint's members changes nothing");
    check(codes(fitting(0, false, 8), [{member: "lid", fastened: ["cover"]}]).indexOf("fasteners.head-clearance") >= 0, "an installed lid still stands on the heads");
    check(codes(fitting(0), [{member: "cover", fastened: ["cover"]}]).indexOf("fasteners.order") >= 0, "a part cannot be installed after its own joint");
    check(codes(fitting(0, false, 30), [{member: "li", fastened: ["cover"]}]).indexOf("fasteners.access") >= 0, "a member name is exact");
    // A cover whose holes are declared drawn but are not: the shank meets its material.
    check(codes(fitting(0, false, null, true, true)).indexOf("fasteners.hole") >= 0, "undrawn holes in a cover are reported: " + codes(fitting(0, false, null, true, true)));
  }

  static function defects():Void {
    // Missing data.
        expect(stack(suitHost(), cover(square(50)), 2), "fasteners.unmatched", "a cover 2 mm off its host");
    expect(stack(BoltedFace.drilledToSuit("top", frameAtTop(10), 100, 100, 20, 21, CastIron), cover(square(50), null)), "fasteners.size", "no side names the size");
    expect(stack(suitHost(), cover(square(50), "M7")), "fasteners.size", "a size the catalogue lacks");
    var lonely = new AxisBuilder();
    lonely.place("cover", new FaceBlock("LONELY", [cover(square(50), "M5")]), AssemblyFrames.translation(0, 0, 0));
    expect(lonely, "fasteners.unmatched", "a clamped face with nothing to thread into");
    var threw = false;
    try BoltedFace.clamped("x", AssemblyFrames.identity(), [], 5.5, 6) catch (_:Dynamic) threw = true;
    check(threw, "a clamped face with no holes is refused");
    // Incompatible patterns.
    expect(stack(tappedHost(square(50)), cover(square(40))), "fasteners.pattern", "holes on a different pitch");
    expect(stack(tappedHost(square(50)), cover([{x: 25.0, y: 25.0}, {x: -25.0, y: 25.0}, {x: 25.0, y: -25.0}, {x: -25.0, y: -20.0}])), "fasteners.pattern", "one hole 5 mm off");
    expect(stack(tappedHost(square(50), "M6", 8, 8), cover(square(50), "M5")), "fasteners.thread", "M5 holes over an M6 thread");
    expect(stack(suitHost(60, 60), cover(square(50), "M5")), "fasteners.edge", "holes too near the edge of the seat");
    // A bore under part of the seat: the wall is checked where the holes are.
    var bored = BoltedFace.drilledToSuit("top", frameAtTop(10), 100, 100, 20, 21, CastIron, null, false, (u, _) -> u < 0 ? 8.0 : 21.0);
    expect(stack(bored, cover(square(50), "M5")), "fasteners.back-wall", "holes over a bore that leaves 8 mm");
    JointFasteners.derive(stack(bored, cover([{x: 25.0, y: 0.0}, {x: 40.0, y: 0.0}], "M5"))).require();
    // Grip and engagement.
    expect(stack(BoltedFace.tapped("top", frameAtTop(10), square(50), "M5", 3, 8, false), cover(square(50))), "fasteners.engagement", "a blind thread 3 mm deep");
    expect(stack(suitHost(100, 100, 12), cover(square(50), "M5")), "fasteners.back-wall", "a 12 mm wall tapped to suit");
    expect(stack(tappedHost(square(50)), cover(square(50), null, 400)), "fasteners.length", "a grip beyond every standard length");
    expect(stack(tappedHost(square(50)), cover(square(50), null, 6, 4.0)), "fasteners.clearance", "4 mm holes for an M5");
    // Head clearance.
    expect(stack(tappedHost(square(50)), cover(square(50), null, 6, 5.5, {diameter: 7.0, depth: 6.0})), "fasteners.head-clearance", "an M5 head (8.5) in a 7 mm recess");
    expect(stack(tappedHost(square(50)), cover(square(50), null, 6, 5.5, {diameter: 9.0, depth: 4.0})), "fasteners.head-clearance", "an M5 head (5) in a 4 mm deep recess");
    JointFasteners.derive(stack(tappedHost(square(50)), cover(square(50), null, 6, 5.5, {diameter: 9.0, depth: 5.0}))).require();
    // Held together by fixed mates.
    expect(stack(tappedHost(square(50)), cover(square(50)), 0, false), "fasteners.not-fixed", "members with no fixed mate");
    // Duplicate counting: two covers on the same tapped holes, and one declared twice.
    var twin = stack(tappedHost(square(50)), cover(square(50)));
    twin.attach("cover2", new FaceBlock("COVER2", [cover(square(50))]), AssemblyFrames.translation(0, 0, 10), "host");
    expect(twin, "fasteners.duplicate", "two covers on the same tapped holes");
    var coincident = stack(suitHost(), cover(square(50), "M5"));
    coincident.attach("cover2", new FaceBlock("COVER2", [cover(square(50), "M5")]), AssemblyFrames.translation(0, 0, 10), "host");
    expect(coincident, "fasteners.duplicate", "two covers deriving the same hole of a seat");
    var crowded = stack(suitHost(), cover(square(50), "M5"));
    crowded.attach("cover2", new FaceBlock("COVER2", [cover([{x: 26.0, y: 25.0}], "M5")]), AssemblyFrames.translation(0, 0, 10), "host");
    expect(crowded, "fasteners.spacing", "a hole derived 1 mm from another");
    // Two receiving faces in one plane (an insert's M5 and M6 holes): the cover's pattern picks the one it lands on.
    var plane = new AxisBuilder();
    plane.place("host", new FaceBlock("HOST", [tappedHost(square(50)), BoltedFace.tapped("top6", frameAtTop(10), [{x: 0.0, y: 0.0}, {x: 30.0, y: 0.0}], "M6", 8, 8, true)]),
      AssemblyFrames.translation(0, 0, 0));
    plane.attach("cover", new FaceBlock("COVER", [cover(square(50))]), AssemblyFrames.translation(0, 0, 10), "host");
    JointFasteners.derive(plane).require();
    // The close holes of one catalogue pattern (M6 holes 13 mm apart) are the part's, not a crowding by two joints.
    JointFasteners.derive(stack(suitHost(), cover([{x: 20.0, y: 0.0}, {x: 20.0, y: 13.0}], "M6", 6, 6.6))).require();
    var twice = stack(tappedHost(square(50)), cover(square(50)));
    twice.deriveFasteners();
    var again = false;
    try twice.deriveFasteners() catch (_:Dynamic) again = true;
    check(again, "deriving a second time must not count the screws twice");
  }

  static function mill():Void {
    var machine = new BenchMill();
    var derivation = machine.derivedFasteners();
    if (derivation == null) throw "Joint fasteners: the bench mill derives its fasteners";
    var railHoles = 0;
    for (entry in machine.components()) if (Std.isOfType(entry.component, LinearRail)) {
      var rail:LinearRail = cast entry.component;
      railHoles += rail.holePositions.length;
    }
    // Every carriage block is bolted by its four top threads from the casting it carries, with the same M4 x 16.
    var blocks = 0;
    for (entry in machine.components()) if (Std.isOfType(entry.component, machinekit.motion.LinearRailBlock)) blocks++;
    check(blocks == 12, "the mill has twelve carriage blocks, got " + blocks);
    var count = new Map<String, Int>();
    for (line in derivation.lines()) count.set(line.item.partNumber, line.quantity);
    check(count.get("ISO4762-M4x16") == railHoles + 4 * blocks && railHoles > 0,
      'M4x16 screws must be the rail holes $railHoles and four per block ($blocks), got ${count.get("ISO4762-M4x16")}');
    check(count.get("ISO4762-M5x12") == 24 && count.get("ISO4762-M6x12") == 4 && count.get("ISO4762-M5x10") == 56, 'motor and cover flange screws: ${[for (k in count.keys()) k + "=" + count.get(k)]}');
    var total = 0;
    for (key in count.keys()) total += count.get(key);
    // The three ball nuts (six flange bolts each) are bolted through their adapter plates with a washer and a nut.
    check(count.get("ISO4762-M5x30") == 18 && count.get("ISO7089-M5") == 90 && count.get("ISO4032-M5") == 18, 'nut flange bolts: ${[for (k in count.keys()) k + "=" + count.get(k)]}');
    var feet = [for (joint in derivation.joints) if (joint.clampedFace == "foot") joint];
    var footScrews = 0;
    for (joint in feet) footScrews += joint.count;
    // Every screw support is bolted down by its feet: the Y and X supports to their stands and bridges, the Z supports to the steel bosses cast into the column.
    check(feet.length == 6 && footScrews == 18, 'six supports are bolted: ${feet.length} joints, $footScrews screws');
    for (joint in feet) check(joint.drilledToSuit && joint.size == "M5" && joint.engagement >= joint.minEngagement - 1e-9, 'support joint ${joint.id}: ${joint.size} x${joint.length}');
    // The six covers add twelve flange joints: 72 screws and 72 head washers, measured before making them standard.
    var flangeJoints = [for (joint in derivation.joints) if (StringTools.startsWith(joint.clampedMember, "cover")) joint];
    var flangeScrews = 0;
    for (joint in flangeJoints) flangeScrews += joint.count;
    check(flangeJoints.length == 12 && flangeScrews == 72, 'cover flange joints: ${flangeJoints.length}, screws: $flangeScrews');
    var mounts = [for (joint in derivation.joints) if (StringTools.startsWith(joint.clampedMember, "wall") || StringTools.startsWith(joint.clampedMember, "carrier")) joint];
    var mountScrews = 0; for (joint in mounts) mountScrews += joint.count;
    check(mounts.length == 9 && mountScrews == 16 && derivation.dowels.length == 8 && derivation.dowelCount() == 16,
      "all eight cover mounts have two derived screws and two dowels");
    check(count.get("ISO4762-M5x25") == 12 && count.get("ISO4762-M5x20") == 4 && count.get("ISO8734-4x8") == 16,
      "cover mount screws and dowels follow their actual grip and host material");
    check(total == railHoles + 4 * blocks + footScrews + 12 + 18 + 18 + 18 + 2 * flangeScrews + mountScrews + derivation.dowelCount(), 'derived screws, washers, nuts and dowels only: $total');
    var rails = [for (joint in derivation.joints) if (joint.drilledToSuit && joint.clampedFace == "base") joint];
    var blockJoints = [for (joint in derivation.joints) if (joint.clampedFace == "blockLeft" || joint.clampedFace == "blockRight") joint];
    check(blockJoints.length == blocks, "one joint per block, got " + blockJoints.length);
    for (joint in blockJoints) check(joint.length == 16 && joint.grip == 12 && joint.engagement >= joint.minEngagement - 1e-9
      && joint.count == 4 && !joint.drilledToSuit, 'block joint ${joint.id}: x${joint.length}, grip ${joint.grip}, engagement ${joint.engagement}');
    check(rails.length == 6, "six rails are bolted to cast pads, got " + rails.length);
    // The derived fasteners fit the mill's solids: holes cut, heads and drivers clear.
    var started = Sys.time();
    var fit = machinekit.assembly.AssemblyIntegrity.fasteners(machine, {fasteners: true, removable: machine.workholdingMembers(), fittedBefore: machine.coverInstallationOrder()});
    Sys.println('bench mill fastener fit: ${fit.errors().length} findings in ${Math.round(Sys.time() - started)} s');
    for (item in fit.errors()) Sys.println("  " + item.code + ": " + item.message);
    // Lead ruling 2026-11-07: run the new made-part edge rule on every machine,
    // but report other examples without changing their hardware in this session.
    // Preserve the pre-existing holes/head/driver regression gate independently.
    var otherFitErrors = fit.errors().filter(item -> item.code != "fasteners.edge-support");
    var edgeFindings = fit.diagnostics.items.filter(item -> item.code == "fasteners.edge-support");
    Sys.println('unmodified bench-mill made-part edge findings (reported separately): ${edgeFindings.length}');
    check(otherFitErrors.length == 0, 'the mill holes, heads and driver corridors fit: ${[for (item in otherFitErrors) item.message].join("; ")}');
    // The steel bosses cast into the column fill the pockets cut for them: no volume shared with the casting, and the support's foot on the boss.
    var poses = machine.solvedPoses(), posed = new machinekit.assembly.PosedParts();
    var parts = new Map<String, machinekit.component.MachineComponent>();
    for (entry in machine.components()) parts.set(entry.id, entry.component);
    for (end in 0...2) {
      var boss = 'screwZStand$end';
      var shared = posed.overlap(parts.get(boss), poses.get(boss), parts.get("column"), poses.get("column"));
      check(shared < 1e-3, 'the Z boss $end shares ${shared} mm3 with the column casting');
      check(machine.column.bosses.length == 2 && parts.get(boss).fasteningFaces().length == 1, "each Z support has one boss with one tapped foot face");
    }
    posed.close();
    Sys.println('bench mill: ${derivation.joints.length} joints, ${derivation.count()} screws (${railHoles} rail, ${4 * blocks} block, ${footScrews} support, 12 motor, 18 nut flange, $flangeScrews cover flange, $mountScrews cover mount), ${derivation.dowelCount()} dowels, ' + [for (k in count.keys()) k + " x" + count.get(k)].join(", "));
    // The drilled seats are in the castings' solids: one tap-drill hole per rail screw, none anywhere else.
    var cut = 0;
    for (entry in machine.components()) {
      var cuts = entry.component.machinedCuts();
      if (cuts.length > 0) check(Std.isOfType(entry.component, machinekit.milling.MillCasting) || StringTools.startsWith(entry.id, "screwXNearBridge") || StringTools.startsWith(entry.id, "screwXFarBridge") || StringTools.startsWith(entry.id, "screwYStand") || StringTools.startsWith(entry.id, "screwZStand") || Std.isOfType(entry.component, machinekit.cover.WayCoverMount) || Std.isOfType(entry.component, machinekit.cover.WayCoverSection), entry.id + " is machined but is not a casting, support stand or cover part");
      cut += cuts.length;
    }
    // Each block screw cuts its clearance hole and its counterbored pocket into the casting it is bolted through.
    check(cut == railHoles + 2 * 4 * blocks + footScrews + 2 * flangeScrews + 2 * mountScrews + 2 * derivation.dowelCount(),
      'every rail hole is tapped, every block screw has a hole and pocket, every cover flange/mount screw and dowel cuts both faces: $cut cuts');
    for (joint in rails) check(joint.length == 16 && joint.engagement >= joint.minEngagement - 1e-9, 'rail joint ${joint.id}: x${joint.length} engages ${joint.engagement} of ${joint.minEngagement}');

    // Recipe-backed hosts and purchased flanges keep their actual machining when an enclosing assembly takes a snapshot.
    var nested = new MachineAssembly();
    nested.include("mill", machine, materia.assembly.AssemblyFrames.translation(73, -21, 41));
    for (saved in [{assembly: machine.snapshot(), prefix: ""}, {assembly: nested, prefix: "mill/"}]) {
      var copyParts = new Map<String, machinekit.component.MachineComponent>();
      for (entry in saved.assembly.components()) copyParts.set(entry.id, entry.component);
      for (entry in machine.components()) if (entry.component.componentType() != null && entry.component.machinedCuts().length > 0)
        check(copyParts.get(saved.prefix + entry.id).machinedCuts().length == entry.component.machinedCuts().length,
          saved.prefix + entry.id + " preserves its derived hole table");
      for (id in ["base", "head", "coverYFront-fixed"]) {
        var original:MachineComponent = parts.get(id);
        var copied:MachineComponent = copyParts.get(saved.prefix + id);
        var originalBom = original.bom, copiedBom = copied.bom;
        check(originalBom.fingerprint == copiedBom.fingerprint, saved.prefix + id + " preserves its machined revision");
        var sourceSolid = original.geometry(), copySolid = copied.geometry();
        var missing = sourceSolid.subtract(copySolid), added = copySolid.subtract(sourceSolid);
        var difference = missing.shape.volume() + added.shape.volume();
        missing.close();added.close();sourceSolid.close();copySolid.close();
        check(difference < 0.01, saved.prefix + id + ' preserves the drilled solid: $difference mm3 difference');
      }
    }
  }
}
