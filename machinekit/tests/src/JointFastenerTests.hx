import machinekit.assembly.AxisBuilder;
import machinekit.assembly.JointFasteners;
import machinekit.component.BoltedFace;
import machinekit.component.BoltedFace.BoltHole;
import machinekit.component.BoltedFace.ThreadMaterial;
import machinekit.component.MachineComponent;
import machinekit.motion.LinearRail;
import materia.assembly.AssemblyFrames;

typedef Vec = {x:Float, y:Float, z:Float}

/** A part that only states its bolted faces, for exercising `JointFasteners`. */
class FaceBlock extends MachineComponent {
  final faces:Array<BoltedFace>;

  public function new(name:String, faces:Array<BoltedFace>) {
    super("TEST-FACE-BLOCK-" + name, "Face block " + name, "steel", true);
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

  static function cover(holes:Array<BoltHole>, ?size:String, grip:Float = 6, clearance:Float = 5.5, ?recess:machinekit.component.BoltedFace.BoltRecess):BoltedFace
    return BoltedFace.clamped("bottom", BoltedFace.frameOf(origin, up, across), holes, clearance, grip, size, recess);

  static function tappedHost(holes:Array<BoltHole>, size:String = "M5", depth:Float = 8, thickness:Float = 8, through:Bool = true):BoltedFace
    return BoltedFace.tapped("top", frameAtTop(10), holes, size, depth, thickness, through);

  static function suitHost(width:Float = 100, height:Float = 100, thickness:Float = 21, ?material:ThreadMaterial):BoltedFace
    return BoltedFace.drilledToSuit("top", frameAtTop(10), width, height, Math.min(20, thickness), thickness, material ?? CastIron);

  static function expect(builder:AxisBuilder, code:String, label:String):Void {
    var result = JointFasteners.derive(builder);
    check(result.codes().indexOf(code) >= 0, '$label should report $code, found ${result.codes()}: ${[for (i in result.diagnostics.items) i.message].join("; ")}');
  }

  public static function main():Void run();

  public static function run():Void {
    sound();
    defects();
    mill();
    Sys.println("joint fasteners: derivation, validation and the bench mill's rail and motor screws pass");
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
    var count = new Map<String, Int>();
    for (line in derivation.lines()) count.set(line.item.partNumber, line.quantity);
    check(count.get("ISO4762-M4x16") == railHoles && railHoles > 0, 'rail screws M4x16 must equal the rail holes $railHoles, got ${count.get("ISO4762-M4x16")}');
    check(count.get("ISO4762-M5x12") == 8 && count.get("ISO4762-M6x12") == 4, 'motor screws: ${[for (k in count.keys()) k + "=" + count.get(k)]}');
    var total = 0;
    for (key in count.keys()) total += count.get(key);
    check(total == railHoles + 12, "no other screw is derived");
    var rails = [for (joint in derivation.joints) if (joint.drilledToSuit) joint];
    check(rails.length == 6, "six rails are bolted to cast pads, got " + rails.length);
    Sys.println('bench mill: ${derivation.joints.length} joints, ${derivation.count()} screws (${railHoles} rail, 12 motor), ' + [for (k in count.keys()) k + " x" + count.get(k)].join(", "));
    for (joint in rails) check(joint.length == 16 && joint.engagement >= joint.minEngagement - 1e-9, 'rail joint ${joint.id}: x${joint.length} engages ${joint.engagement} of ${joint.minEngagement}');
  }
}
