package machinekit.hmi;

import cadkit.modeling.Part;
import cadkit.modeling.Polygon;
import cadkit.threads.ThreadSpec;
import cadkit.threads.ThreadedFace.ThreadStart;
import cadkit.threads.Threads;
import machinekit.component.ComponentDetail;
import machinekit.component.MachineComponent;
import machinekit.component.Solids;
import machinekit.standard.FastenerThreads;

/**
 * A male-female hex standoff, brass: a hex body `length` long with an internal thread in its bottom end and an external stud
 * on its top. Frame: the bottom face at z = 0 and the axis along +z, so the body fills z from 0 to `length` and the stud
 * rises above it. Both threads are textures (`Threads.attach`) on the nominal diameter, which is also the bore of the insert
 * the stud goes into and the shank of the screw that goes into the bottom.
 */
class HexStandoff extends MachineComponent {
  /** Lengths of the standard stock, millimetres. */
  public static final STANDARD_LENGTHS:Array<Float> = [6.0, 8.0, 10.0, 12.0, 15.0, 20.0, 25.0, 30.0, 35.0, 40.0];

  public final name:String;
  public final length:Float;
  public final studLength:Float;
  public final diameter:Float;
  public final acrossFlats:Float;
  public final boreDepth:Float;
  public final thread:ThreadSpec;

  public function new(
    name:String,
    diameter:Float,
    pitch:Float,
    acrossFlats:Float,
    length:Float,
    studLength:Float,
    boreDepth:Float
  ) {
    super('STANDOFF-MF-$name-$length-$studLength', 'hex standoff $name x $length, male $studLength', "brass");
    this.name = name;
    this.diameter = diameter;
    this.thread = FastenerThreads.metric(diameter, pitch);
    this.acrossFlats = acrossFlats;
    this.length = length;
    this.studLength = studLength;
    this.boreDepth = boreDepth;
  }

  /** M2.5 with a 5 mm hex, a 4 mm stud and a 5 mm tapped end: the standard stock shortened to the stud the pendant's bosses take. */
  public static function m2_5(length:Float):HexStandoff return new HexStandoff("M2.5",
    2.5, 0.45, 5.0, length, 4.0, 5.0);

  /** The shortest standard length that is at least `atLeast`. */
  public static function standardLength(atLeast:Float):Float {
    for (l in STANDARD_LENGTHS) if (l >= atLeast - 1e-9) return l;
    throw 'No standard standoff length reaches $atLeast mm';
  }

  override public function hasGeometry():Bool return true;

  override public function geometry(detail:ComponentDetail = Preview):Part {
    var body = Part.prism(Polygon.regular(6, acrossFlats), 0.0, length);
    var bore = Solids.named(Part.cylinderSpan(diameter / 2, -0.1, boreDepth), "bore");
    var tool = new Part(Threads.attach(bore.shape, "bore:cyl.side", thread.mating(), AxisMinimum));
    bore.close();
    var shell = Solids.cut(body, [tool]);
    var rod = Solids.named(Part.cylinderSpan(diameter / 2, length, length + studLength), "stud");
    var stud = new Part(Threads.attach(rod.shape, "stud:cyl.side", thread, AxisMaximum, studLength - 0.5, 0.5));
    rod.close();
    return Solids.union([shell, stud]);
  }
}
