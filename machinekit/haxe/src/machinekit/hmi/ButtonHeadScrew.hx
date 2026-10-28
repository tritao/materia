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
 * An ISO 7380 button head socket screw, A2 stainless. Frame: the head's bearing face at z = 0, the domed head (with a hex
 * socket) above it and the shank below. The thread is a texture on the shank (`Threads.attach`), not geometry: the shank
 * is the nominal diameter, which is also the bore of its insert.
 */
class ButtonHeadScrew extends MachineComponent {
  public final name:String;
  public final diameter:Float;
  public final length:Float;
  public final headDiameter:Float;
  public final headHeight:Float;
  public final socketAcrossFlats:Float;
  public final socketDepth:Float;
  public final thread:ThreadSpec;

  public function new(
    name:String,
    diameter:Float,
    pitch:Float,
    length:Float,
    headDiameter:Float,
    headHeight:Float,
    socketAcrossFlats:Float,
    socketDepth:Float
  ) {
    super('ISO7380-$name-$length', 'button head screw $name x $length', "stainless A2-70");
    this.name = name;
    this.thread = FastenerThreads.metric(diameter, pitch);
    this.diameter = diameter;
    this.length = length;
    this.headDiameter = headDiameter;
    this.headHeight = headHeight;
    this.socketAcrossFlats = socketAcrossFlats;
    this.socketDepth = socketDepth;
  }

  /** M3 x `length`: 5.7 mm head, 1.65 high, 2 mm hex socket. */
  public static function m3(length:Float):ButtonHeadScrew return new ButtonHeadScrew(
    "M3",
    3.0,
    0.5,
    length,
    5.7,
    1.65,
    2.0,
    1.0
  );

  /** M2.5 x `length`: 4.7 mm head, 1.4 high, 1.5 mm hex socket. */
  public static function m2_5(length:Float):ButtonHeadScrew return new ButtonHeadScrew(
    "M2.5",
    2.5,
    0.45,
    length,
    4.7,
    1.4,
    1.5,
    0.8
  );

  override public function hasGeometry():Bool return true;

  override public function geometry(detail:ComponentDetail = Preview):Part {
    var h = headDiameter / 2, k = headHeight;
    var head = Solids.named(
      Part.revolve([
        {r: 0.0, z: 0.0},
        {r: h, z: 0.0},
        {r: h, z: k * 0.45},
        {
          r: h * 0.88,
          z: k * 0.8
        },
        {
          r: h * 0.6,
          z: k * 0.97
        },
        {
          r: h * 0.3,
          z: k
        },
        {
          r: 0.0,
          z: k
        }
      ]),
      "head"
    );
    var shank = Solids.named(Part.cylinderSpan(diameter / 2, -length, 0.0), "shank");
    var threaded = new Part(Threads.attach(shank.shape, "shank:cyl.side", thread, AxisMaximum, length));
    shank.close();
    var body = Solids.union([head, threaded]);
    var socket = Solids.named(Part.prism(Polygon.regular(6, socketAcrossFlats), k - socketDepth, k + 0.1), "socket");
    return Solids.cut(body, [socket]);
  }
}
