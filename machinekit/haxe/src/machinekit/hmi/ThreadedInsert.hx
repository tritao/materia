package machinekit.hmi;

import cadkit.modeling.Part;
import cadkit.threads.ThreadSpec;
import cadkit.threads.ThreadedFace.ThreadStart;
import cadkit.threads.Threads;
import machinekit.component.Solids;
import machinekit.standard.FastenerThreads;
import machinekit.component.ComponentDetail;
import machinekit.component.MachineComponent;

/**
 * A heat-set brass insert, pressed into a printed hole with a soldering iron. Frame: the top (the face the screw enters)
 * at z = 0, the insert below it. It is modelled at its installed envelope, the pilot hole's diameter, because melted
 * plastic flows round the knurls; the bore is the screw's nominal diameter, since threads are semantic. A design
 * derives its pilot hole from `pilotDiameter` and `pilotDepth` so hole and insert cannot disagree.
 */
class ThreadedInsert extends MachineComponent {
  public final name:String;
  public final thread:ThreadSpec;

  /** Printed hole the insert goes into. */
  public final pilotDiameter:Float;

  public final pilotDepth:Float;
  public final length:Float;
  public final boreDiameter:Float;

  public function new(name:String,
    pitch:Float, pilotDiameter:Float, pilotDepth:Float, length:Float, boreDiameter:Float) {
    super('HEATSET-$name-$length', 'heat-set insert $name x $length', "brass");
    this.name = name;
    this.thread = FastenerThreads.metric(boreDiameter, pitch).mating();
    this.pilotDiameter = pilotDiameter;
    this.pilotDepth = pilotDepth;
    this.length = length;
    this.boreDiameter = boreDiameter;
  }

  /** M3 x 5.7 (4.6 mm knurled body) in a 4.2 x 7.5 hole. */
  public static function m3():ThreadedInsert return new ThreadedInsert("M3", 0.5, 4.2, 7.5, 5.7, 3.0);

  /** M2.5 x 4 (3.6 mm knurled body) in a 3.4 x 6 hole. */
  public static function m2_5():ThreadedInsert return new ThreadedInsert("M2.5", 0.45, 3.4, 6.0, 4.0, 2.5);

  /** M2.5 x 3 (3.6 mm knurled body) in a 3.4 x 4.4 hole, for where there is little material behind it. */
  public static function m2_5Short():ThreadedInsert return new ThreadedInsert("M2.5", 0.45, 3.4, 4.4, 3.0, 2.5);

  override public function hasGeometry():Bool return true;

  override public function geometry(detail:ComponentDetail = Preview):Part {
    // A tube of the installed envelope with a chamfered foot and three retention grooves; its bore carries the thread texture.
    var r = pilotDiameter / 2;
    var section = [{r: 0.0, z: 0.0}, {r: r, z: 0.0}];
    for (i in 1...4) {
      var z = -length * i / 4.0;
      section.push({r: r, z: z + 0.3});
      section.push({r: r - 0.3, z: z + 0.15});
      section.push({r: r - 0.3, z: z - 0.15});
      section.push({r: r, z: z - 0.3});
    }
    section.push({r: r, z: -length + 0.3});
    section.push({r: r - 0.3, z: -length});
    section.push({r: 0.0, z: -length});
    var body = Part.revolve(section);
    var bore = Solids.named(Part.cylinderSpan(boreDiameter / 2, -length - 0.1, 0.1), "bore");
    var threaded = new Part(Threads.attach(bore.shape, "bore:cyl.side", thread, AxisMaximum, length));
    bore.close();
    return Solids.cut(body, [threaded]);
  }
}
