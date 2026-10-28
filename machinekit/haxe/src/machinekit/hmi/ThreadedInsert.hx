package machinekit.hmi;

import cadkit.modeling.Part;
import machinekit.component.ComponentDetail;
import machinekit.component.MachineComponent;

/**
 * A heat-set brass insert, pressed into a printed hole with a soldering iron. Frame: the top (the face the screw enters)
 * at z = 0, the insert below it. It is modelled at its installed envelope, the pilot hole's diameter, because melted
 * plastic flows round the knurls; the bore is the screw's nominal diameter, since threads are semantic. A design
 * derives its pilot hole from `pilotDiameter` and `pilotDepth` so hole and insert cannot disagree.
 */
class ThreadedInsert extends MachineComponent {
  public final thread:String;

  /** Printed hole the insert goes into. */
  public final pilotDiameter:Float;

  public final pilotDepth:Float;
  public final length:Float;
  public final boreDiameter:Float;

  public function new(thread:String, pilotDiameter:Float, pilotDepth:Float, length:Float, boreDiameter:Float) {
    super('HEATSET-$thread-$length', 'heat-set insert $thread x $length', "brass");
    this.thread = thread;
    this.pilotDiameter = pilotDiameter;
    this.pilotDepth = pilotDepth;
    this.length = length;
    this.boreDiameter = boreDiameter;
  }

  /** M3 x 5.7 (4.6 mm knurled body) in a 4.2 x 7.5 hole. */
  public static function m3():ThreadedInsert return new ThreadedInsert("M3", 4.2, 7.5, 5.7, 3.0);

  /** M2.5 x 4 (3.6 mm knurled body) in a 3.4 x 6 hole. */
  public static function m2_5():ThreadedInsert return new ThreadedInsert("M2.5", 3.4, 6.0, 4.0, 2.5);

  override public function hasGeometry():Bool return true;

  override public function geometry(detail:ComponentDetail = Preview):Part {
    var body = Part.cylinderSpan(pilotDiameter / 2, -length, 0.0, 0.0, 0.0);
    var bore = Part.cylinderSpan(boreDiameter / 2, -length - 1.0, 1.0, 0.0, 0.0);
    var result = body.subtract(bore);
    body.close();
    bore.close();
    return result;
  }
}
