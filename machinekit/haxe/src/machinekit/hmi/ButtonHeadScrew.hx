package machinekit.hmi;

import cadkit.modeling.Part;
import machinekit.component.ComponentDetail;
import machinekit.component.MachineComponent;

/**
 * An ISO 7380 button head socket screw, A2 stainless. Frame: the head's bearing face at z = 0, the head above it and the
 * shank below. Threads are semantic: the shank is the nominal diameter, which is also the bore of its insert.
 */
class ButtonHeadScrew extends MachineComponent {
  public final thread:String;
  public final diameter:Float;
  public final length:Float;
  public final headDiameter:Float;
  public final headHeight:Float;

  public function new(thread:String, diameter:Float, length:Float, headDiameter:Float, headHeight:Float) {
    super('ISO7380-$thread-$length', 'button head screw $thread x $length', "stainless A2-70");
    this.thread = thread;
    this.diameter = diameter;
    this.length = length;
    this.headDiameter = headDiameter;
    this.headHeight = headHeight;
  }

  /** M3 x `length`: 5.7 mm head, 1.65 high. */
  public static function m3(length:Float):ButtonHeadScrew return new ButtonHeadScrew("M3", 3.0, length, 5.7, 1.65);

  /** M2.5 x `length`: 4.7 mm head, 1.4 high. */
  public static function m2_5(length:Float):ButtonHeadScrew return new ButtonHeadScrew("M2.5", 2.5, length, 4.7, 1.4);

  override public function hasGeometry():Bool return true;

  override public function geometry(detail:ComponentDetail = Preview):Part {
    var head = Part.cylinderSpan(headDiameter / 2, 0.0, headHeight, 0.0, 0.0);
    var shank = Part.cylinderSpan(diameter / 2, -length, 0.0, 0.0, 0.0);
    var result = head.combine(shank);
    head.close();
    shank.close();
    return result;
  }
}
