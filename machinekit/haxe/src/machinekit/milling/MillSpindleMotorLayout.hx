package machinekit.milling;

/** Spindle-drive datums the head and its carrier share, in the cartridge gauge frame: the belt adjustment travel and the
 * carrier's anchor bolts on the head top. */
class MillSpindleMotorLayout {
  public static inline var ADJUSTMENT:Float = 5;
  public static inline var FOOT_BOLT_X:Float = 68;
  public static inline var FOOT_BOLT_FRONT:Float = 110;
  public static inline var FOOT_BOLT_REAR:Float = 130;
}
