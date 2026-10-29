package machinekit.operator;

/**
 * What the pendant's simulated display reads, by name: the driver (`PendantDisplayDriver`) asks for these signals and a model
 * binds each to the joint that feeds it (`ReferencePendant.displayBindings`).
 */
class PendantDisplaySignals {
  /** The source name a model gives a display that the pendant firmware's core fills. */
  public static inline var SOURCE:String = "pendant-oled";

  /** The axis selector: its detent index is the pendant's position (0 is OFF). */
  public static inline var AXIS:String = "axis";
  /** The step selector: its detent index is x1, x10 or x100. */
  public static inline var STEP:String = "step";
  /** The hand wheel's rotation; 400 counts per turn. */
  public static inline var WHEEL:String = "wheel";
  public static inline var WHEEL_ENABLE:String = "wheel-enable";
  public static inline var START:String = "start";
  public static inline var HOLD:String = "hold";
  public static inline var STOP:String = "stop";
  public static inline var F1:String = "f1";
  public static inline var F2:String = "f2";
  public static inline var F3:String = "f3";

  /** The keys in the order of PENDANT_STATE's button bits. */
  public static function keys():Array<String> return [START, HOLD, STOP, F1, F2, F3];
}
