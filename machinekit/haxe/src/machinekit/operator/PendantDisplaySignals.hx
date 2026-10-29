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
  /** The E-stop's head: pushed in, the chain is open. The pendant only reads it back; the chain itself is hard wired. */
  public static inline var ESTOP:String = "estop";
  public static inline var START:String = "start";
  public static inline var HOLD:String = "hold";
  public static inline var STOP:String = "stop";
  public static inline var F1:String = "f1";
  public static inline var F2:String = "f2";
  public static inline var F3:String = "f3";

  /** The source name of a key's LED; its output is the key's own signal name (`keys()`). */
  public static inline var LED_SOURCE:String = "pendant-led";

  /** The colour a key's LED lights in, as red, green and blue out of 255: the colour of its cap. */
  public static function ledColour(key:String):Array<Int> {
    if (key == START) return [60, 230, 90];
    if (key == HOLD) return [250, 210, 40];
    if (key == STOP) return [255, 60, 50];
    return [235, 240, 255];
  }

  /** The keys in the order of PENDANT_STATE's button bits. */
  public static function keys():Array<String> return [START, HOLD, STOP, F1, F2, F3];
}
