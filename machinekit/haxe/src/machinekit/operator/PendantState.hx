package machinekit.operator;

/** One decoded PENDANT_STATE frame (see PENDANT_PLAN.md). Also the heartbeat. */
class PendantState {
  public static inline final START:Int = 1;
  public static inline final HOLD:Int = 2;
  public static inline final STOP:Int = 4;
  public static inline final F1:Int = 8;
  public static inline final F2:Int = 16;
  public static inline final F3:Int = 32;
  public static inline final SEQUENCE_MODULUS:Int = 65536;

  public final sequence:Int;
  /** Signed wheel counts since the previous frame. */
  public final wheelDelta:Int;
  /** Axis switch position: 0 is OFF, 1 to 11 select a profile axis. */
  public final position:Int;
  /** Step switch: 0, 1 or 2 for x1, x10, x100. */
  public final step:Int;
  public final buttons:Int;
  public final wheelEnable:Bool;
  /** The pendant's read-back of the E-stop chain; true when the chain is closed. */
  public final chainClosed:Bool;

  public function new(sequence:Int, wheelDelta:Int, position:Int, step:Int, buttons:Int, wheelEnable:Bool, chainClosed:Bool) {
    if (sequence < 0 || sequence >= SEQUENCE_MODULUS) throw "Pendant sequence out of range";
    if (position < 0 || position > OperatorProfile.PENDANT_POSITIONS) throw "Pendant position out of range";
    if (step < 0 || step > 2) throw "Pendant step out of range";
    if (buttons < 0 || buttons > 63) throw "Pendant buttons out of range";
    this.sequence = sequence;
    this.wheelDelta = wheelDelta;
    this.position = position;
    this.step = step;
    this.buttons = buttons;
    this.wheelEnable = wheelEnable;
    this.chainClosed = chainClosed;
  }
}
