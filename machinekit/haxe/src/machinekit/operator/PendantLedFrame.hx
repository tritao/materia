package machinekit.operator;

/** PENDANT_OUTPUT: LEDs, lease state and the one-line status text. */
class PendantLedFrame {
  public static inline final TEXT_BYTES:Int = 21;
  /** One bit per button, in PendantState bit order. */
  public final leds:Int;
  public final lease:LeaseState;
  public final text:String;
  public function new(leds:Int, lease:LeaseState, text:String) {
    if (leds < 0 || leds > 63) throw "LED bits out of range";
    if (text != "") OperatorProfile.requireLabel(text, TEXT_BYTES, "status text");
    this.leds = leds;
    this.lease = lease;
    this.text = text;
  }
}
