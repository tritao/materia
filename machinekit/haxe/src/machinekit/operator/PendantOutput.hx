package machinekit.operator;

/** What the host may do with one accepted pendant frame. */
class PendantOutput {
  /** Distance to jog, in the axis' units, or null. */
  public final move:Null<JogMove>;
  /** Short refusal text for the OLED status line, "" when nothing is refused. */
  public final refusal:String;
  /** The pendant holds jog authority; the HMI's own jog controls are refused. */
  public final authority:Bool;
  /** Hold-to-run spindle jog: true only while the key is held and the lease is live. */
  public final spindleJog:Bool;
  /** Keys newly pressed in this frame. Fixed keys are START, HOLD, STOP bits of PendantState. */
  public final pressed:Int;
  public function new(move:Null<JogMove>, refusal:String, authority:Bool, spindleJog:Bool, pressed:Int) {
    this.move = move;
    this.refusal = refusal;
    this.authority = authority;
    this.spindleJog = spindleJog;
    this.pressed = pressed;
  }
}
