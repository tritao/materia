package machinekit.operator;

/**
 * Host-side authority for one pendant on the cabinet controller. It is fed decoded frames and a
 * monotonic clock, and decides what is allowed. Not a safety function: the E-stop chain is hard
 * wired, and `chainClosed` is only an observation that makes the pendant refuse to jog.
 *
 * - Frames older than or equal to the last accepted sequence are dropped (16-bit wrap aware).
 * - No frame within `timeoutSeconds` ends the lease. It returns only after a frame arrives with
 *   the wheel released, so a held button cannot resume motion by itself.
 * - Wheel counts count only while the enable is held in this frame and the previous one, and only
 *   in Jog mode, so enabling cannot produce a jump from counts accumulated earlier.
 * - A lost frame loses its wheel counts, which under-travels and never over-travels.
 */
class PendantAuthority {
  final profile:OperatorProfile;
  final stepSizes:Array<Float>;
  public final timeoutSeconds:Float;
  var lastSequence = -1;
  var lastSeen = 0.0;
  var alive = false;
  var needsRelease = false;
  var previous:Null<PendantState> = null;

  public function new(profile:OperatorProfile, stepSizes:Array<Float>, timeoutSeconds:Float) {
    if (profile == null) throw "Pendant authority needs a profile";
    if (stepSizes == null || stepSizes.length != 3) throw "Three step sizes are required";
    for (s in stepSizes) if (!Math.isFinite(s) || s <= 0) throw "Step sizes must be positive";
    if (!Math.isFinite(timeoutSeconds) || timeoutSeconds <= 0) throw "Timeout must be positive";
    this.profile = profile;
    this.stepSizes = stepSizes.copy();
    this.timeoutSeconds = timeoutSeconds;
  }

  /** True while a live pendant has its axis switch away from OFF. The HMI must not jog then. */
  public function holdsAuthority(now:Float):Bool
    return alive && now - lastSeen <= timeoutSeconds && previous != null && previous.position != 0;

  /** Call periodically even without frames; ends the lease when the heartbeat stops. */
  public function tick(now:Float):Void {
    if (alive && now - lastSeen > timeoutSeconds) {
      alive = false;
      needsRelease = true;
      previous = null;
    }
  }

  public function accept(state:PendantState, now:Float, mode:OperatorMode, active:Array<String>):Null<PendantOutput> {
    tick(now);
    if (lastSequence >= 0) {
      var ahead = (state.sequence - lastSequence + PendantState.SEQUENCE_MODULUS) % PendantState.SEQUENCE_MODULUS;
      if (ahead == 0 || ahead >= PendantState.SEQUENCE_MODULUS / 2) return null; // stale or duplicate
    }
    lastSequence = state.sequence;
    lastSeen = now;
    var before = previous;
    previous = state;
    if (needsRelease) {
      if (state.wheelEnable || (state.buttons & PendantState.F2) != 0) return new PendantOutput(null, "Release keys", false, false, 0);
      needsRelease = false;
    }
    alive = true;
    var pressed = before == null ? 0 : state.buttons & ~before.buttons;
    var authority = state.position != 0;
    if (!state.chainClosed) return new PendantOutput(null, "E-stop open", authority, false, pressed);
    var refusal = "";
    var move:Null<JogMove> = null;
    if (state.position != 0) {
      if (mode != Jog || !profile.modes.contains(Jog)) refusal = "Not in jog mode";
      else if (profile.pendantLabel(state.position) == "") refusal = "No axis here";
      else {
        var axisId = profile.pendantAxes[state.position - 1];
        var rule = profile.jogRefusal(axisId, active);
        if (rule != null) refusal = rule.reason;
        else if (state.wheelEnable && before != null && before.wheelEnable && state.wheelDelta != 0)
          move = new JogMove(axisId, state.wheelDelta * stepSizes[state.step]);
      }
    }
    var spindle = profile.hasSpindle && authority && mode == Jog && refusal == "" &&
      (state.buttons & (PendantState.F1 << functionKey(SpindleJog))) != 0;
    return new PendantOutput(move, refusal, authority, spindle, pressed);
  }

  /** Index 0 to 2 of the profile key with this action, or a value whose bit never matches. */
  function functionKey(action:KeyAction):Int {
    for (i in 0...profile.keys.length) if (profile.keys[i].action == action) return i;
    return 8;
  }
}
