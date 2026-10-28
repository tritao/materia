package machinekit.operator;

import haxe.io.Bytes;
import machinekit.operator.wire.OperatorState;
import machinekit.operator.wire.PendantEvent;
import machinekit.operator.wire.PendantRefusal;

/**
 * The host's side of the pendant: turns the controller's `PendantEvent`s into jog distances, tracks
 * who holds jog authority, and builds the `OperatorState` frames that tell the controller what the
 * host allows. The host decides nothing about authority: the controller does, and an event says so.
 *
 * Distances arrive as integers in thousandths of the axis' user unit, so x1, x10 and x100 are
 * 0.001, 0.01 and 0.1 per wheel count, whether the axis is linear (mm) or rotary (degrees).
 */
class PendantJogDriver {
  public static inline final UNIT:Float = 0.001;
  static inline final NO_AXIS:Int = 255;

  final profile:OperatorProfile;
  final reasonTexts:Array<String>;
  final sink:JogSink;
  final authorityTimeoutSeconds:Float;
  var lastSequence = -1;
  var lastSeen = 0.0;
  var holds = false;
  var stateSequence = 0;

  /** Text for the HMI status line from the latest event, "" when nothing was refused. */
  public var refusal(default, null):String = "";
  /** Called with the buttons newly pressed in an event (PendantState bit order), when any. */
  public var onKeys:Null<(pressed:Int) -> Void> = null;

  /**
   * `reasonTexts` are the compiled profile's refusal texts by reason id (`CompiledPendant.reasonTexts`).
   * With no event for `authorityTimeoutSeconds` the pendant no longer holds authority, as on the
   * controller, where the lease has lapsed.
   */
  public function new(
    profile:OperatorProfile,
    reasonTexts:Array<String>,
    sink:JogSink,
    authorityTimeoutSeconds:Float = 0.25
  ) {
    if (profile == null
      || reasonTexts == null || sink == null) throw "A pendant jog driver needs a profile, texts and a sink";
    if (!Math.isFinite(authorityTimeoutSeconds) || authorityTimeoutSeconds <= 0) throw "Timeout must be positive";
    this.profile = profile;
    this.reasonTexts = reasonTexts.copy();
    this.sink = sink;
    this.authorityTimeoutSeconds = authorityTimeoutSeconds;
  }

  /** False for a stale or duplicate event, which changes nothing. A malformed one throws. */
  public function accept(event:PendantEvent, now:Float):Bool {
    if (lastSequence >= 0) {
      var ahead = (event.sequence - lastSequence + 65536) % 65536;
      if (ahead == 0 || ahead >= 32768) return false;
    }
    var axisId:Null<String> = null;
    if (event.axis != NO_AXIS) {
      if (event.axis >= profile.axes.length) throw "PendantEvent names an axis the profile does not have";
      axisId = profile.axes[event.axis].id;
    }
    if (event.distance != 0 && (axisId == null
      || event.refusal != PendantRefusal.none)) throw "PendantEvent carries a move without an axis, or with a refusal";
    lastSequence = event.sequence;
    lastSeen = now;
    holds = (event.flags & 1) != 0;
    refusal = refusalText(event);
    if (axisId != null && event.distance != 0) sink.jog(axisId, event.distance * UNIT);
    if (event.pressed != 0 && onKeys != null) onKeys(event.pressed);
    return true;
  }

  public function pendantHoldsAuthority(now:Float):Bool return holds && now - lastSeen <= authorityTimeoutSeconds;

  /** The HMI's own jog controls are refused while the pendant holds authority. */
  public function hmiMayJog(now:Float):Bool return !pendantHoldsAuthority(now);

  /**
   * An `OperatorState` frame for the controller; send one about every 100 ms. `jogAllowed` is the
   * host's intent only (jog mode, nothing faulted): machine facts such as a running program are
   * the controller's to evaluate.
   */
  public function operatorStateFrame(jogAllowed:Bool):Bytes {
    var state = new OperatorState();
    stateSequence = (stateSequence + 1) % 65536;
    state.sequence = stateSequence;
    state.jog_allowed = jogAllowed ? 1 : 0;
    return new OperatorFrame(OperatorFrame.STATE, state.encode()).encode();
  }

  function refusalText(event:PendantEvent):String {
    var kind = event.refusal;
    if (kind == PendantRefusal.none) return "";
    if (kind == PendantRefusal.notJogMode) return "Not in jog mode";
    if (kind == PendantRefusal.noAxis) return "No axis here";
    if (kind == PendantRefusal.chainOpen) return "E-stop open";
    if (kind == PendantRefusal.busy) return "Program running";
    if (kind == PendantRefusal.releaseKeys) return "Release keys";
    if (kind == PendantRefusal.rule) {
      if (event.reason >= reasonTexts.length) throw "PendantEvent names a refusal text the profile does not have";
      return reasonTexts[event.reason];
    }
    throw "Unknown pendant refusal";
  }
}
