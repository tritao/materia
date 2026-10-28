package machinekit.operator;

/**
 * What the host has said it allows (OperatorState). The host sends only intent; machine facts come
 * from the controller. No fresh state means "not allowed", the same fail-safe as the pendant lease.
 * Reference model of `HostLink` in pendant_wire/src/authority.rs.
 */
class HostLink {
  public final timeoutSeconds:Float;
  var lastSequence = -1;
  var lastSeen = 0.0;
  var allowed = false;

  public function new(timeoutSeconds:Float) {
    if (!Math.isFinite(timeoutSeconds) || timeoutSeconds <= 0) throw "Timeout must be positive";
    this.timeoutSeconds = timeoutSeconds;
  }

  /** False when the state was stale or a duplicate and changed nothing. */
  public function accept(sequence:Int, jogAllowed:Bool, now:Float):Bool {
    if (lastSequence >= 0) {
      var ahead = (sequence - lastSequence + 65536) % 65536;
      if (ahead == 0 || ahead >= 32768) return false;
    }
    lastSequence = sequence;
    lastSeen = now;
    allowed = jogAllowed;
    return true;
  }

  /** The mode argument of PendantAuthority.accept is Jog only while this is true. */
  public function jogAllowed(now:Float):Bool return allowed && lastSequence >= 0 && now - lastSeen <= timeoutSeconds;
}
