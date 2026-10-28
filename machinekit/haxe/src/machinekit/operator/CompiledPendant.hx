package machinekit.operator;

import haxe.io.Bytes;

/**
 * Everything the cabinet controller needs to serve one pendant for one OperatorProfile, as wire
 * records from pendant_wire/schema/pendant.wire.idl. The host sends `profile`, then `reasons`, then
 * `labels` when a session opens, and the controller checks `digest` against its own over the same
 * bytes before it accepts the pendant.
 */
class CompiledPendant {
  /** One PendantProfile record (52 bytes). */
  public final profile:Bytes;
  /** PendantReasonText records, by reason id. */
  public final reasons:Array<Bytes>;
  /** PendantConfig records: switch positions 1 to 11, then keys F1 to F3. Unused ones clear the label. */
  public final labels:Array<Bytes>;
  /** 32-bit FNV-1a over profile, reasons and labels in that order. */
  public final digest:Int;

  public function new(profile:Bytes, reasons:Array<Bytes>, labels:Array<Bytes>) {
    this.profile = profile;
    this.reasons = reasons.copy();
    this.labels = labels.copy();
    var h = -2128831035; // 0x811C9DC5
    for (record in [profile].concat(reasons).concat(labels))
      for (i in 0...record.length) {
        h = h ^ record.get(i);
        // h * 0x01000193 spelled with shifts, so 32-bit wrap-around does not depend on Int multiplication.
        h = (h << 24) + (h << 8) + (h << 7) + (h << 4) + (h << 1) + h;
      }
    this.digest = h;
  }
}
