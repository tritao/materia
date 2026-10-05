package processkit;

import processkit.perception.ContactRegistrationSequence;
import robotkit.spatial.Transform3;

/** Execute all contact stages through one exclusive probe owner and expose only a fully accepted work frame. */
class ContactRegistrationRunner {
  public final probe:ContactProbeRunner;
  public final sequence:ContactRegistrationSequence;
  public var workFrame(default, null):Null<Transform3> = null;
  public var failure(default, null):Null<String> = null;
  var active = false;
  var started = false;

  public function new(probe:ContactProbeRunner, sequence:ContactRegistrationSequence) {
    if (probe == null || sequence == null) throw "Contact registration runner needs a probe owner and measured sequence";
    this.probe = probe; this.sequence = sequence;
  }
  public function running():Bool return active;
  public function completed():Bool return workFrame != null && failure == null;

  public function start():Void {
    if (started || probe.running()) throw "Contact registration runner needs an idle owner and cannot be reused";
    started = true; active = true;
    try probe.start(sequence.start()) catch (error:Dynamic) fail(Std.string(error));
    if (probe.failure != null) fail(cast probe.failure);
  }

  public function update(dt:Float):Void {
    if (!active) return;
    try {
      probe.update(dt);
      if (probe.failure != null) throw probe.failure;
      if (!probe.completed()) return;
      var next = sequence.observe(cast probe.contact);
      if (next != null) {
        probe.start(next);
        if (probe.failure != null) throw probe.failure;
      } else {
        workFrame = cast(sequence.result, processkit.perception.ContactRegistration.ContactRegistrationResult).require();
        active = false;
      }
    } catch (error:Dynamic) fail(Std.string(error));
  }
  public function cancel():Void if (active) fail("contact registration cancelled");
  function fail(reason:String):Void {
    probe.cancel(); workFrame = null; failure = reason; active = false;
  }
}
