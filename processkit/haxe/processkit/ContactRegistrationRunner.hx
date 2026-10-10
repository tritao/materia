package processkit;

import processkit.perception.ContactRegistrationSequence;
import processkit.ContactProbeRunner.ContactProbeRequest;
import robotkit.spatial.Transform3;

/** Execute all contact stages through one exclusive probe owner and expose only a fully accepted work frame. */
class ContactRegistrationRunner {
  public final probe:ContactProbeRunner;
  public final sequence:ContactRegistrationSequence;
  public var workFrame(default, null):Null<Transform3> = null;
  public var failure(default, null):Null<String> = null;
  var active = false;
  var started = false;
  var sourceClock:String = "";
  var startJoints:Null<Array<Float>> = null;
  var acceptedFrame:Null<Transform3> = null;
  var returning = false;
  var betweenStages = false;
  var returnTarget:Null<Array<Float>> = null;
  var pendingProbe:Null<ContactProbeRequest> = null;

  public function new(probe:ContactProbeRunner, sequence:ContactRegistrationSequence) {
    if (probe == null || sequence == null) throw "Contact registration runner needs a probe owner and measured sequence";
    this.probe = probe; this.sequence = sequence;
  }
  public function running():Bool return active;
  public function completed():Bool return workFrame != null && failure == null;

  public function start():Void {
    if (started || probe.running()) throw "Contact registration runner needs an idle owner and cannot be reused";
    started = true; active = true;
    try {
      var snapshot = probe.motion.robot.snapshot();
      sourceClock = snapshot.sourceClockId;
      startJoints = [for (index in probe.motion.jointIndices) snapshot.positions.get(index)];
      beginStage(sequence.start());
    } catch (error:Dynamic) fail(Std.string(error));
    if (probe.failure != null) fail(cast probe.failure);
  }

  public function update(dt:Float):Void {
    if (!active) return;
    try {
      if (probe.motion.robot.snapshot().sourceClockId != sourceClock)
        throw "Contact registration joint clock changed epoch";
      if (returning) {
        probe.motion.update(dt);
        if (probe.motion.failure != null) throw 'Contact registration return failed: ${probe.motion.failure}';
        if (!probe.motion.completed) return;
        if (returnTarget == null || returnTarget.length != probe.motion.jointIndices.length)
          throw "Contact registration return has no valid checked target";
        var snapshot = probe.motion.robot.snapshot();
        var target:Array<Float> = cast returnTarget;
        for (joint in 0...target.length) if (Math.abs(snapshot.positions.get(probe.motion.jointIndices[joint]) - target[joint]) > 0.005)
          throw 'Contact registration return missed joint ${probe.motion.jointIndices[joint]}';
        if (betweenStages) {
          betweenStages = false;
          returning = false;
          returnTarget = null;
          var request = pendingProbe;
          if (request == null) throw "Contact registration entry lost its prepared probe";
          pendingProbe = null;
          probe.start(request);
          if (probe.failure != null) throw probe.failure;
          return;
        }
        workFrame = acceptedFrame;
        acceptedFrame = null;
        returning = false;
        returnTarget = null;
        active = false;
        return;
      }
      probe.update(dt);
      if (probe.failure != null) throw probe.failure;
      if (!probe.completed()) return;
      var next = sequence.observe(cast probe.contact);
      if (next != null) {
        probe.start(next);
        if (probe.failure != null) throw probe.failure;
      } else if (sequence.stageTransitionPending) {
        beginStage(sequence.selectNextStage());
      } else {
        acceptedFrame = cast(sequence.result, processkit.perception.ContactRegistration.ContactRegistrationResult).require();
        var current = [for (index in probe.motion.jointIndices) probe.motion.robot.snapshot().positions.get(index)];
        if (startJoints == null) throw "Contact registration lost its saved starting posture";
        var target:Array<Float> = cast startJoints;
        var moved = false;
        for (joint in 0...target.length) if (Math.abs(current[joint] - target[joint]) > 1e-12) moved = true;
        if (!moved) {
          workFrame = acceptedFrame;
          acceptedFrame = null;
          active = false;
        } else {
          // A contact sequence is a measurement operation. Restore its measured
          // start through the same collision-aware planner before publishing the
          // frame, so later authored welding motions begin at their planned pose.
          var restore = probe.planner.approachJoints(target, current);
          probe.motion.reset();
          probe.motion.run(restore.program);
          returnTarget = target.copy();
          returning = true;
        }
      }
    } catch (error:Dynamic) fail(Std.string(error));
  }
  function beginStage(request:ContactProbeRequest):Void {
    var stage = sequence.activeStage();
    if (stage == null || startJoints == null) throw "Contact registration stage has no checked entry";
    var entry = stage.entryJoints;
    var target:Array<Float> = entry == null ? startJoints.copy() : entry;
    var snapshot = probe.motion.robot.snapshot();
    var current = [for (index in probe.motion.jointIndices) snapshot.positions.get(index)];
    if (target.length != current.length) throw "Contact registration entry has the wrong joint count";
    var moved = false;
    for (joint in 0...target.length) if (Math.abs(current[joint] - target[joint]) > 1e-12) moved = true;
    if (!moved) {
      probe.start(request);
      if (probe.failure != null) throw probe.failure;
      return;
    }
    var approach = probe.planner.approachJoints(target, current);
    probe.motion.reset();
    probe.motion.run(approach.program);
    pendingProbe = request;
    returnTarget = target.copy();
    returning = true;
    betweenStages = true;
  }
  public function cancel():Void if (active) fail("contact registration cancelled");
  function fail(reason:String):Void {
    probe.cancel(); workFrame = null; failure = reason; active = false;
    if (returning) {
      try probe.motion.robot.stop(robotkit.core.StopMode.Normal) catch (_:Dynamic) {}
      try probe.motion.abort() catch (_:Dynamic) {}
    }
    acceptedFrame = null; returning = false; betweenStages = false; returnTarget = null; pendingProbe = null;
  }
}
