package tests;

import haxe.Int64;
import nativekit.scene.Scene;
import nativekit.sim.MotionType;
import nativekit.sim.SimPose;
import nativekit.sim.SimSession;
import nativekit.sim.SimShape;
import nativekit.sim.SimWorld;
import robotkit.runtime.RobotRuntimeBlueprint;
import robotkit.runtime.Simulation;
import robotkit.runtime.SimulationPresentationSnapshot;
import robotkit.runtime.SimulationStepObserver;

private class TickRecorder implements SimulationStepObserver {
  public final times:Array<Int64> = [];
  public function new() {}
  public function afterSimulationStep(sourceTimestampNs:Int64):Void
    times.push(sourceTimestampNs);
}

/** Robots joining a session another owner steps, alongside its props and people. */
class SharedSessionTests {
  public static function run():Void {
    var scene = Scene.create();
    var world = new SimWorld(scene, {timestep: 0.01});
    var session = new SimSession(scene, world.nativeHandle());
    var simulation = Simulation.inSession(session);
    var runtime = simulation.addRobotAtPose(new RobotRuntimeBlueprint(1, 0, 1), [0.0, 0.0, 0.0],
      [0.0, 0.0, 0.0, 1.0]);
    var ticks = new TickRecorder();
    simulation.addStepObserver(ticks);
    var crate = session.createObject(MotionType.Dynamic, SimShape.box(0.2, 0.2, 0.2),
      new SimPose(3.0, 0.0, 2.0), 1.0);
    var person = session.createActor([SimShape.capsule(0.2, 1.2)], [new SimPose(-2.0, 0.0, 0.8)]);
    person.pushKeyframe(0.0, [new SimPose(-2.0, 0.0, 0.8)]);
    person.pushKeyframe(1.0, [new SimPose(-1.0, 0.0, 0.8)]);

    // A joined session's clock belongs to its owner: Simulation has no
    // step/start/stop/reset of its own to misuse; only the session does.
    for (_ in 0...50) session.step();
    if (ticks.times.length != 50 || ticks.times[0] != Int64.ofInt(10000000) ||
        ticks.times[49] != Int64.ofInt(500000000) ||
        simulation.sourceTimestampNs() != ticks.times[49])
      throw "Joined robot did not observe every shared session step";
    if (Int64.toInt(runtime.snapshot().sequence) < 1)
      throw "robots did not publish from the shared session's ticks";

    var frame = session.capture();
    var presentation = simulation.presentFrame(frame);
    if (presentation.poses.length != 2 || Int64.toInt(presentation.stepIndex) != 50)
      throw 'robots were not presented from the shared frame: ${presentation.poses.length}';
    var base = presentation.poses[0];
    if (base.kind != SimulationPresentationSnapshot.ROBOT_BASE || Math.abs(base.position[0]) > 1e-9)
      throw "robot base pose missing from the shared frame";
    if (Math.abs(frame.actorPose(person, 0).x + 1.5) > 1e-6)
      throw "person did not follow its keyframes on the shared clock";
    if (frame.objectPose(crate).z >= 2.0)
      throw "the shared session's crate did not fall";
    presentation.dispose();
    frame.dispose();

    session.stop();
    simulation.dispose();
    session.step();
    if (ticks.times.length != 50)
      throw "Disposed simulation still observed shared session steps";
    session.dispose();
    world.dispose();
    scene.dispose();
  }
}
