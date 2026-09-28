package robotkit.runtime;

import haxe.Int64;
import nativekit.sim.MotionType;
import nativekit.sim.SimObject;
import nativekit.sim.SimPose;
import nativekit.sim.SimShape;

/**
 * Test/standalone convenience that owns a SimulationSpace and the Simulation
 * joined to it end to end: construction, clock control (step, start, stop,
 * reset), and simple box environment objects, mirroring the deleted
 * self-owned rk_simulation mode for callers, mainly tests, that just want one
 * thing to construct, step, and dispose. Production code should own a
 * SimulationSpace and session directly and drive them itself, as the app
 * (ApplicationSimulation, SimulationSpace) does.
 */
class SimulationHarness {
  public final space:SimulationSpace;
  public final simulation:Simulation;
  var disposed:Bool = false;

  public function new(?fixedTimestep:Float = 0.01, ?physicsSubsteps:Int = 1,
      ?backend:Int = SimulationSpace.DETERMINISTIC) {
    space = SimulationSpace.create(backend, fixedTimestep);
    try {
      simulation = Simulation.inSession(space.session);
    } catch (error:Dynamic) {
      space.dispose();
      throw error;
    }
  }

  /** Advances the session one tick and notifies the simulation's step observers. */
  public function step(?timestampNs:Int64):Void {
    space.session.step(timestampNs == null ? Int64.ofInt(0) : timestampNs);
    simulation.notifyStepped();
  }

  public function start():Void
    space.session.start();

  /** Stops realtime stepping but leaves the harness available for disposal. */
  public function stop():Void
    if (!disposed) space.session.stop();

  /** Restores every body and the fixed-step clock to the editable-scene state. */
  public function reset():Void {
    stop();
    space.session.reset();
  }

  /** Adds a box to the shared physics world and returns its owned session object. */
  public function spawnBox(position:Array<Float>, halfExtents:Array<Float>,
      ?dynamicBody:Bool = false, ?mass:Float = 1.0, ?orientation:Array<Float>):SimObject {
    if (position == null || position.length != 3 || halfExtents == null || halfExtents.length != 3)
      throw "SimulationHarness.spawnBox requires three-component position and extents";
    stop();
    return space.session.createObject(dynamicBody ? MotionType.Dynamic : MotionType.Static,
      SimShape.box(halfExtents[0], halfExtents[1], halfExtents[2]),
      makePose(position, orientation), dynamicBody ? mass : 0.0);
  }

  /** Removes an environment object from the shared scene and physics world. */
  public function removeObject(object:SimObject):Void {
    stop();
    object.dispose();
  }

  /** Teleports an environment object and clears its velocity. */
  public function teleportObject(object:SimObject, position:Array<Float>,
      ?rotation:Array<Float>):Void {
    stop();
    object.teleport(makePose(position, rotation));
  }

  /** Reads an environment body's latest physics pose. */
  public function objectPose(object:SimObject):{position:Array<Float>,rotation:Array<Float>} {
    var frame = space.session.capture();
    try {
      var pose = frame.objectPose(object);
      frame.dispose();
      return {position: [pose.x, pose.y, pose.z], rotation: [pose.qx, pose.qy, pose.qz, pose.qw]};
    } catch (error:Dynamic) {
      frame.dispose();
      throw error;
    }
  }

  public function dispose():Void {
    if (disposed) return;
    disposed = true;
    space.session.stop();
    simulation.dispose();
    space.dispose();
  }

  function makePose(position:Array<Float>, ?rotation:Array<Float>):SimPose {
    var chosen = rotation == null ? [0.0, 0.0, 0.0, 1.0] : rotation;
    if (chosen.length != 4) throw "SimulationHarness pose rotation requires four components";
    return new SimPose(position[0], position[1], position[2], chosen[0], chosen[1], chosen[2], chosen[3]);
  }
}
