package robotkit.runtime;

import RobotKitSimKit;
import haxe.Int64;

/**
 * Owns one shared simulated universe and its fixed-step clock.
 *
 * The object creates RobotRuntime handles but remains their simulation owner:
 * callers should submit through those handles and advance this object once per
 * tick. It is intentionally separate from SimulatedRobot, which is only a
 * live Robot adapter for RobotWorld.
 */
class Simulation {
  final owner:Ownedrk_simulation;
  final robots:Array<RobotRuntime> = [];
  var disposed:Bool = false;

  public function new(? fixedTimestep:Float = 0.01, ? physicsSubsteps:Int = 1) {
    var desc = new rk_simulation_desc();
    desc.set_struct_size(rk_simulation_desc.size());
    desc.set_fixed_timestep(fixedTimestep);
    desc.set_physics_substeps(physicsSubsteps);
    var result = RobotKitSimKit.rk_simulation_create(desc);
    check(result.status, "simulation.create");
    owner = result.out_simulation;
  }

  /** Adds topology before the first start or step. */
  public function addRobot(blueprint:RobotRuntimeBlueprint):RobotRuntime {
    ensureLive();
    var result = RobotKitSimKit.rk_simulation_add_robot(owner.borrow(), blueprint.nativeValue());
    check(result.status, "simulation.addRobot");
    var runtime = new RobotRuntime(result.out_runtime);
    robots.push(runtime);
    return runtime;
  }

  /** Applies every attached runtime command and advances the world once. */
  public function step(timestampNs:Int64):Void {
    ensureLive();
    check(RobotKitSimKit.rk_simulation_step(owner.borrow(), timestampNs), "simulation.step");
  }

  /** Starts the shared realtime clock after topology construction is complete. */
  public function start():Void {
    ensureLive();
    check(RobotKitSimKit.rk_simulation_start(owner.borrow()), "simulation.start");
  }

  /** Stops realtime stepping but leaves the simulation available for disposal. */
  public function stop():Void {
    if (!disposed) check(RobotKitSimKit.rk_simulation_stop(owner.borrow()), "simulation.stop");
  }

  public function stepIndex():Int64 {
    var value = readClock();
    return value.get_step_index();
  }

  public function simulationTime():Float {
    var value = readClock();
    return value.get_simulation_time();
  }

  public function dispose():Void {
    if (disposed) return;
    stop();
    for (runtime in robots) runtime.dispose();
    robots.resize(0);
    owner.close();
    disposed = true;
  }

  function readClock():rk_simulation_clock {
    ensureLive();
    var value = new rk_simulation_clock();
    value.set_struct_size(rk_simulation_clock.size());
    var result = RobotKitSimKit.rk_simulation_get_clock(owner.borrow(), value);
    check(result.status, "simulation.getClock");
    return value;
  }

  function ensureLive():Void {
    if (disposed) throw "RobotKit simulation has been disposed";
  }

  static function check(status:Int, operation:String):Void {
    if (status != RobotKitRuntimeConstants.RK_OK) throw '$operation failed with RobotKit status $status';
  }
}
