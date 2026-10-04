package robotkit.runtime;
import RobotKitRuntime;
/** Factories for simulation-owned cyclic and virtual scheduled endpoints.
    The shared Simulation constructs and drives their native owners. */
class SimulationEndpoints {
  public static function simulation(owner:Ownedrk_robot_runtime,
      contacts:Void -> Array<RobotContact>):RuntimeEndpoint
    return new NativeRuntimeEndpoint(owner, contacts);
  public static function virtualDevice(owner:Ownedrk_robot_runtime,
      contacts:Void -> Array<RobotContact>):RuntimeEndpoint
    return new NativeRuntimeEndpoint(owner, contacts);
}
