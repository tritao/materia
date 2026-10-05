package robotkit.runtime;
import RobotKitRuntime;
/** Factories for core runtime endpoints. Transport adapters provide their own factories. */
class RuntimeEndpoints {
  public static function inMemory(blueprint:RobotRuntimeBlueprint):RuntimeEndpoint {
    if (blueprint == null) throw "Endpoint requires a compiled blueprint";
    var result = RobotKitRuntime.rk_robot_runtime_create(blueprint.nativeValue());
    RobotRuntime.check(result.status, "inMemoryEndpoint.create");
    return new NativeRuntimeEndpoint(result.out_runtime, new robotkit.time.SourceClock("robotkit.monotonic"));
  }
}
