package robotkit.safety;

/** User-facing safety state source; native RobotRuntime enforces hard limits and stops. */
interface Safety {
  function state():SafetyState;
}
