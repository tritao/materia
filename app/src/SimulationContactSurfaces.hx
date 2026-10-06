package app;

/** Explicit, assumed contact response for hard pneumatic machine fixtures. */
class SimulationContactSurfaces {
  /** Assumed metal contact; 1 ms is above MuJoCo's 2·dt floor at the mill's 0.25 ms substep. */
  public static inline final PNEUMATIC_FIXTURE_TIME_CONSTANT:Float = 0.001;

  public static function robotPart():robotkit.model.CollisionShape.ContactSurface
    return new robotkit.model.CollisionShape.ContactSurface(null, 0,
      PNEUMATIC_FIXTURE_TIME_CONSTANT, 1.0);

  public static function environmentPart():nativekit.sim.SimContactSurface
    return new nativekit.sim.SimContactSurface(null, 0,
      PNEUMATIC_FIXTURE_TIME_CONSTANT, 1.0);
}
