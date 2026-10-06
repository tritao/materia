package app;

/** Explicit, assumed contact response for hard pneumatic machine fixtures. */
class SimulationContactSurfaces {
  /** Assumed hard fixture contact; matches MuJoCo's 2·dt floor at the 0.25 ms drive substep. */
  public static inline final PNEUMATIC_FIXTURE_TIME_CONSTANT:Float = 0.0005;

  public static function robotPart():robotkit.model.CollisionShape.ContactSurface
    return new robotkit.model.CollisionShape.ContactSurface(null, 0,
      PNEUMATIC_FIXTURE_TIME_CONSTANT, 1.0);

  /** Contact parameters carried by the physical replaceable grip pads. */
  public static function gripperFinger(frictionCoefficient:Float):robotkit.model.CollisionShape.ContactSurface {
    if (!(frictionCoefficient > 0.0) || !Math.isFinite(frictionCoefficient))
      throw "Gripper pad friction must be finite and positive";
    return new robotkit.model.CollisionShape.ContactSurface([frictionCoefficient, 0.0, 0.0], 3,
      PNEUMATIC_FIXTURE_TIME_CONSTANT, 1.0);
  }

  public static function environmentPart():nativekit.sim.SimContactSurface
    return new nativekit.sim.SimContactSurface(null, 0,
      PNEUMATIC_FIXTURE_TIME_CONSTANT, 1.0);
}
