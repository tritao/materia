package app;

/** Application facade for the shared machine-fixture contact assumptions. */
class SimulationContactSurfaces {
  public static inline final PNEUMATIC_FIXTURE_TIME_CONSTANT:Float = robotkit.runtime.SimulatedContactSurfaces.HARD_FIXTURE_TIME_CONSTANT;
  public static function robotPart():robotkit.model.CollisionShape.ContactSurface
    return robotkit.runtime.SimulatedContactSurfaces.robotPart();
  public static function gripperFinger(frictionCoefficient:Float):robotkit.model.CollisionShape.ContactSurface
    return robotkit.runtime.SimulatedContactSurfaces.gripperFinger(frictionCoefficient);
  public static function environmentPart(filter:nativekit.sim.SimContactFilter=nativekit.sim.SimContactFilter.Layers):nativekit.sim.SimContactSurface
    return robotkit.runtime.SimulatedContactSurfaces.environmentPart(filter);
}
