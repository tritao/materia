package robotkit.runtime;

/** Shared, explicit contact assumptions for hard machine fixtures and replaceable grip pads. */
class SimulatedContactSurfaces {
  /** Matches MuJoCo's 2*dt floor at a 0.25 ms drive substep. This is a simulation assumption. */
  public static inline final HARD_FIXTURE_TIME_CONSTANT:Float=0.0005;
  public static function robotPart():robotkit.model.CollisionShape.ContactSurface
    return new robotkit.model.CollisionShape.ContactSurface(null,0,HARD_FIXTURE_TIME_CONSTANT,1);
  public static function gripperFinger(frictionCoefficient:Float):robotkit.model.CollisionShape.ContactSurface {
    if(!(frictionCoefficient>0) || !Math.isFinite(frictionCoefficient))throw "Gripper pad friction must be finite and positive";
    return new robotkit.model.CollisionShape.ContactSurface([frictionCoefficient,0,0],3,HARD_FIXTURE_TIME_CONSTANT,1);
  }
  public static function environmentPart(filter:nativekit.sim.SimContactFilter=nativekit.sim.SimContactFilter.Layers):nativekit.sim.SimContactSurface
    return new nativekit.sim.SimContactSurface(null,0,HARD_FIXTURE_TIME_CONSTANT,1,filter);
}
