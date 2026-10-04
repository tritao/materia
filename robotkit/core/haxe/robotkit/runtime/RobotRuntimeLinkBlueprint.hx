package robotkit.runtime;

import RobotKitRuntime;

/** Physical properties lowered for one authored robot link. */
class RobotRuntimeLinkBlueprint {
  public final mass:Float;
  public final centerOfMass:Array<Float>;
  public final inertiaTensor:Array<Float>;

  public function new(mass:Float, centerOfMass:Array<Float>, inertiaTensor:Array<Float>) {
    this.mass = mass;
    this.centerOfMass = centerOfMass.copy();
    this.inertiaTensor = inertiaTensor.copy();
  }

  @:allow(RobotRuntimeBlueprint)
  function nativeValue():rk_robot_runtime_link {
    var value = new rk_robot_runtime_link();
    value.set_mass(mass);
    for (i in 0...3) value.set_center_of_mass(i, centerOfMass[i]);
    for (i in 0...9) value.set_inertia_tensor(i, inertiaTensor[i]);
    return value;
  }
}
