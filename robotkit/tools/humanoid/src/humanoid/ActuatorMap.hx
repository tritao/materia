package humanoid;

import robotkit.model.RobotModel;
import robotkit.model.Transmission;

/** Resolves each actuator's joint index, gear and default servo gains. */
class ActuatorMap {
  public final joints:Array<Int> = [];
  public final gears:Array<Float> = [];

  public function new(model:RobotModel) {
    for (actuator in model.actuators) switch actuator.transmission {
      case SimpleTransmission(jointId, ratio, _):
        var index = -1;
        for (joint in 0...model.joints.length) if (model.joints[joint].id == jointId) index = joint;
        if (index < 0) throw 'actuator ${actuator.id} drives an unknown joint';
        joints.push(index);
        gears.push(ratio);
    }
  }
}
