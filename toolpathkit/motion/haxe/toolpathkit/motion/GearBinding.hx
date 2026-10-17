package toolpathkit.motion;

import robotkit.model.RobotModel;
import robotkit.execution.GearEvents;
import robotkit.execution.ProcessChannelDeclaration;

/** Resolve names and encoder counts from the model. Controller timing is deliberately explicit:
    the driver position loop is a ceiling, not evidence of the owner's actual scheduling frequency.
*/
class GearBinding {
  public static function install(binding:MachineBinding, model:RobotModel, controllerHz:Float,
      encoderDelayNs:Map<String, Int>, followers:Array<String>):Array<ProcessChannelDeclaration> {
    if (!Math.isFinite(controllerHz) || controllerHz <= 0) throw "Gear controller needs a stated owner loop rate";
    if (followers == null || followers.length == 0) throw "Gear binding needs explicit follower joints";
    binding.gearFollowers = followers.copy();
    binding.gearJointIds = [for (joint in model.joints) joint.id];
    binding.gearEncoders = new Map();
    var channels:Array<ProcessChannelDeclaration> = [];
    for (i in 0...model.joints.length) {
      var joint = model.joints[i];
      for (actuator in model.actuators) {
        var id = switch actuator.transmission { case SimpleTransmission(id, _, _): id; };
        if (id != joint.id || actuator.positionLoopRate <= 0) continue;
        var encoder = model.encoderFor(actuator), delay = encoderDelayNs.get(joint.id);
        if (encoder == null || delay == null) continue;
        if (delay < 0 || delay > 1000000000) throw 'Invalid encoder latency for ${joint.id}';
        binding.gearEncoders.set(joint.id, {latencyNs:delay, quantum:encoder.resolution(),
          loopHz:Math.min(controllerHz, actuator.positionLoopRate)});
        if (followers.indexOf(joint.id) >= 0 && actuator.processVelocity == null) channels.push(GearEvents.declaration(i));
      }
    }
    for (follower in followers) {
      if (!binding.gearEncoders.exists(follower)) throw 'Gear follower $follower needs an installed encoder and timing';
      for (coupling in model.couplings) if (coupling.leader == follower || coupling.follower == follower)
        throw 'Gear follower $follower must be independent of mechanical couplings';
    }
    if (channels.length != followers.length) throw "Gear followers must be distinct, independently planned servo joints";
    return channels;
  }
}
