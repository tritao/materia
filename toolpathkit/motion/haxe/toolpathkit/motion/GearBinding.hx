package toolpathkit.motion;

import robotkit.model.RobotModel;
import robotkit.execution.GearEvents;
import robotkit.execution.ProcessChannelDeclaration;

/** Resolve names and encoder counts from the model. Controller timing is deliberately explicit:
    the driver position loop is a ceiling, not evidence of the owner's actual scheduling frequency.
*/
class GearBinding {
  public static function install(
    binding:MachineBinding,
    model:RobotModel,
    controllerHz:Float,
    encoderDelayNs:Map<String, Int>,
    followers:Array<String>
  ):Array<ProcessChannelDeclaration> {
    if (!Math.isFinite(controllerHz) || controllerHz <= 0) throw "Gear controller needs a stated owner loop rate";
    if (followers == null || followers.length == 0) throw "Gear binding needs explicit follower joints";
    binding.gearFollowers = followers.copy();
    binding.gearJointIds = [for (joint in model.joints) joint.id];
    binding.gearEncoders = new Map();
    var channels:Array<ProcessChannelDeclaration> = [];
    // A master is a measured joint, not necessarily an actuator: a cartridge encoder sees belt stretch independently
    // of the motor's feedback. The explicit controller rate schedules these measurements. Aliases of the same
    // joint encoder must agree on quantization; conflicting sensors require an explicit selection in the model.
    for (encoder in model.encoders) {
      if (followers.indexOf(encoder.joint) >= 0) continue;
      var delay = encoderDelayNs.get(encoder.joint);
      if (delay == null) continue;
      if (delay < 0 || delay > 1000000000) throw 'Invalid encoder latency for ${encoder.joint}';
      var previous = binding.gearEncoders.get(encoder.joint);
      if (previous != null && previous.quantum != encoder.resolution())
        throw 'Gear master ${encoder.joint} has conflicting encoder resolutions';
      binding.gearEncoders.set(encoder.joint,
        {latencyNs: delay, quantum: encoder.resolution(), loopHz: controllerHz});
    }
    for (i in 0...model.joints.length) {
      var joint = model.joints[i];
      if (followers.indexOf(joint.id) < 0) continue;
      for (actuator in model.actuators) {
        var id = switch actuator.transmission {
          case SimpleTransmission(id, _, _):
            id;
        };
        if (id != joint.id || actuator.positionLoopRate <= 0) continue;
        var encoder = model.encoderFor(actuator), delay = encoderDelayNs.get(joint.id);
        if (encoder == null || encoder.joint != joint.id || delay == null) continue;
        if (delay < 0 || delay > 1000000000) throw 'Invalid encoder latency for ${joint.id}';
        binding.gearEncoders.set(
          joint.id,
          {latencyNs: delay, quantum: encoder.resolution(), loopHz: Math.min(controllerHz, actuator.positionLoopRate)}
        );
        if (actuator.processVelocity == null) channels.push(GearEvents.declaration(i));
      }
    }
    for (follower in followers) {
      if (!binding.gearEncoders.exists(follower)) throw 'Gear follower $follower needs an installed encoder and timing';
      for (coupling in model.couplings) if (coupling.leader == follower
        || coupling.follower == follower) throw 'Gear follower $follower must be independent of mechanical couplings';
    }
    if (channels.length
      != followers.length) throw "Gear followers must be distinct, independently planned servo joints";
    return channels;
  }
}
