package robotkit.device;

import robotkit.model.Actuator;
import robotkit.model.RobotModel;

/** One actuator coordinate as a function of a single independent joint. */
typedef DeviceTransmissionMapping = {
  var jointIndex:Int;
  var ratio:Float;
  var offset:Float;
  /** False when a shaft follows a sum of independent coordinates. */
  var singleLeader:Bool;
}

/** Resolve complete one-leader transmission chains for RKD's physical channel grouping. */
class DeviceTransmission {
  public static function of(model:RobotModel, actuator:Actuator):DeviceTransmissionMapping {
    var target:String;
    var ratio:Float;
    var offset:Float;
    switch actuator.transmission {
      case SimpleTransmission(joint, value, zero): target = joint; ratio = value; offset = zero;
    }
    function indexOf(id:String):Int {
      for (index in 0...model.joints.length) if (model.joints[index].id == id) return index;
      throw 'Device actuator "${actuator.id}" follows missing joint "$id"';
    }
    var original:DeviceTransmissionMapping = {jointIndex: indexOf(target), ratio: ratio, offset: offset, singleLeader: false};
    var visited = new Map<String, Bool>();
    while (true) {
      if (visited.exists(target)) throw 'Device transmission cycle at "$target"';
      visited.set(target, true);
      var terms = [for (coupling in model.couplings) if (coupling.follower == target) coupling];
      if (terms.length == 0) return {jointIndex: indexOf(target), ratio: ratio, offset: offset, singleLeader: true};
      // A sum cannot be represented by RKD6's one-coordinate channel. Preserve the
      // original shaft coordinate; the native compiler evaluates its coupling sum.
      if (terms.length != 1) return original;
      var term = terms[0];
      offset += ratio * term.offset;
      ratio *= term.ratio;
      if (!Math.isFinite(ratio) || ratio == 0 || !Math.isFinite(offset))
        throw 'Device transmission for "${actuator.id}" must be finite and nonzero';
      target = term.leader;
    }
    return original;
  }
}
