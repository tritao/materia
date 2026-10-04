package motionkit.robot;

import motionkit.axis.MotionAxisBlueprint;
import robotkit.runtime.RobotRuntimeBlueprint;

/** Expand independent task axes through the compiled physical coupling graph. */
class MotionAxisCouplings {
  public static function expand(axes:Array<MotionAxisBlueprint>, runtime:RobotRuntimeBlueprint,
      jointNames:Array<String>):Array<MotionAxisBlueprint> {
    if (axes == null || runtime == null || jointNames == null || jointNames.length != runtime.jointCount)
      throw "Coupled motion mapping requires full runtime joint names";
    var owners = [for (_ in jointNames) -1];
    var scales = [for (_ in jointNames) 0.0], offsets = [for (_ in jointNames) 0.0];
    var ids:Array<Array<String>> = [], ratios:Array<Array<Float>> = [], zeros:Array<Array<Float>> = [];
    for (i in 0...axes.length) {
      var axis = axes[i];
      if (axis == null) throw "Null logical motion axis";
      ids.push(axis.jointIds.copy()); ratios.push(axis.jointScales.copy()); zeros.push(axis.jointOffsets.copy());
      for (k in 0...axis.jointIds.length) {
        var joint = jointNames.indexOf(axis.jointIds[k]);
        if (joint < 0 || owners[joint] >= 0) throw "Motion joint mapping is missing or shared between axes";
        owners[joint] = i; scales[joint] = axis.jointScales[k]; offsets[joint] = axis.jointOffsets[k];
      }
    }
    // Resolve only after every term's leader is known. Repeated passes handle
    // authored order independently of depth; the compiled graph has no cycles.
    for (_ in 0...runtime.jointCount) {
      var changed = false;
      for (joint in 0...runtime.jointCount) {
        var hasTerms = false, ready = true, owner = -1, ratio = 0.0, zero = 0.0;
        for (term in runtime.couplings) if (term.follower == joint) {
          hasTerms = true;
          if (owners[term.leader] < 0) { ready = false; continue; }
          if (owner >= 0 && owner != owners[term.leader])
            throw "A multi-axis follower requires coordinated polynomial planning";
          owner = owners[term.leader];
          ratio += term.ratio * scales[term.leader];
          zero += term.ratio * offsets[term.leader] + term.offset;
        }
        if (!hasTerms || !ready) continue;
        if (!Math.isFinite(ratio) || ratio == 0.0 || !Math.isFinite(zero))
          throw "Coupled axis mapping has an invalid scale or zero";
        if (owners[joint] >= 0) {
          if (owners[joint] != owner || Math.abs(scales[joint] - ratio) > 1e-9 ||
              Math.abs(offsets[joint] - zero) > 1e-9)
            throw "Authored motion mapping contradicts physical couplings";
        } else {
          owners[joint] = owner; scales[joint] = ratio; offsets[joint] = zero;
          ids[owner].push(jointNames[joint]); ratios[owner].push(ratio); zeros[owner].push(zero);
          changed = true;
        }
      }
      if (!changed) break;
    }
    for (term in runtime.couplings)
      if (owners[term.leader] >= 0 && owners[term.follower] < 0)
        throw "A partially mapped follower requires coordinated polynomial planning";
    return [for (i in 0...axes.length) new MotionAxisBlueprint(axes[i].id, ids[i],
      axes[i].lowerLimit, axes[i].upperLimit, axes[i].maxVelocity, axes[i].maxAcceleration,
      axes[i].homePosition, ratios[i], zeros[i])];
  }
}
