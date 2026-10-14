package motionkit.robot;

import motionkit.axis.MotionAxisBlueprint;
import robotkit.runtime.RobotRuntimeBlueprint;

/** Expand independent task axes through the compiled physical coupling graph. */
class MotionAxisCouplings {
  /**
   * The joints that follow more than one axis and so belong to none, as one projection (each after the joints it follows,
   * the axes' joints first). `axes` are the expanded axes. A joint that follows joints no axis drives (an extruder's gear
   * train) stays where it is and is not in it, unless it mixes those with driven joints, which needs coordinated planning.
   * A cycle among the couplings is an error.
   */
  public static function derived(axes:Array<MotionAxisBlueprint>, runtime:RobotRuntimeBlueprint,
      jointNames:Array<String>):CouplingProjection {
    var owned = [for (_ in jointNames) false];
    for (axis in axes) for (id in axis.jointIds) owned[jointNames.indexOf(id)] = true;
    var termsOf = [for (joint in 0...runtime.jointCount) [for (term in runtime.couplings) if (term.follower == joint)
      {leader: term.leader, follower: joint, ratio: term.ratio, offset: term.offset}]];
    var included = owned.copy();
    var remaining = [for (joint in 0...runtime.jointCount) if (!owned[joint] && termsOf[joint].length > 0) joint];
    var progress = true;
    while (remaining.length > 0 && progress) {
      progress = false;
      for (joint in remaining.copy()) {
        var ready = true;
        for (term in termsOf[joint]) if (!included[term.leader]) ready = false;
        if (!ready) continue;
        included[joint] = true;
        remaining.remove(joint);
        progress = true;
      }
    }
    for (joint in remaining)
      for (term in termsOf[joint]) if (included[term.leader]) throw "A partially mapped follower requires coordinated polynomial planning";
    // The joints left follow only joints nothing drives: they stay put, but a cycle among them is still an error.
    CouplingProjection.of([for (joint in remaining) for (term in termsOf[joint]) term], runtime.jointCount);
    return CouplingProjection.of([for (joint in 0...runtime.jointCount) if (!owned[joint] && included[joint])
      for (term in termsOf[joint]) term], runtime.jointCount);
  }

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
        var hasTerms = false, ready = true, multiple = false, owner = -1, ratio = 0.0, zero = 0.0;
        for (term in runtime.couplings) if (term.follower == joint) {
          hasTerms = true;
          if (owners[term.leader] < 0) { ready = false; continue; }
          // A follower of several axes belongs to none: see `derived`.
          if (owner >= 0 && owner != owners[term.leader]) { multiple = true; continue; }
          owner = owners[term.leader];
          ratio += term.ratio * scales[term.leader];
          zero += term.ratio * offsets[term.leader] + term.offset;
        }
        if (!hasTerms || !ready || multiple) continue;
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
    return [for (i in 0...axes.length) new MotionAxisBlueprint(axes[i].id, ids[i],
      axes[i].lowerLimit, axes[i].upperLimit, axes[i].maxVelocity, axes[i].maxAcceleration,
      axes[i].homePosition, ratios[i], zeros[i])];
  }
}
