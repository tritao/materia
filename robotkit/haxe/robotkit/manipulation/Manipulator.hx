package robotkit.manipulation;

import robotkit.model.RobotModel;
import robotkit.model.FrameId;
import robotkit.spatial.Transform3;
import robotkit.world.JointTarget;

/**
 * A kinematic chain paired with its tool mount and the runtime joint
 * indices `toJointTargets` needs to drive a compiled robot. The chain's
 * tip must be a Frame (the flange); M3 adds the tool-center-point offset
 * from that flange.
 */
class Manipulator {
  public final chain:KinematicChain;
  public final group:JointGroup;
  public final flangeFrame:FrameId;

  final jointModelIndices:Array<Int>;

  public function new(model:RobotModel, chain:KinematicChain) {
    if (model == null || chain == null) throw "Manipulator requires a model and kinematic chain";
    if (chain.tipFrame == null)
      throw "Manipulator requires a kinematic chain whose tip is a mounted frame (the flange)";
    this.chain = chain;
    this.group = JointGroup.fromChain(chain);
    this.flangeFrame = chain.tipFrame;
    var indices:Array<Int> = [];
    for (id in chain.dofJointIds()) {
      var index = -1;
      for (i in 0...model.joints.length) if (model.joints[i] != null && model.joints[i].id == id) { index = i; break; }
      if (index < 0) throw 'Manipulator chain references joint "$id", which is not part of the model';
      indices.push(index);
    }
    this.jointModelIndices = indices;
  }

  public function forwardKinematics(q:Array<Float>):Transform3 return chain.forwardKinematics(q);

  /** Position targets for every chain joint, indexed by the joint's compiled RobotModel position. */
  public function toJointTargets(q:Array<Float>):Array<JointTarget> {
    if (q == null || q.length != jointModelIndices.length)
      throw 'Manipulator requires ${jointModelIndices.length} joint values, got ${q == null ? 0 : q.length}';
    return [for (i in 0...q.length) JointTarget.position(jointModelIndices[i], q[i])];
  }
}
