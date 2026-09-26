package robotkit.manipulation;

import robotkit.model.RobotModel;
import robotkit.model.FrameId;
import robotkit.spatial.Transform3;
import robotkit.world.JointTarget;

/**
 * A kinematic chain paired with its tool mount and the runtime joint
 * indices `toJointTargets` needs to drive a compiled robot. The chain's
 * tip must be a Frame (the flange). `flangeTTcp` is the mounted tool's
 * `flange_T_tcp` offset (identity with no tool attached); `tcpPose` and
 * `solveIkForTcp` compose it with the chain's own FK/IK so callers can work
 * directly in tool-center-point coordinates.
 */
class Manipulator {
  public final chain:KinematicChain;
  public final group:JointGroup;
  public final flangeFrame:FrameId;
  public final flangeTTcp:Transform3;

  final jointModelIndices:Array<Int>;

  public function new(model:RobotModel, chain:KinematicChain, ?flangeTTcp:Transform3) {
    if (model == null || chain == null) throw "Manipulator requires a model and kinematic chain";
    if (chain.tipFrame == null)
      throw "Manipulator requires a kinematic chain whose tip is a mounted frame (the flange)";
    this.chain = chain;
    this.group = JointGroup.fromChain(chain);
    this.flangeFrame = chain.tipFrame;
    this.flangeTTcp = flangeTTcp == null ? Transform3.identity() : flangeTTcp;
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

  /** `base_T_tcp`: the chain's flange FK composed with the mounted tool's `flange_T_tcp`. */
  public function tcpPose(q:Array<Float>):Transform3 return chain.forwardKinematics(q).compose(flangeTTcp);

  /**
   * Solves IK for a target expressed at the tool center point by converting
   * it to the equivalent flange target (`target.compose(flangeTTcp.inverse())`)
   * before delegating to `InverseKinematics.solve`.
   */
  public function solveIkForTcp(target:Transform3, seed:Array<Float>, ?positionTolerance:Float = 1e-4,
      ?orientationTolerance:Float = 1e-3, ?maxIterations:Int = 100, ?damping:Float = 0.02):IKResult {
    if (target == null) throw "TCP inverse kinematics requires a target";
    var flangeTarget = target.compose(flangeTTcp.inverse());
    return InverseKinematics.solve(chain, group, flangeTarget, seed,
      positionTolerance, orientationTolerance, maxIterations, damping);
  }

  /** Position targets for every chain joint, indexed by the joint's compiled RobotModel position. */
  public function toJointTargets(q:Array<Float>):Array<JointTarget> {
    if (q == null || q.length != jointModelIndices.length)
      throw 'Manipulator requires ${jointModelIndices.length} joint values, got ${q == null ? 0 : q.length}';
    return [for (i in 0...q.length) JointTarget.position(jointModelIndices[i], q[i])];
  }
}
