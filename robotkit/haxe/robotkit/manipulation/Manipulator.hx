package robotkit.manipulation;

import kinematicskit.KinematicModel;
import robotkit.model.FrameId;
import robotkit.model.LinkId;
import robotkit.model.RobotModel;
import robotkit.spatial.Transform3;

/**
 * An arm of a robot: the `KinematicGroup` of the joints from `baseLink` to
 * the link carrying the `flangeFrame`, with its mounted tool (`flangeTTcp`,
 * identity without one) and no work frame, so poses, Jacobians and IK
 * targets are in the base link's frame. Everything else (solving,
 * Jacobians, the swivel of a 7-DOF arm, a moving base) is the group's.
 */
class Manipulator extends KinematicGroup {
  public final baseLink:LinkId;

  public function new(robot:RobotModel, baseLink:LinkId, flangeFrame:FrameId, ?flangeTTcp:Transform3,
      ?compiled:KinematicModel, ?swivel:ArmSwivel) {
    super(robot, baseLink, flangeFrame, null, flangeTTcp, swivel, null, 0.3, compiled);
    this.baseLink = baseLink;
  }

  /** The same arm carrying a different tool; shares the compiled model. */
  public function withTool(flangeTTcp:Transform3):Manipulator
    return new Manipulator(robot, baseLink, flangeFrame, flangeTTcp, model, swivel);
}
