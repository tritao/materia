package robotkit.manipulation;

import robotkit.model.RobotModel;
import robotkit.model.LinkId;
import robotkit.model.FrameId;
import robotkit.model.JointId;
import robotkit.model.Frame;

/** External joints come from the include owning the nearest physical tool flange. */
class ExternalAxes {
  public static function derive(robot:RobotModel, root:LinkId, tool:FrameId):Array<JointId> {
    var target:Null<Frame> = null;
    for (frame in robot.frames) if (frame.id == tool) target = frame;
    if (target == null) throw 'Cannot derive external axes: missing tool frame "$tool"';
    var path = KinematicGroup.walk(robot, root, target.link.id);
    var link = target.link.id;
    var owner:Null<String> = null;
    var index = path.length;
    while (true) {
      for (frame in robot.frames) if (frame.link.id == link && frame.flangeIncludePath != null) {
        if (owner != null && owner != frame.flangeIncludePath)
          throw 'Ambiguous flange ownership on link "$link"';
        owner = frame.flangeIncludePath;
      }
      if (owner != null || index == 0) break;
      index--;
      link = path[index].parent.id;
    }
    if (owner == null) return [];
    var result:Array<JointId> = [];
    for (joint in path) {
      var scope = joint.includePath;
      if (scope == null) throw 'Assembly joint "${joint.id}" has no include ownership';
      if (owner != "" && scope != owner && !StringTools.startsWith(scope, owner + "/")) result.push(joint.id);
    }
    return result;
  }
}
