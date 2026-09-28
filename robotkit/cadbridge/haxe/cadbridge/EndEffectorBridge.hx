package cadbridge;

import cadkit.modeling.AssemblyModel;
import cadkit.modeling.AssemblyState;
import machinekit.robotics.EndEffector;
import machinekit.robotics.EndEffectorFrames;
import machinekit.component.ComponentDetail;
import materia.assembly.AssemblyFrames;
import robotkit.spatial.Quat;
import robotkit.spatial.Transform3;
import robotkit.spatial.Vec3;
import robotkit.tool.Tool;
import robotkit.tool.ToolCollisionShape;

/** Convert one end-effector working frame into a RobotKit tool. */
class EndEffectorBridge {
  public static function toTool(endEffector:EndEffector, frameName:String,
      ?state:AssemblyState):Tool {
    if (endEffector == null) throw "End effector is required";
    endEffector.validate();
    var converted = EndEffectorFrames.toRobotFrame(endEffector.mountTFrame(frameName, state));
    var transform = new Transform3(new Vec3(converted.position.x,
      converted.position.y, converted.position.z), new Quat(converted.quaternion.x,
      converted.quaternion.y, converted.quaternion.z, converted.quaternion.w));
    var properties = endEffector.massPropertiesAtMount(state);
    if (properties.unaccounted.length != 0)
      throw 'End effector has unaccounted BOM mass: ${properties.unaccounted.join(", ")}';
    if (!Math.isFinite(properties.mass) || properties.mass <= 0)
      throw "End effector requires positive mass";
    return new Tool(frameName, frameName, transform,
      ToolCollisionShape.Box(envelopeHalfExtents(endEffector, state)), properties.mass);
  }

  /** Conservative flange-centred box: Box has no offset field in RobotKit. */
  static function envelopeHalfExtents(endEffector:EndEffector, state:AssemblyState):Vec3 {
    var model = new AssemblyModel();
    endEffector.addTo(model, "");
    var mount = endEffector.mountReference();
    var mountPose = state == null ? model.pose(mount.instanceId) : state.worldPose(mount.instanceId);
    var mountWorld = AssemblyFrames.compose(mountPose,
      endEffector.memberConnectorFrame(mount.instanceId, mount.connectorName));
    var mountInverse = AssemblyFrames.inverse(mountWorld);
    var halfX = 0.0, halfY = 0.0, halfZ = 0.0;
    for (member in endEffector.components()) {
      var part = member.component.geometry(Envelope);
      var bounds:CadKit.Bounds;
      try bounds = part.shape.bounds() catch (error:Dynamic) {
        part.close();
        throw error;
      }
      part.close();
      var min = bounds.get_min(), max = bounds.get_max();
      var pose = state == null ? model.pose(member.id) : state.worldPose(member.id);
      var relative = AssemblyFrames.compose(mountInverse, pose);
      for (x in [min.get_x(), max.get_x()])
        for (y in [min.get_y(), max.get_y()])
          for (z in [min.get_z(), max.get_z()]) {
            var point = AssemblyFrames.transformPoint(relative, x, y, z);
            var robot = EndEffectorFrames.pointYToZ(point.x, point.y, point.z);
            halfX = Math.max(halfX, Math.abs(robot.x));
            halfY = Math.max(halfY, Math.abs(robot.y));
            halfZ = Math.max(halfZ, Math.abs(robot.z));
          }
    }
    return new Vec3(halfX * 1e-3, halfY * 1e-3, halfZ * 1e-3);
  }
}
