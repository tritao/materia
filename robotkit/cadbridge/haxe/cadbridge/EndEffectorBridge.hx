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
      ?state:AssemblyState, ?id:String):Tool {
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
    var bounds = envelopeBox(endEffector, state);
    return new Tool(id == null ? frameName : id, frameName, transform,
      ToolCollisionShape.Box(bounds.halfExtents, bounds.centre), properties.mass);
  }

  /** Axis-aligned bounds of all component envelopes in the robot flange frame. */
  static function envelopeBox(endEffector:EndEffector, state:AssemblyState):{centre:Vec3, halfExtents:Vec3} {
    var model = new AssemblyModel();
    endEffector.addTo(model, "");
    var mount = endEffector.mountReference();
    var mountPose = state == null ? model.pose(mount.instanceId) : state.worldPose(mount.instanceId);
    var mountWorld = AssemblyFrames.compose(mountPose,
      endEffector.memberConnectorFrame(mount.instanceId, mount.connectorName));
    var mountInverse = AssemblyFrames.inverse(mountWorld);
    var minX = Math.POSITIVE_INFINITY, minY = Math.POSITIVE_INFINITY, minZ = Math.POSITIVE_INFINITY;
    var maxX = Math.NEGATIVE_INFINITY, maxY = Math.NEGATIVE_INFINITY, maxZ = Math.NEGATIVE_INFINITY;
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
            minX = Math.min(minX, robot.x);
            minY = Math.min(minY, robot.y);
            minZ = Math.min(minZ, robot.z);
            maxX = Math.max(maxX, robot.x);
            maxY = Math.max(maxY, robot.y);
            maxZ = Math.max(maxZ, robot.z);
          }
    }
    if (!Math.isFinite(minX) || !Math.isFinite(maxX))
      throw "End effector needs envelope geometry for collision";
    return {centre: new Vec3((minX + maxX) * 0.5e-3,
      (minY + maxY) * 0.5e-3, (minZ + maxZ) * 0.5e-3),
      halfExtents: new Vec3((maxX - minX) * 0.5e-3,
        (maxY - minY) * 0.5e-3, (maxZ - minZ) * 0.5e-3)};
  }
}
