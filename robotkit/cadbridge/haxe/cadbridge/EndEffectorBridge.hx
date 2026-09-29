package cadbridge;

import cadkit.modeling.AssemblyState;
import machinekit.robotics.EndEffector;
import machinekit.robotics.EndEffector.EndEffectorSolvedContext;
import machinekit.robotics.EndEffectorFrames;
import machinekit.units.Millimetres;
import machinekit.units.KgMm2;
import machinekit.component.ComponentDetail;
import materia.assembly.AssemblyFrames;
import robotkit.spatial.Quat;
import robotkit.spatial.Inertia3;
import robotkit.spatial.Transform3;
import robotkit.spatial.Vec3;
import robotkit.tool.Tool;
import robotkit.tool.MassProperties;
import robotkit.tool.ToolCollisionShape;
import cadbridge.EndEffectorCollision.EndEffectorCollisionOptions;

/** Convert one end-effector working frame into a RobotKit tool. */
class EndEffectorBridge {
  public static function toTool(endEffector:EndEffector, frameName:String,
      ?state:AssemblyState, ?id:String, ?collisionOptions:EndEffectorCollisionOptions,
      boxCollision:Bool = false):Tool {
    if (endEffector == null) throw "End effector is required";
    endEffector.validate();
    var solved = endEffector.solve(state);
    var converted = EndEffectorFrames.toRobotFrame(new machinekit.robotics.ConnectorFrame(endEffector.mountTFrame(frameName, state, solved)));
    var transform = new Transform3(new Vec3(converted.position.x,
      converted.position.y, converted.position.z), new Quat(converted.quaternion.x,
      converted.quaternion.y, converted.quaternion.z, converted.quaternion.w));
    var properties = endEffector.massPropertiesAtMount(state, solved);
    if (properties.unaccounted.length != 0)
      throw 'End effector has unaccounted BOM mass: ${properties.unaccounted.join(", ")}';
    if (!Math.isFinite(properties.mass) || properties.mass <= 0)
      throw "End effector requires positive mass";
    var centre = EndEffectorFrames.pointYToZ(properties.centreOfMass.x,
      properties.centreOfMass.y, properties.centreOfMass.z);
    var tensor = properties.inertia;
    var inertia = tensor == null ? null : new Inertia3((new KgMm2(tensor.xx)).kgM2(),
      (new KgMm2(tensor.xy)).kgM2(), (new KgMm2(tensor.xz)).kgM2(), (new KgMm2(tensor.yy)).kgM2(),
      (new KgMm2(tensor.yz)).kgM2(), (new KgMm2(tensor.zz)).kgM2())
      .rotated(new Quat(Math.sqrt(0.5), 0, 0, Math.sqrt(0.5)));
    var mass = new MassProperties(properties.mass,
      new Vec3((new Millimetres(centre.x)).metres(), (new Millimetres(centre.y)).metres(), (new Millimetres(centre.z)).metres()), inertia);
    var collision:ToolCollisionShape;
    if (boxCollision) {
      var bounds = envelopeBox(endEffector, solved);
      collision = ToolCollisionShape.Box(bounds.halfExtents, bounds.centre);
    } else {
      var result = EndEffectorCollision.pieces(endEffector, state, collisionOptions, solved);
      collision = ToolCollisionShape.Hulls([for (piece in result.pieces) piece.vertices], result.padding);
    }
    return new Tool(id == null ? frameName : id, frameName, transform,
      collision, properties.mass, mass);
  }

  /** Axis-aligned bounds of all component envelopes in the robot flange frame. */
  static function envelopeBox(endEffector:EndEffector,
      solved:EndEffectorSolvedContext):{centre:Vec3, halfExtents:Vec3} {
    var mountInverse = AssemblyFrames.inverse(solved.mountWorld);
    var minX = Math.POSITIVE_INFINITY, minY = Math.POSITIVE_INFINITY, minZ = Math.POSITIVE_INFINITY;
    var maxX = Math.NEGATIVE_INFINITY, maxY = Math.NEGATIVE_INFINITY, maxZ = Math.NEGATIVE_INFINITY;
    for (member in endEffector.components()) {
      if (endEffector.collisionExcluded(member.id)) continue;
      var part = member.component.geometry(Envelope);
      var bounds:CadKit.Bounds;
      try bounds = part.shape.bounds() catch (error:Dynamic) {
        part.close();
        throw error;
      }
      part.close();
      var min = bounds.get_min(), max = bounds.get_max();
      var pose = solved.poses.get(member.id);
      if (pose == null) throw 'Missing solved pose for "${member.id}"';
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
    return {centre: new Vec3((new Millimetres((minX + maxX) * 0.5)).metres(),
      (new Millimetres((minY + maxY) * 0.5)).metres(), (new Millimetres((minZ + maxZ) * 0.5)).metres()),
      halfExtents: new Vec3((new Millimetres((maxX - minX) * 0.5)).metres(),
        (new Millimetres((maxY - minY) * 0.5)).metres(), (new Millimetres((maxZ - minZ) * 0.5)).metres())};
  }
}
