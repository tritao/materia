package app;

import app.AssemblyRobot.AssemblyPart;
import materia.assembly.AssemblyFrames;
import materia.project.SceneArtifact.SceneArtifactBeltVisual;
import machinekit.transmission.TimingBelt;
import machinekit.transmission.TimingBelt.BeltWrap;
import machinekit.transmission.TimingBeltMesh;
import nativekit.scene.GeometryData;
import haxe.io.Bytes;

/** Rebuilds display belts from physical attachment poses only when a presented pose changes. */
class BeltVisuals {
  final project:ProjectDocumentSession;
  final robot:AssemblyRobot;
  final belts:Array<SceneArtifactBeltVisual>;
  final simulation:robotkit.runtime.Simulation;
  final definition:materia.assembly.AssemblyDefinition;
  final scale:Float;
  var previous:Array<Float> = [];
  public var updates(default, null):Int = 0;

  public function new(project:ProjectDocumentSession, robot:AssemblyRobot, simulation:robotkit.runtime.Simulation, belts:Array<SceneArtifactBeltVisual>) {
    this.project = project; this.robot = robot; this.belts = belts.copy();
    var definition = project.projectAssemblyDefinition, physical = project.projectPhysical;
    if (definition == null || physical == null) throw "Belt visuals need a physical assembly";
    this.definition = materia.assembly.AssemblyDefinitionFlattener.flatten(definition);
    this.simulation = simulation;
    scale = physical.metresPerUnit;
  }

  public function reset():Void {
    previous = [];
    for (belt in belts) project.scene.clearRuntimeGeometry("project:" + belt.occurrence);
  }

  public function present():Void {
    var observation = robot.robot.snapshot();
    var values = observation.positions.toArray();
    var changed = previous.length != values.length;
    if (!changed) for (i in 0...values.length) if (Math.abs(values[i] - previous[i]) > 1e-8) changed = true;
    if (!changed) return;
    function frame(part:AssemblyPart):materia.assembly.AssemblyRecord.AssemblyFrame {
      var pose = AssemblyRobot.partPose(simulation, part);
      return {x: pose.position[0], y: pose.position[1], z: pose.position[2],
        qx: pose.rotation[0], qy: pose.rotation[1], qz: pose.rotation[2], qw: pose.rotation[3]};
    }
    for (visual in belts) {
      var inverse = AssemblyFrames.inverse(frame(robot.part("project:" + visual.occurrence)));
      var wraps = [for (wrap in visual.wraps) {
        var occurrence = [for (value in definition.occurrences) if (value.id == wrap.occurrence) value][0];
        var component = [for (value in definition.definitions) if (value.id == occurrence.definition) value][0];
        var connector = [for (value in component.connectors) if (value.name == wrap.connector) value][0];
        var world = AssemblyFrames.transformPoint(frame(robot.part("project:" + wrap.occurrence)),
          connector.frame.x * scale, connector.frame.y * scale, connector.frame.z * scale);
        var local = AssemblyFrames.transformPoint(inverse, world.x, world.y, world.z);
        new BeltWrap(local.x / scale, local.y / scale, wrap.radius, wrap.side);
      }];
      var belt = new TimingBelt(machinekit.transmission.TimingBeltProfile.Custom("CUSTOM", visual.pitch, 0.0), visual.width, wraps);
      var side = visual.wraps[visual.driverWrap].side;
      var driver = [for (i in 0...robot.model.joints.length) if (robot.model.joints[i].id == visual.driverJoint) i][0];
      var phase = values[driver] * visual.wraps[visual.driverWrap].radius * side * visual.driverSign;
      var mesh = TimingBeltMesh.build(belt, phase, side, visual.thickness);
      var part = robot.part("project:" + visual.occurrence), center = part.center;
      var positions = Bytes.alloc(mesh.positions.length * 4), normals = Bytes.alloc(mesh.normals.length * 4);
      var indices = Bytes.alloc(mesh.indices.length * 4);
      var low = [Math.POSITIVE_INFINITY, Math.POSITIVE_INFINITY, Math.POSITIVE_INFINITY];
      var high = [Math.NEGATIVE_INFINITY, Math.NEGATIVE_INFINITY, Math.NEGATIVE_INFINITY];
      for (i in 0...mesh.positions.length) {
        var axis = i % 3, value = mesh.positions[i] * scale - center[axis];
        positions.setFloat(i * 4, value); normals.setFloat(i * 4, mesh.normals[i]);
        low[axis] = Math.min(low[axis], value); high[axis] = Math.max(high[axis], value);
      }
      for (i in 0...mesh.indices.length) indices.setInt32(i * 4, mesh.indices[i]);
      var geometry = new GeometryData();
      geometry.addStream(1, 2, positions, Std.int(mesh.positions.length / 3), 12);
      geometry.addStream(2, 2, normals, Std.int(mesh.normals.length / 3), 12);
      geometry.setIndexBuffer(indices, mesh.indices.length);
      geometry.setBounds(low[0], low[1], low[2], high[0], high[1], high[2]);
      project.scene.setRuntimeGeometry("project:" + visual.occurrence, geometry);
      updates++;
    }
    previous = values;
  }
}
