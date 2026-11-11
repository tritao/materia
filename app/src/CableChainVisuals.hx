package app;

import NativeKitScene;

import app.AssemblyRobot.AssemblyPart;
import materia.assembly.AssemblyFrames as F;
import materia.assembly.AssemblyRecord.AssemblyFrame;
import materia.project.SceneArtifact.SceneArtifactCableChainVisual;
import materia.project.SceneArtifact.SceneArtifactChainEnd;
import machinekit.motion.CableChain;
import machinekit.motion.PackedCableChainMesh;
import machinekit.component.ComponentValues;
import robotkit.runtime.SimulationPresentationSnapshot;
import nativekit.scene.GeometryData;

private class ChainAttachment {
  public final part:AssemblyPart;
  public final connector:AssemblyFrame;
  public final frame:AssemblyFrame = F.identity();
  public var world:AssemblyFrame = F.identity();
  public function new(project:ProjectDocumentSession, robot:AssemblyRobot, end:SceneArtifactChainEnd, scale:Float) {
    part = robot.part("project:" + end.occurrence);
    var source = project.projectAssemblyDefinition;
    if (source == null) throw "Cable chain visuals need a physical assembly";
    var definition = materia.assembly.AssemblyDefinitionFlattener.flatten(source);
    var occurrence = [for (o in definition.occurrences) if (o.id == end.occurrence) o][0];
    var component = [for (c in definition.definitions) if (c.id == occurrence.definition) c][0];
    connector = F.compose(F.identity(), [for (c in component.connectors) if (c.name == end.connector) c.frame][0]);
    connector.x *= scale;
    connector.y *= scale;
    connector.z *= scale;
  }
}
private class ChainDisplay {
  public final visual:SceneArtifactCableChainVisual;
  public final part:AssemblyPart;
  public final fixed:ChainAttachment;
  public final moving:ChainAttachment;
  public final chain:CableChain;
  public final mesh = new PackedCableChainMesh();
  public final partFrame = F.identity();
  public final inverse = F.identity();
  public var last:Null<Array<Float>> = null;
  public function new(project:ProjectDocumentSession,
    robot:AssemblyRobot, v:SceneArtifactCableChainVisual, scale:Float) {
    visual = v;
    part = robot.part("project:" + v.occurrence);
    fixed = new ChainAttachment(project, robot, v.fixed, scale);
    moving = new ChainAttachment(project, robot, v.moving, scale);
    chain = new CableChain(new ComponentValues().setNumber("bendRadius", v.bendRadius).setNumber(
      "innerWidth",
      v.innerWidth
    ).setNumber(
      "innerHeight",
      v.innerHeight
    ).setNumber(
      "wall",
      v.wall
    ).setNumber(
      "travel",
      v.travel
    ).setNumber(
      "fillArea",
      0
    ));
  }
}
/** Runtime replacement of the exported static mesh, from both live end poses. */
class CableChainVisuals {
  final project:ProjectDocumentSession;
  final simulation:robotkit.runtime.Simulation;
  final scale:Float;
  final displays:Array<ChainDisplay> = [];
  public var updates(default, null) = 0;
  public function new(
    project:ProjectDocumentSession,
    robot:AssemblyRobot,
    simulation:robotkit.runtime.Simulation,
    chains:Array<SceneArtifactCableChainVisual>
  ) {
    this.project = project;
    this.simulation = simulation;
    var physical = project.projectPhysical;
    if (physical == null) throw "Cable chain visuals need physical units";
    scale = physical.metresPerUnit;
    for (v in chains) displays.push(new ChainDisplay(project, robot, v, scale));
  }
  function frame(part:AssemblyPart, poses:Null<SimulationPresentationSnapshot>, target:AssemblyFrame):AssemblyFrame {
    var link = poses == null ? null : poses.linkPose(part.robotIndex, part.linkIndex);
    if (link == null) {
      var p = simulation.linkPose(part.robotIndex, part.linkIndex);
      AssemblyRobot.composeComponentsInto(p.position, p.rotation, part.offset, target);
    } else AssemblyRobot.composeComponentsInto(link.position, link.rotation, part.offset, target);
    return target;
  }
  public function reset():Void {
    for (d in displays) {
      d.last = null;
      project.scene.clearRuntimeGeometry("project:" + d.visual.occurrence);
    }
  }
  public function present(?poses:SimulationPresentationSnapshot):Void {
    var changed:Map<String, GeometryData> = [];
    for (d in displays) {
      F.inverseInto(frame(d.part, poses, d.partFrame), d.inverse);
      for (end in [d.fixed, d.moving]) end.world = F.compose(frame(end.part, poses, end.frame), end.connector);
      var relative = F.compose(F.inverse(d.fixed.world), d.moving.world);
      var position = relative.y / scale + d.chain.n("travel") / 2;
      // Physics has metre precision; tolerate a small display-only endpoint drift.
      if (Math.abs(relative.x / scale) > 0.05 || Math.abs(relative.z / scale - 2 * d.chain.n("bendRadius"))
        > 0.05) throw "Live cable chain ends leave its supported routing plane";
      if (!Math.isFinite(position)
        || position < -0.05 || position > d.chain.n("travel") + 0.05) throw "Live cable chain end exceeds its travel";
      position = Math.max(0, Math.min(d.chain.n("travel"), position));
      var fixed = F.compose(d.inverse, d.fixed.world);
      fixed.x /= scale;
      fixed.y /= scale;
      fixed.z /= scale;
      var key = [position, fixed.x, fixed.y, fixed.z, fixed.qx, fixed.qy, fixed.qz, fixed.qw];
      var prior = d.last, same = prior != null;
      if (prior != null) for (i in 0...key.length) if (Math.abs(key[i] - prior[i]) > 1e-7) same = false;
      if (same) continue;
      d.mesh.update(d.chain, position, scale, d.part.center, fixed);
      var mesh = d.mesh, g = new GeometryData();
      g.addStream(NksceneVertexSemantic.Position, NksceneVertexFormat.Float32x3, mesh.positions, mesh.vertexCount, 12);
      g.addStream(NksceneVertexSemantic.Normal, NksceneVertexFormat.Float32x3, mesh.normals, mesh.vertexCount, 12);
      g.setIndexBuffer(mesh.indices, mesh.indexCount);
      g.setBounds(mesh.minimum[0], mesh.minimum[1], mesh.minimum[2], mesh.maximum[0], mesh.maximum[1], mesh.maximum[2]);
      changed.set("project:" + d.visual.occurrence, g);
      d.last = key;
    }
    var count = 0;
    for (_ in changed) count++;
    if (count > 0) {
      try project.scene.setRuntimeGeometries(changed) catch (e:Dynamic) {
        for (d in displays) if (changed.exists("project:" + d.visual.occurrence)) d.last = null;
        throw e;
      }
      updates += count;
    }
  }
}
