package app;

import app.AssemblyRobot.AssemblyPart;
import materia.assembly.AssemblyFrames;
import materia.assembly.AssemblyRecord.AssemblyFrame;
import materia.project.SceneArtifact.SceneArtifactBeltVisual;
import machinekit.transmission.TimingBelt;
import machinekit.transmission.TimingBelt.BeltWrap;
import machinekit.transmission.PackedTimingBeltMesh;
import haxe.io.Bytes;
import robotkit.runtime.SimulationPresentationSnapshot;
import nativekit.scene.GeometryData;

private typedef BeltAttachment = {
  var part:AssemblyPart;
  var x:Float;
  var y:Float;
  var z:Float;
  var frame:AssemblyFrame;
  var world:{x:Float, y:Float, z:Float};
  var local:{x:Float, y:Float, z:Float};
  var currentX:Float;
  var currentY:Float;
}

private class BeltDisplay {
  public final visual:SceneArtifactBeltVisual;
  public final part:AssemblyPart;
  public final attachments:Array<BeltAttachment>;
  public final driver:Int;
  public final mesh = new PackedTimingBeltMesh();
  public final partFrame:AssemblyFrame = emptyFrame();
  public final inverseFrame:AssemblyFrame = emptyFrame();
  public static function emptyFrame():AssemblyFrame return {x:0, y:0, z:0, qx:0, qy:0, qz:0, qw:1};
  public var belt:Null<TimingBelt> = null;
  public var route:Array<BeltWrap> = [];
  public var phase:Float = Math.NaN;
  public function new(visual:SceneArtifactBeltVisual, part:AssemblyPart,
      attachments:Array<BeltAttachment>, driver:Int) {
    this.visual = visual; this.part = part; this.attachments = attachments; this.driver = driver;
  }
}

/** Updates each display belt from its own physical attachments and driver, using reusable mesh storage. */
class BeltVisuals {
  final project:ProjectDocumentSession;
  final robot:AssemblyRobot;
  final displays:Array<BeltDisplay> = [];
  final simulation:robotkit.runtime.Simulation;
  final driverPositions:Array<Float>;
  final scale:Float;
  // Display-only tolerance in millimetres, well below a pixel; physics retains its full precision.
  static inline var DISPLAY_TOLERANCE:Float = 0.01;
  public var updates(default, null):Int = 0;

  public function new(project:ProjectDocumentSession, robot:AssemblyRobot, simulation:robotkit.runtime.Simulation, belts:Array<SceneArtifactBeltVisual>) {
    this.project = project; this.robot = robot; this.simulation = simulation;
    driverPositions = [for (_ in 0...robot.model.joints.length) 0.0];
    var definition = project.projectAssemblyDefinition, physical = project.projectPhysical;
    if (definition == null || physical == null) throw "Belt visuals need a physical assembly";
    definition = materia.assembly.AssemblyDefinitionFlattener.flatten(definition);
    scale = physical.metresPerUnit;
    for (visual in belts) {
      var attachments:Array<BeltAttachment> = [];
      for (wrap in visual.wraps) {
        var occurrence = [for (value in definition.occurrences) if (value.id == wrap.occurrence) value][0];
        var component = [for (value in definition.definitions) if (value.id == occurrence.definition) value][0];
        var connector = [for (value in component.connectors) if (value.name == wrap.connector) value][0];
        attachments.push({part: robot.part("project:" + wrap.occurrence),
          x: connector.frame.x * scale, y: connector.frame.y * scale, z: connector.frame.z * scale,
          frame:BeltDisplay.emptyFrame(), world:{x:0, y:0, z:0}, local:{x:0, y:0, z:0}, currentX:0, currentY:0});
      }
      var driver = [for (i in 0...robot.model.joints.length) if (robot.model.joints[i].id == visual.driverJoint) i][0];
      displays.push(new BeltDisplay(visual, robot.part("project:" + visual.occurrence), attachments, driver));
    }
  }

  public function reset():Void {
    for (display in displays) {
      display.belt = null; display.phase = Math.NaN;
      project.scene.clearRuntimeGeometry("project:" + display.visual.occurrence);
    }
  }

  function frame(part:AssemblyPart, poses:Null<SimulationPresentationSnapshot>, target:AssemblyFrame):AssemblyFrame {
    var link = poses == null ? null : poses.linkPose(part.robotIndex, part.linkIndex);
    if (link == null) {
      var pose = simulation.linkPose(part.robotIndex, part.linkIndex);
      AssemblyRobot.composeComponentsInto(pose.position, pose.rotation, part.offset, target);
    } else {
      AssemblyRobot.composeComponentsInto(link.position, link.rotation, part.offset, target);
    }
    return target;
  }

  public function present(?poses:SimulationPresentationSnapshot):Void {
    // Belt phase needs only joint positions. A full robot snapshot would also copy sensors just
    // before the presentation path captures those same sensor streams for the world view.
    robot.runtime.copyJointPositionsInto(driverPositions);
    var changed:Map<String, GeometryData> = new Map();
    var count = 0;
    for (display in displays) {
      var visual = display.visual;
      var partPose = frame(display.part, poses, display.partFrame);
      AssemblyFrames.inverseInto(partPose, display.inverseFrame);
      var routeChanged = display.belt == null;
      for (i in 0...display.attachments.length) {
        var attachment = display.attachments[i], wrap = visual.wraps[i];
        AssemblyFrames.transformPointInto(frame(attachment.part, poses, attachment.frame),
          attachment.x, attachment.y, attachment.z, attachment.world);
        AssemblyFrames.transformPointInto(display.inverseFrame, attachment.world.x, attachment.world.y,
          attachment.world.z, attachment.local);
        var x = attachment.currentX = attachment.local.x / scale;
        var y = attachment.currentY = attachment.local.y / scale;
        if (display.belt != null && (Math.abs(x - display.route[i].x) > DISPLAY_TOLERANCE ||
            Math.abs(y - display.route[i].y) > DISPLAY_TOLERANCE)) routeChanged = true;
      }
      var side = visual.wraps[visual.driverWrap].side;
      var phase = driverPositions[display.driver] * visual.wraps[visual.driverWrap].radius * side * visual.driverSign;
      phase -= visual.pitch * Math.floor(phase / visual.pitch);
      var phaseDelta = Math.abs(phase - display.phase);
      if (!routeChanged && Math.min(phaseDelta, visual.pitch - phaseDelta) <= DISPLAY_TOLERANCE) continue;
      if (routeChanged) {
        var wraps:Array<BeltWrap> = [];
        for (i in 0...display.attachments.length) {
          var attachment = display.attachments[i], wrap = visual.wraps[i];
          wraps.push(new BeltWrap(attachment.currentX, attachment.currentY, wrap.radius, wrap.side));
        }
        display.belt = new TimingBelt(
          machinekit.transmission.TimingBeltProfile.Custom("CUSTOM", visual.pitch, 0.0), visual.width, wraps);
        display.route = wraps;
      }
      var mesh = display.mesh;
      mesh.update(display.belt, phase, side, visual.thickness, scale, display.part.center);
      var geometry = new GeometryData();
      geometry.addStream(1, 2, mesh.positions, mesh.vertexCount, 12);
      geometry.addStream(2, 2, mesh.normals, mesh.vertexCount, 12);
      geometry.setIndexBuffer(mesh.indices, mesh.indexCount);
      geometry.setBounds(mesh.minimum[0], mesh.minimum[1], mesh.minimum[2],
        mesh.maximum[0], mesh.maximum[1], mesh.maximum[2]);
      changed.set("project:" + visual.occurrence, geometry);
      display.phase = phase;
      count++;
    }
    if (count > 0) {
      try project.scene.setRuntimeGeometries(changed) catch (error:Dynamic) {
        // A failed publication must be retried, even if the physical pose stays unchanged.
        for (display in displays) if (changed.exists("project:" + display.visual.occurrence)) {
          display.belt = null; display.phase = Math.NaN;
        }
        throw error;
      }
      updates += count;
    }
  }
}
