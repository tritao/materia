package app;

import app.AssemblyRobot.AssemblyPart;
import app.SimulatedTools.GripObject;
import materia.assembly.AssemblyDefinitionFlattener;
import materia.assembly.AssemblyFrames;
import nativekit.sim.SimObject;
import nativekit.sim.SimPose;
import robotkit.runtime.Simulation;

/** A CAD occurrence's live frame and collision pieces, whether carried by a robot or a scene object. */
class SimulationAssemblyPart implements robotkit.runtime.SimulationObjectFixture {
  public final id:String;
  /** Preview geometry centre in the occurrence's own frame, in metres. */
  public final center:Array<Float>;
  /** Convex pieces in that same CAD frame, in metres (`AssemblySimulationBridge.collisionPieces`). Contact policy is separate from geometry. */
  public final pieces:Array<Array<Float>>;
  final simulation:Simulation;
  final part:Null<AssemblyPart>;
  final object:Null<SimObject>;

  public function new(id:String, simulation:Simulation, center:Array<Float>, pieces:Array<Array<Float>>,
      part:Null<AssemblyPart>, object:Null<SimObject>) {
    if ((part == null) == (object == null)) throw "A simulated assembly part needs exactly one carrier";
    this.id = id;
    this.simulation = simulation;
    this.center = center.copy();
    this.pieces = [for (piece in pieces) piece.copy()];
    this.part = part;
    this.object = object;
  }

  public function pose():{position:Array<Float>, rotation:Array<Float>} {
    if (part != null) return AssemblyRobot.partPose(simulation, part);
    var body = object;
    if (body == null) throw 'Assembly occurrence "$id" lost its free-object carrier';
    var live = simulation.objectPose(body);
    return AssemblyRobot.compose({position: [live.x, live.y, live.z], rotation: [live.qx, live.qy, live.qz, live.qw]},
      AssemblyFrames.translation(-center[0], -center[1], -center[2]));
  }

  /** Whether this exact free body overlaps an oriented volume, using SimKit's shape-level test. */
  public function overlapsVolume(pose:SimPose, halfExtents:Array<Float>):Bool {
    var body = object;
    return body != null && body.overlapsVolume(pose, halfExtents);
  }

  /** Changes the live free body's inertial mass after machining changes its material volume. */
  public function setMass(mass:Float):Void {
    var body = object;
    if (body == null) throw 'Assembly occurrence "$id" is carried by the robot and has no free mass';
    body.setMass(mass);
  }

  public var freeObject(get, never):Bool;
  function get_freeObject():Bool return object != null;

  /** The independently simulated body of this exact screw occurrence. */
  public function freeBody():SimObject {
    var body = object;
    if (body == null) throw 'Assembly occurrence "$id" has no independent screw body';
    return body;
  }

  /** Fasten at the actual seated pose, preserving the member's live carrier and all offsets. */
  public function retain(body:SimObject, world:SimPose):Void {
    var member = part, carrier = object;
    var parent = member != null ? simulation.linkPose(member.robotIndex,member.linkIndex) : null;
    if (carrier != null) {
      var live = simulation.objectPose(carrier);
      parent = {position:[live.x,live.y,live.z],rotation:[live.qx,live.qy,live.qz,live.qw]};
    }
    if (parent == null) throw 'Assembly occurrence "$id" lost its fastening carrier';
    var p = parent.position, q = parent.rotation;
    var relative = AssemblyFrames.compose(AssemblyFrames.inverse({x:p[0],y:p[1],z:p[2],qx:q[0],qy:q[1],qz:q[2],qw:q[3]}),
      {x:world.x,y:world.y,z:world.z,qx:world.qx,qy:world.qy,qz:world.qz,qw:world.qw});
    var pose = new SimPose(relative.x,relative.y,relative.z,relative.qx,relative.qy,relative.qz,relative.qw);
    if (member != null) simulation.holdObjectOnLink(body,member.robotIndex,member.linkIndex,pose);
    else if (carrier != null) simulation.holdObjectOnObject(body,carrier,pose);
  }

  /** Whether this occurrence's exact native carrier retains the fastened body. */
  public function carries(body:SimObject):Bool {
    var member = part, carrier = object;
    if (member != null) return simulation.objectCarrier(body) == simulation.linkBody(member.robotIndex,member.linkIndex);
    return carrier != null && simulation.objectCarrier(body) == simulation.objectBody(carrier);
  }

  /** Excludes only this free workpiece from one cutter link's rigid-body contacts. */
  public function excludeContactWith(robotIndex:Int, linkIndex:Int):Void {
    var body = object;
    if (body != null) simulation.excludeContactPair(robotIndex, linkIndex, body);
  }
}

/** Resolves stable CAD occurrence ids without confusing robot membership with physical ownership. */
class SimulationAssemblyParts {
  final simulation:Simulation;
  final robots:Array<AssemblyRobot>;
  final objects:Array<GripObject>;
  final project:ProjectDocumentSession;
  final definitions:Map<String, String> = new Map();
  final components:Map<String, materia.assembly.AssemblyDefinition.AssemblyComponentDefinition> = new Map();
  final cache:Map<String, SimulationAssemblyPart> = new Map();

  public function new(simulation:Simulation, robots:Array<AssemblyRobot>, objects:Array<GripObject>, project:ProjectDocumentSession) {
    this.simulation = simulation;
    this.robots = robots.copy();
    this.objects = objects;
    this.project = project;
    var assembly = project.projectAssemblyDefinition;
    if (assembly == null) throw "Simulated assembly parts require an assembly";
    var flat = AssemblyDefinitionFlattener.flatten(assembly);
    for (occurrence in flat.occurrences) definitions.set("project:" + occurrence.id, occurrence.definition);
    for (component in flat.definitions) components.set(component.id, component);
  }

  public function get(id:String):SimulationAssemblyPart {
    var cached = cache.get(id);
    if (cached != null) return cached;
    var definition = definitions.get(id);
    var physical = project.projectPhysical;
    if (definition == null || physical == null) throw 'No CAD physical data for "$id"';
    var centre = project.assemblyPreviewCenter(definition);
    if (centre == null) throw 'No CAD preview centre for "$id"';
    var record = project.scene.object(id);
    if (record == null) throw 'No scene occurrence "$id"';
    var found = [for (body in physical.parts) if (body.id == definition) body];
    var component = components.get(definition);
    if (found.length != 1 || component == null) throw 'No CAD physical part for "$id"';
    var source = cadbridge.AssemblySimulationBridge.collisionPieces(id, component, found[0]);
    if (source.length == 0) throw 'No CAD collision hull for "$id"';
    var pieces = [for (piece in source) [for (value in piece) value * physical.metresPerUnit]];
    var parts = [for (robot in robots) for (part in robot.parts) if (part.id == id) part];
    var bodies = [for (entry in objects) if (entry.id == id) entry.object];
    if (parts.length + bodies.length != 1) throw 'No unique simulation carrier for "$id"';
    var result = new SimulationAssemblyPart(id, simulation, [for (value in centre) value * physical.metresPerUnit],
      pieces, parts.length == 1 ? parts[0] : null, bodies.length == 1 ? bodies[0] : null);
    cache.set(id, result);
    return result;
  }
}
