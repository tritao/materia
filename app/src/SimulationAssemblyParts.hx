package app;

import app.AssemblyRobot.AssemblyPart;
import app.SimulatedTools.GripObject;
import materia.assembly.AssemblyDefinitionFlattener;
import materia.assembly.AssemblyFrames;
import nativekit.sim.SimObject;
import robotkit.runtime.Simulation;

/** A CAD occurrence's live frame and collision hull, whether carried by a robot or a scene object. */
class SimulationAssemblyPart {
  public final id:String;
  /** Preview geometry centre in the occurrence's own frame, in metres. */
  public final center:Array<Float>;
  /** Convex vertices in that same CAD frame, in metres; empty when collisions are disabled. */
  public final vertices:Array<Float>;
  final simulation:Simulation;
  final part:Null<AssemblyPart>;
  final object:Null<SimObject>;

  public function new(id:String, simulation:Simulation, center:Array<Float>, vertices:Array<Float>,
      part:Null<AssemblyPart>, object:Null<SimObject>) {
    if ((part == null) == (object == null)) throw "A simulated assembly part needs exactly one carrier";
    this.id = id;
    this.simulation = simulation;
    this.center = center.copy();
    this.vertices = vertices.copy();
    this.part = part;
    this.object = object;
  }

  public function pose():{position:Array<Float>, rotation:Array<Float>} {
    if (part != null) return AssemblyRobot.partPose(simulation, part);
    var live = simulation.objectPose(cast object);
    return AssemblyRobot.compose({position: [live.x, live.y, live.z], rotation: [live.qx, live.qy, live.qz, live.qw]},
      AssemblyFrames.translation(-center[0], -center[1], -center[2]));
  }
}

/** Resolves stable CAD occurrence ids without confusing robot membership with physical ownership. */
class SimulationAssemblyParts {
  final simulation:Simulation;
  final robot:AssemblyRobot;
  final objects:Array<GripObject>;
  final project:ProjectDocumentSession;
  final definitions:Map<String, String> = new Map();
  final cache:Map<String, SimulationAssemblyPart> = new Map();

  public function new(simulation:Simulation, robot:AssemblyRobot, objects:Array<GripObject>, project:ProjectDocumentSession) {
    this.simulation = simulation;
    this.robot = robot;
    this.objects = objects;
    this.project = project;
    var assembly = project.projectAssemblyDefinition;
    if (assembly == null) throw "Simulated assembly parts require an assembly";
    for (occurrence in AssemblyDefinitionFlattener.flatten(assembly).occurrences)
      definitions.set("project:" + occurrence.id, occurrence.definition);
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
    var vertices:Array<Float> = [];
    if (record.collisionEnabled) {
      var found = [for (body in physical.parts) if (body.id == definition) body];
      if (found.length != 1 || found[0].collisionHull == null) throw 'No CAD collision hull for "$id"';
      var hull:Array<Float> = cast found[0].collisionHull;
      vertices = [for (value in hull) value * physical.metresPerUnit];
    }
    var parts = [for (part in robot.parts) if (part.id == id) part];
    var bodies = [for (entry in objects) if (entry.id == id) entry.object];
    if (parts.length + bodies.length != 1) throw 'No unique simulation carrier for "$id"';
    var result = new SimulationAssemblyPart(id, simulation, [for (value in centre) value * physical.metresPerUnit],
      vertices, parts.length == 1 ? parts[0] : null, bodies.length == 1 ? bodies[0] : null);
    cache.set(id, result);
    return result;
  }
}
