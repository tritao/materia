package app;

import haxe.Int64;
import robotkit.runtime.RobotRuntimeCompiler;
import robotkit.runtime.Simulation;
import robotkit.world.Robot;
import robotkit.world.RobotWorld;
import robotkit.world.SimulatedRobot;
import robotkit.world.WorldSnapshot;
import robotkit.world.SensorFrame;

typedef SimulationRobotVisual={
  var id:String;
  var position:Array<Float>;
  var rotation:Array<Float>;
  var sensors:Array<SensorFrame>;
  var links:Array<SimulationPoseVisual>;
}
typedef SimulationPoseVisual={var id:String;var position:Array<Float>;var rotation:Array<Float>;}

/** Owns the one shared editable-scene simulation attached to the application world. */
class ApplicationSimulation {
  public static inline var DETERMINISTIC:Int=0;
  public static inline var MUJOCO:Int=1;
  public final world:RobotWorld;
  public var appliedRevision(default, null):Int = 0;
  public var appliedDocumentRevision(default, null):Int = -1;
  public var appliedEnvironmentRevision(default, null):Int = -1;
  public var backend(default,null):Int;
  public var appliedBackend(default,null):Int=-1;
  public var error(default, null):Null<String> = null;
  var simulation:Null<Simulation> = null;
  var simulatedIds:Array<String> = [];
  var simulatedLinks:Array<Array<String>> = [];
  var simulatedObjects:Array<{id:String,handle:Int}> = [];
  var running:Bool = false;

  public function new(world:RobotWorld,?backend:Int=DETERMINISTIC) {
    this.world=world;this.backend=DETERMINISTIC;setBackend(backend);
  }

  public function setBackend(value:Int):Bool {
    if(value!=DETERMINISTIC&&value!=MUJOCO)throw "Unsupported simulation backend";
    if(backend==value)return false;backend=value;return true;
  }
  public function backendName():String return backend==MUJOCO?"MuJoCo":"Deterministic";

  public function pending(configuration:SensorConfiguration, scene:EditorScene):Bool
    return appliedDocumentRevision != configuration.revision() ||
      appliedEnvironmentRevision != scene.environmentRevision||appliedBackend!=backend;

  /** Builds the complete candidate before changing any live world adapter. */
  public function rebuild(configuration:SensorConfiguration, scene:EditorScene):Bool {
    var candidate:Null<Simulation> = null;
    var candidateRobots:Array<SimulatedRobot> = [];
    var candidateLinks:Array<Array<String>> = [];
    var candidateObjects:Array<{id:String,handle:Int}> = [];
    try {
      var models = configuration.robotModels();
      if (models.length == 0) throw "No simulated robot configurations";
      for (configured in models) {
        var id=configured.id;
        var existing = world.robot(id);
        if (existing != null && simulatedIds.indexOf(id) < 0)
          throw 'Robot "$id" is remote and read-only';
      }
      candidate = new Simulation(0.01,1,backend);
      for (index in 0...models.length) {
        var editable=models[index];
        var blueprint = RobotRuntimeCompiler.compile(editable.model, appliedRevision + 1);
        var runtime = candidate.addRobot(blueprint);
        var id = editable.id;
        candidateRobots.push(new SimulatedRobot(id, runtime, editable.model.name,
          [for (link in editable.model.links) link.id], [for (joint in editable.model.joints) joint.id]));
        candidateLinks.push([for (link in editable.model.links) link.id]);
        candidate.teleportRobot(index, editable.position, editable.rotation);
      }
      for (object in scene.records()) if (object.collisionEnabled) {
        var handle=candidate.spawnBox([object.x, object.y, object.z],
          [object.width/2.0,object.height/2.0,object.depth/2.0],object.dynamicBody,object.mass);
        candidateObjects.push({id:object.id,handle:handle});
      }
      if (running) candidate.start();

      var previousSimulation = simulation;
      var previousIds = simulatedIds.copy();
      var previousRobots:Array<Robot> = [];
      for (id in previousIds) {
        var detached = world.detach(id);
        if (detached != null) previousRobots.push(detached);
      }
      var attached:Array<String> = [];
      try {
        for (robot in candidateRobots) { world.attach(robot); attached.push(robot.id()); }
      } catch (failure:Dynamic) {
        for (id in attached) world.detach(id);
        for (robot in previousRobots) world.attach(robot);
        throw failure;
      }
      simulation = candidate;
      simulatedIds = [for (robot in candidateRobots) robot.id()];
      simulatedLinks = candidateLinks;
      simulatedObjects = candidateObjects;
      appliedRevision++;
      appliedDocumentRevision = configuration.revision();
      appliedEnvironmentRevision = scene.environmentRevision;
      appliedBackend=backend;
      error = null;
      if (previousSimulation != null) previousSimulation.dispose();
      for (robot in previousRobots) robot.close();
      return true;
    } catch (failure:Dynamic) {
      error = Std.string(failure);
      if (candidate != null && candidate != simulation) candidate.dispose();
      return false;
    }
  }

  public function step(?timestampNs:Int64):WorldSnapshot {
    if (simulation == null) throw "Apply the pending simulation configuration first";
    if (running) throw "Stop realtime simulation before deterministic stepping";
    simulation.step(timestampNs == null ? Int64.ofInt(0) : timestampNs);
    return world.snapshot();
  }
  public function start():Void {
    if (simulation == null) throw "Apply the pending simulation configuration first";
    if (!running) simulation.start(); running = true;
  }
  public function stop():Void { if (simulation != null) simulation.stop(); running = false; }
  public function reset():Bool {
    if (simulation == null) return false;
    simulation.reset(); running = false; return true;
  }
  public function isRunning():Bool return running;
  /** True in both running and paused simulation modes. */
  public function isActive():Bool return simulation!=null;
  public function simulatedRobotIds():Array<String> return simulatedIds.copy();
  public function visualRevision():Int return simulation==null?0:Int64.toInt(simulation.stepIndex());
  /** Read-only runtime state for overlays; never mutates the editable document. */
  public function visualState():Array<SimulationRobotVisual> {
    if(simulation==null)return [];
    var snapshot=world.snapshot();var result:Array<SimulationRobotVisual> = [];
    for(index in 0...simulatedIds.length){
      var id=simulatedIds[index],robot=snapshot.robot(id),pose=simulation.robotPose(index);
      result.push({id:id,position:pose.position,rotation:pose.rotation,
        sensors:robot==null?[]:robot.sensors.toArray(),links:[for(linkIndex in 0...simulatedLinks[index].length) {
          var linkPose=simulation.linkPose(index,linkIndex);
          {id:simulatedLinks[index][linkIndex],position:linkPose.position,rotation:linkPose.rotation};
        }]});
    }
    return result;
  }
  public function environmentVisualState():Array<SimulationPoseVisual> {
    if(simulation==null)return [];
    return [for(object in simulatedObjects){
      var pose=simulation.objectPose(object.handle);
      {id:object.id,position:pose.position,rotation:pose.rotation};
    }];
  }
  public function clear():Void {
    stop();
    for (id in simulatedIds) { var robot=world.detach(id); if(robot!=null)robot.close(); }
    simulatedIds.resize(0);
    simulatedLinks.resize(0); simulatedObjects.resize(0);
    if (simulation != null) simulation.dispose(); simulation = null;
    appliedDocumentRevision=-1;appliedEnvironmentRevision=-1;appliedBackend=-1;
  }
  public function dispose():Void clear();
}
