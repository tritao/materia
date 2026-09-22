package app;

import haxe.Int64;
import robotkit.runtime.RobotRuntimeCompiler;
import robotkit.runtime.Simulation;
import robotkit.world.Robot;
import robotkit.world.RobotWorld;
import robotkit.world.SimulatedRobot;
import robotkit.world.WorldSnapshot;

/** Owns the one shared editable-scene simulation attached to the application world. */
class ApplicationSimulation {
  public final world:RobotWorld;
  public var appliedRevision(default, null):Int = 0;
  public var appliedDocumentRevision(default, null):Int = -1;
  public var error(default, null):Null<String> = null;
  var simulation:Null<Simulation> = null;
  var simulatedIds:Array<String> = [];
  var running:Bool = false;

  public function new(world:RobotWorld) this.world = world;

  public function pending(configuration:SensorConfiguration):Bool
    return appliedDocumentRevision != configuration.revision();

  /** Builds the complete candidate before changing any live world adapter. */
  public function rebuild(configuration:SensorConfiguration, scene:EditorScene):Bool {
    var candidate:Null<Simulation> = null;
    var candidateRobots:Array<SimulatedRobot> = [];
    try {
      var models = configuration.robotModels();
      if (models.length == 0) throw "No simulated robot configurations";
      for (configured in models) {
        var id=configured.id;
        var existing = world.robot(id);
        if (existing != null && simulatedIds.indexOf(id) < 0)
          throw 'Robot "$id" is remote and read-only';
      }
      candidate = new Simulation();
      for (index in 0...models.length) {
        var editable=models[index];
        var blueprint = RobotRuntimeCompiler.compile(editable.model, appliedRevision + 1);
        var runtime = candidate.addRobot(blueprint);
        var id = editable.id;
        candidateRobots.push(new SimulatedRobot(id, runtime, editable.model.name,
          [for (link in editable.model.links) link.id], [for (joint in editable.model.joints) joint.id]));
        if (index > 0) candidate.teleportRobot(index, [0.0, index * 2.0, 0.0]);
      }
      for (object in scene.records()) if (object.visible)
        candidate.spawnBox([object.x, object.y, object.z],
          [object.width / 2.0, object.height / 2.0, 0.05]);
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
      appliedRevision++;
      appliedDocumentRevision = configuration.revision();
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
  public function simulatedRobotIds():Array<String> return simulatedIds.copy();
  public function dispose():Void {
    for (id in simulatedIds) { var robot=world.detach(id); if(robot!=null)robot.close(); }
    simulatedIds.resize(0);
    if (simulation != null) simulation.dispose(); simulation = null;
  }
}
