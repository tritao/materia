package tests;

import cadbridge.EndEffectorBridge;
import cadbridge.EndEffectorCollision;
import cadbridge.EndEffectorCollision.EndEffectorCollisionOptions;
import cadbridge.EndEffectorRuntimeBridge;
import cadbridge.EndEffectorVacuumFeedback;
import eoat.EndEffectorExample;
import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.component.ComponentDetail;
import machinekit.component.MachineComponent;
import machinekit.component.Solids;
import machinekit.pneumatic.SuctionCup;
import machinekit.pneumatic.VacuumGenerator;
import machinekit.pneumatic.VacuumPressureSensor;
import machinekit.robotics.EndEffector;
import machinekit.robotics.EndEffectorSet;
import haxe.Int64;
import robotkit.runtime.RobotRuntimeBlueprint;
import robotkit.runtime.RobotRuntime;
import robotkit.runtime.Simulation;
import robotkit.runtime.SimulationHarness;
import robotkit.world.SimulatedRobot;
import robotkit.world.RobotRecording;
import robotkit.world.ReplayRobot;
import robotkit.world.SensorFrame;
import robotkit.tool.ToolCollisionShape;
import robotkit.tool.ToolCollisionShapes;
import robotkit.tool.ToolRuntimeSelection;
import robotkit.tool.SimulatedVacuum;
import robotkit.world.FiredProcessEvent;
import robotkit.world.ProcessEventValue;

private class AirSource extends MachineComponent {
  public function new() {
    super("AIR-SOURCE", "simulation air source", "steel", true);
    addConnector("mount", Mount, Solids.axial(0, 0, 0));
    addConnector("couple", Mount, Solids.axial(0, 0, 10));
    addPort({name: "air", kind: Pneumatic, role: Supply,
      iface: Unspecified, required: false});
    declareMass(1, new Vector(0, 0, 5), cadkit.InertiaTensor.zero());
  }

  override public function geometry(detail:ComponentDetail = Preview):Part
    return Part.box(10, 10, 10);
}

/** The example cup reports a proximity contact without touching other pieces. */
class EndEffectorContactMuJoCo {
  public static function main():Void {
    testCupProximity();
    testVacuumFeedback();
    Sys.println("CadBridge MuJoCo cup proximity and vacuum feedback passed");
  }

  static function testCupProximity():Void {
    var effector = EndEffectorExample.build().configuration("short");
    var options = new EndEffectorCollisionOptions(10, 16, 10);
    var collision = EndEffectorCollision.pieces(effector, null, options);
    var cupIndex = -1;
    for (index in 0...collision.pieces.length)
      if (collision.pieces[index].memberIds.indexOf("tool/cup") >= 0) cupIndex = index;
    if (cupIndex < 0) throw "Example cup has no collision piece";
    var cup = ToolCollisionShapes.bounds(ToolCollisionShape.Hulls(
      [collision.pieces[cupIndex].vertices], 0));
    var tool = EndEffectorBridge.toTool(effector, "contact", null, "short/contact", options);
    var simulationHarness = new SimulationHarness(0.01, 2, 1);

    var simulation = simulationHarness.simulation;
    var runtime = simulation.addRobotAtPose(new RobotRuntimeBlueprint(1, 0, 1),
      [0, 0, 0], [0, 0, 0, 1], null, null, null, null, tool.collision, 0);
    var obstacle = simulationHarness.spawnBox([
      cup.centre.x, cup.centre.y, cup.centre.z + cup.halfExtents.z + 0.007],
      [0.005, 0.005, 0.005]);
    simulationHarness.step(Int64.ofInt(0));
    var contacts = runtime.toolProximity();
    if (contacts.length == 0) throw "Example cup proximity was not reported";
    for (contact in contacts)
      if (contact.toolPieceIndex != cupIndex || contact.otherObject != obstacle.handle.rawValue())
        throw "Proximity came from a piece other than the example cup";
    simulationHarness.dispose();
  }

  static function testVacuumFeedback():Void {
    var set = new EndEffectorSet();
    set.addComponent("source", new AirSource());
    set.mount("source", "mount");
    set.exposePort("air", "source", "air");
    set.changer("manual", "source", "couple", [{robot: "air", tool: "air"}]);
    var effector = new EndEffector();
    effector.addComponent("generator", new VacuumGenerator());
    effector.addComponent("sensor", new VacuumPressureSensor(6));
    effector.addComponent("cup", new SuctionCup(25, 18));
    effector.mount("generator", "mount");
    effector.addMemberConnector("generator", "sensor-seat", Solids.axial(0, 0, 40));
    effector.addMemberConnector("generator", "cup-seat", Solids.axial(0, 0, 100));
    effector.addMate("sensor-mate", "fixed", "generator", "sensor-seat", "sensor", "mount");
    effector.addMate("cup-mate", "fixed", "generator", "cup-seat", "cup", "mount");
    effector.connectPorts("generator-sensor", "generator", "vacuum", "sensor", "vacuumIn");
    effector.connectPorts("sensor-cup", "sensor", "vacuumOut", "cup", "vacuum");
    effector.exposePort("air", "generator", "air");
    effector.workingFrame("contact", "cup", "contact", true);
    set.addTool("sensed", effector);
    var bundle = EndEffectorRuntimeBridge.toRuntimeFromDesign(set, "sensed", "contact");
    var vacuum:SimulatedVacuum = cast bundle.runtime.vacuum;
    var pieces = EndEffectorCollision.pieces(set.configuration("sensed"));
    var cupIndex = -1;
    for (index in 0...pieces.pieces.length)
      if (pieces.pieces[index].memberIds.indexOf("tool/cup") >= 0) cupIndex = index;
    if (cupIndex < 0) throw "Sensed cup has no tool piece";
    var bounds = ToolCollisionShapes.bounds(ToolCollisionShape.Hulls(
      [pieces.pieces[cupIndex].vertices], 0));
    var blueprint = new RobotRuntimeBlueprint(1, 0, 1);
    var authored = EndEffectorRuntimeBridge.addVacuumSensorToBlueprint(set,
      "sensed", blueprint, 0);
    if (authored.id != bundle.bindings.vacuumSensorId ||
        authored.kind != "tool_vacuum_kpa" || authored.position.get(2) <= 0)
      throw "EOAT pressure sensor was not mounted on the flange blueprint";
    var simulationHarness = new SimulationHarness(0.01, 2, 1);

    var simulation = simulationHarness.simulation;
    var robot = simulation.addRobotAtPose(blueprint,
      [0, 0, 0], [0, 0, 0, 1], null, null, null, null,
      bundle.runtime.tool.collision, 0, 0, 0.01);
    var selection = new ToolRuntimeSelection();
    selection.select(bundle.runtime, simulation.sourceTimestampNs(), "robotkit.simulation");
    var feedback = new EndEffectorVacuumFeedback(set, "sensed", selection, bundle, robot, 55);
    feedback.attach(simulation);
    selection.apply(new FiredProcessEvent(Int64.ofInt(1), "sensed/tool/generator.enable",
      ProcessEventValue.Digital(true), Int64.ofInt(1), Int64.ofInt(1), 1));
    simulationHarness.spawnBox([bounds.centre.x, bounds.centre.y,
      bounds.centre.z + bounds.halfExtents.z + 0.007], [0.005, 0.005, 0.005]);
    simulationHarness.step(Int64.ofInt(2));
    var nearbyStep = simulation.stepIndex();
    var nearbyFrame = pressureFrame(robot, authored.id);
    if (nearbyFrame.values.get(0) != 0 || vacuum.isHolding() ||
        vacuum.vacuumKpa() != 0 || nearbyFrame.sourceClockId != "robotkit.simulation")
      throw "Cup proximity must not produce a seal pressure";
    var touchingObject = simulationHarness.spawnBox([bounds.centre.x, bounds.centre.y,
      bounds.centre.z + bounds.halfExtents.z + 0.003], [0.005, 0.005, 0.005]);
    simulationHarness.step(Int64.ofInt(3));
    var touching = false;
    for (contact in robot.contacts())
      if (contact.toolPieceIndex == cupIndex && contact.distance <= 0) {
        if (Int64.compare(contact.stepIndex, simulation.stepIndex()) != 0)
          throw "Captured contact step must match the session tick";
        if (Int64.compare(contact.stepIndex, nearbyStep) <= 0)
          throw "Captured contact step must increase with session steps";
        touching = true;
      }
    var touchingFrame = pressureFrame(robot, authored.id);
    if (!touching || touchingFrame.values.get(0) != 55 ||
        Int64.compare(touchingFrame.sequence, nearbyFrame.sequence) <= 0 ||
        !vacuum.isHolding() || vacuum.vacuumKpa() != 55)
      throw 'Touching cup must produce simulated pressure feedback (touching=$touching, pressure=${touchingFrame.values.get(0)}, holding=${vacuum.isHolding()}, contacts=${robot.contacts().length})';
    var simulatedRobot = new SimulatedRobot("sensed-robot", robot, "sensed robot",
      ["base_link"], []);
    var recording = new RobotRecording();
    recording.recordSnapshot(simulatedRobot.snapshot());
    simulationHarness.removeObject(touchingObject);
    simulationHarness.step(Int64.ofInt(4));
    var lostFrame = pressureFrame(robot, authored.id);
    if (lostFrame.values.get(0) != 0 || vacuum.isHolding() ||
        vacuum.vacuumKpa() != 0)
      throw "Losing cup contact must release simulated vacuum holding";
    recording.recordSnapshot(simulatedRobot.snapshot());
    var replay = new ReplayRobot("sensed-robot", recording);
    if (sensorValue(replay.sensors(), authored.id) != 55 || !replay.advance() ||
        sensorValue(replay.sensors(), authored.id) != 0)
      throw "Recorded pressure frames must replay in order";
    selection.select(null, simulation.sourceTimestampNs(), "robotkit.simulation");
    simulationHarness.step(Int64.ofInt(5));
    if (vacuum.isHolding() || pressureFrame(robot, authored.id).values.get(0) != 0)
      throw "Deselecting the tool must release its vacuum state";
    var finalSequence = pressureFrame(robot, authored.id).sequence;
    feedback.detach();
    simulationHarness.step(Int64.ofInt(6));
    if (Int64.compare(pressureFrame(robot, authored.id).sequence, finalSequence) != 0)
      throw "Detached vacuum feedback must stop publishing sensor frames";
    replay.close();
    simulatedRobot.close();
    simulationHarness.dispose();
  }

  static function pressureFrame(robot:RobotRuntime, id:String):SensorFrame {
    for (frame in robot.snapshot().sensors.toArray()) if (frame.sensorId == id) return frame;
    throw 'Runtime snapshot has no EOAT pressure sensor "$id"';
  }

  static function sensorValue(frames:Array<SensorFrame>, id:String):Float {
    for (frame in frames) if (frame.sensorId == id) return frame.values.get(0);
    throw 'Recorded snapshot has no EOAT pressure sensor "$id"';
  }
}
