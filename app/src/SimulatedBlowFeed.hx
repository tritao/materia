package app;

import haxe.Int64;
import processkit.feeding.ScrewFeederDevice;
import processkit.feeding.ScrewFeederDevice.ScrewFeederReading;
import processkit.feeding.BlowFeedModel;
import processkit.simulation.SimulatedScrewdriver;
import processkit.screwdriving.ScrewScenePlan;
import materia.project.SceneBlowFeed;
import materia.project.SceneScrewDefinition.SceneScrew;
import robotkit.runtime.RobotRuntime;
import robotkit.runtime.SimulationStepObserver;
import robotkit.spatial.Transform3;
import robotkit.spatial.Vec3;
import robotkit.spatial.Quat;

/** Places the separated identified body at a named live tool socket; receipt and sealing remain physical. */
class SimulatedBlowFeed implements ScrewFeederDevice implements SimulationStepObserver {
  public final definition:SceneBlowFeed;
  public final model:BlowFeedModel;
  final runtime:RobotRuntime;
  final parts:SimulationAssemblyParts;
  final driver:SimulatedScrewdriver;
  final socket:Transform3;
  var selected:Null<SceneScrew> = null;
  public function new(definition:SceneBlowFeed, runtime:RobotRuntime, parts:SimulationAssemblyParts, driver:SimulatedScrewdriver) {
    this.definition = definition; this.runtime = runtime; this.parts = parts; this.driver = driver;
    socket = ScrewScenePlan.frame(parts.connectorFrame("project:"+definition.receiver,definition.socketConnector));
    for (id in definition.inventory) {var body = parts.get("project:"+id).freeBody();}
    model = new BlowFeedModel(definition.inventory,definition.bufferCapacity,definition.trackDelay,definition.deliveryDelay,
      definition.deliveryTimeout,definition.airPulse,runtime.sourceClockId,ready,present,(identity) -> {
        var job = selected;
        if (job == null || identity != job.screw) throw 'Blow-feed "${definition.id}" cannot deposit an unselected screw';
        var receiver = parts.get("project:"+definition.receiver).pose(), part = parts.get("project:"+identity);
        var live = new Transform3(Vec3.fromArray(receiver.position),new Quat(receiver.rotation[0],receiver.rotation[1],receiver.rotation[2],receiver.rotation[3]));
        var body = live.compose(socket).compose(ScrewScenePlan.frame(job.socket).inverse())
          .compose(new Transform3(Vec3.fromArray(part.center),Quat.identity()));
        part.freeBody().setPoseForNextTick(ScrewScenePlan.pose(body));
      });
  }
  public function select(job:SceneScrew):ScrewFeederDevice {
    if (definition.inventory.indexOf(job.screw) < 0) throw 'Blow-feed "${definition.id}" does not own "${job.screw}"';
    selected = job; return this;
  }
  function ready():Bool {
    var state = runtime.snapshot();
    if (state.safety == RobotKitRuntimeConstants.RK_SAFETY_FAULT || state.safety == RobotKitRuntimeConstants.RK_SAFETY_EMERGENCY_STOP || driver.holding.held != null) return false;
    return driver.receivingReady(definition.receivingAngularSpeed,definition.receivingFeedSpeed);
  }
  function present():Bool {
    var state = runtime.snapshot();
    for (i in 0...state.sensors.length) {
      var sensor = state.sensors.get(i);
      if (sensor.sensorId == definition.receiptSensor) {
        if (sensor.kind != "presence_input" || sensor.values.length != 1) throw 'Blow-feed "${definition.id}" receipt acquisition is invalid';
        return sensor.values.get(0) >= 0.5;
      }
    }
    throw 'Blow-feed "${definition.id}" lost its physical receipt sensor';
  }
  public function enable(on:Bool):Void model.enable(on);
  public function request():Void {
    var job = selected;
    if (job == null || model.transport.nextIdentity() != job.screw) throw 'Blow-feed "${definition.id}" buffered identity disagrees with the fastening';
    model.request();
  }
  public function reading():Null<ScrewFeederReading> return model.reading();
  public function faultDetail():Null<String> return model.faultDetail();
  public function safe():Void model.safe();
  public function reset():Void {model.reset(); selected = null;}
  public function poll(time:Float):Void model.poll(time);
  public function afterSimulationStep(timestamp:Int64):Void {
    var state = runtime.snapshot();
    if (state.safety == RobotKitRuntimeConstants.RK_SAFETY_FAULT || state.safety == RobotKitRuntimeConstants.RK_SAFETY_EMERGENCY_STOP) safe();
    var id = model.transport.delivered, job = selected;
    if (id != null && job != null && driver.holding.held == null && !present()
        && parts.get("project:"+job.frame).carries(parts.get("project:"+id).freeBody())) model.confirmConsumed(id);
    model.poll(Int64.toFloat(timestamp)*1e-9);
  }
}
