package app;

import haxe.Int64;
import processkit.feeding.ScrewFeederDevice;
import processkit.feeding.ScrewFeederDevice.ScrewFeederReading;
import processkit.feeding.ScrewFeederModel;
import materia.project.SceneScrewFeeder;
import robotkit.runtime.Simulation;
import robotkit.runtime.SimulationStepObserver;
import robotkit.runtime.RobotRuntime;
import processkit.simulation.SimulatedScrewdriver;
import processkit.screwdriving.ScrewScenePlan;
import robotkit.spatial.Transform3;
import robotkit.spatial.Vec3;
import robotkit.spatial.Quat;

/** Deposits identified inventory into a supported CAD nest; presence still comes from physical geometry. */
class SimulatedScrewFeeder implements SimulationStepObserver implements ScrewFeederDevice {
  public final definition:SceneScrewFeeder;
  public final model:ScrewFeederModel;
  final runtime:RobotRuntime;
  final simulation:Simulation;
  final parts:SimulationAssemblyParts;
  final driver:SimulatedScrewdriver;
  var selected:Null<String> = null;
  public function new(definition:SceneScrewFeeder, simulation:Simulation, runtime:RobotRuntime,
      parts:SimulationAssemblyParts, driver:SimulatedScrewdriver) {
    this.definition = definition; this.simulation = simulation; this.runtime = runtime; this.parts = parts; this.driver = driver;
    for (id in definition.inventory) {var body = parts.get("project:"+id).freeBody();}
    model = new ScrewFeederModel(definition.inventory,definition.bufferCapacity,definition.trackDelay,
      definition.deliveryDelay,definition.deliveryTimeout,runtime.sourceClockId,present,(id) -> {
        var part = parts.get("project:"+id), nest = parts.get("project:"+definition.presenter).pose();
        var frame = new Transform3(Vec3.fromArray(nest.position),new Quat(nest.rotation[0],nest.rotation[1],nest.rotation[2],nest.rotation[3]));
        var target = frame.compose(ScrewScenePlan.frame(definition.deposit))
          .compose(new Transform3(Vec3.fromArray(part.center),Quat.identity()));
        part.freeBody().setPoseForNextTick(ScrewScenePlan.pose(target));
      });
  }
  public function select(identity:String):ScrewFeederDevice {
    if (definition.inventory.indexOf(identity) < 0) throw 'Feeder "${definition.id}" does not supply "$identity"';
    selected = identity; return this;
  }
  public function faultDetail():Null<String> return model.faultDetail();
  public function enable(on:Bool):Void model.enable(on);
  public function safe():Void model.safe();
  public function reset():Void {model.reset(); selected = null;}
  public function poll(now:Float):Void model.poll(now);
  public function request():Void {
    if (selected == null || model.nextIdentity() != selected)
      throw 'Feeder "${definition.id}" next buffered identity does not match requested screw "$selected"';
    model.request();
  }
  public function reading():Null<ScrewFeederReading> {
    var value = model.reading();
    if (value != null && value.present && model.delivered != selected)
      throw 'Feeder "${definition.id}" pickup nest contains an unexpected screw';
    return value;
  }
  function present():Bool {
    var state = runtime.snapshot();
    for (i in 0...state.sensors.length) {
      var sensor = state.sensors.get(i);
      if (sensor.sensorId == definition.presentSensor) {
        if (sensor.kind != "presence_input" || sensor.values.length != 1)
          throw 'Feeder "${definition.id}" presence acquisition is invalid';
        return sensor.values.get(0) >= 0.5;
      }
    }
    throw 'Feeder "${definition.id}" has no presence acquisition';
  }
  public function afterSimulationStep(timestamp:Int64):Void {
    var safety = runtime.snapshot().safety;
    if (safety == RobotKitRuntimeConstants.RK_SAFETY_FAULT || safety == RobotKitRuntimeConstants.RK_SAFETY_EMERGENCY_STOP) model.safe();
    var delivered = model.delivered;
    if (delivered != null && !present() && driver.holding.held == parts.get("project:"+delivered).freeBody())
      model.confirmPickup(delivered);
    model.poll(Int64.toFloat(timestamp)*1e-9);
  }
}
