package app;

import materia.project.SceneArtifact.SceneArtifactMobileBase;
import materia.project.SceneArtifact.SceneArtifactMobileEnergy;
import machinekit.power.BatterySimulator;
import machinekit.power.BatteryStorage;
import processkit.simulation.SimulatedWelder;
import robotkit.power.BatteryState;
import robotkit.power.Power;

/** Integrates measured mobile-welder loads into the CAD pack's simulated state. */
class SimulatedMobilePower implements SessionMember implements Power {
  public final profile:SceneArtifactMobileEnergy;
  final mobileBase:SceneArtifactMobileBase;
  public final battery:BatterySimulator;
  public var armMechanicalWatts(default, null):Float = 0;
  public var driveMechanicalWatts(default, null):Float = 0;
  public var weldAcWatts(default, null):Float = 0;
  public var packLoadWatts(default, null):Float = 0;
  public var charging(default, null):Bool = false;

  final simulation:robotkit.runtime.Simulation;
  final robot:AssemblyRobot;
  final robotIndex:Int;
  final timestep:Float;
  final welder:SimulatedWelder;
  var latest:BatteryState;

  public function new(profile:SceneArtifactMobileEnergy, mobileBase:SceneArtifactMobileBase, robot:AssemblyRobot,
      simulation:robotkit.runtime.Simulation, robotIndex:Int, timestep:Float,
      welder:SimulatedWelder) {
    if (profile == null || mobileBase == null || robot == null || simulation == null || robotIndex < 0 ||
        !Math.isFinite(timestep) || timestep <= 0 || welder == null)
      throw "Mobile power simulation needs its CAD profile, robot, timestep and welder";
    this.profile = profile;
    this.mobileBase = mobileBase;
    this.robot = robot;
    this.simulation = simulation;
    this.robotIndex = robotIndex;
    this.timestep = timestep;
    this.welder = welder;
    var storage = new BatteryStorage(profile.nominalVolts, profile.capacityWh,
      profile.fullVoltage, profile.emptyVoltage, profile.cutoffVoltage,
      profile.internalResistanceOhms, profile.maxDischargeCurrentA,
      profile.maxChargePowerW, profile.chargeEfficiency);
    battery = new BatterySimulator(profile.batteryOccurrence, storage, profile.initialChargeFraction);
    var initial = battery.reading();
    latest = new BatteryState(initial.batteryId, initial.chargeFraction, initial.voltageVolts,
      initial.currentAmps, initial.temperatureCelsius, initial.sourceTimestampNs,
      initial.sourceTimestampNs, "simulation", "simulation", initial.remainingEnergyWattHours);
  }

  public function batteryState():Null<BatteryState> return latest;

  /** MissionPlayer requests charging only during its dock-and-charge transfer. */
  public function requestCharging(value:Bool):Void charging = value;

  /** The CAD contact pose and alignment bounds must be satisfied before the station delivers energy. */
  public function atDock():Bool {
    var base = simulation.linkPose(robotIndex, 0);
    var dx = base.position[0] - profile.dockPose.x;
    var dy = base.position[1] - profile.dockPose.y;
    var yaw = Math.atan2(2 * (base.rotation[3] * base.rotation[2] + base.rotation[0] * base.rotation[1]),
      1 - 2 * (base.rotation[1] * base.rotation[1] + base.rotation[2] * base.rotation[2]));
    var difference = Math.atan2(Math.sin(yaw - profile.dockPose.yaw), Math.cos(yaw - profile.dockPose.yaw));
    return dx * dx + dy * dy <= profile.dockPositionToleranceMeters * profile.dockPositionToleranceMeters &&
      Math.abs(difference) <= profile.dockHeadingToleranceRadians;
  }

  public function feed():Void {
    var snapshot = robot.robot.snapshot();
    armMechanicalWatts = 0;
    driveMechanicalWatts = 0;
    var names = robot.robot.description().joints;
    var leftWheel = profileWheel("left"), rightWheel = profileWheel("right");
    if (names.length != snapshot.efforts.length || names.length != snapshot.velocities.length)
      throw "Mobile power snapshot and CAD joint map have different lengths";
    for (index in 0...names.length) {
      var mechanical = snapshot.efforts.get(index) * snapshot.velocities.get(index);
      // The stepper drives do not regenerate during braking; negative joint work is dissipated.
      if (mechanical <= 0) continue;
      var name = names[index];
      if (name == leftWheel || name == rightWheel)
        driveMechanicalWatts += mechanical;
      else
        armMechanicalWatts += mechanical;
    }
    var reading = welder.reading();
    weldAcWatts = reading.powerW;
    var armDc = armMechanicalWatts / (profile.armDriveEfficiency * profile.armSupplyEfficiency);
    var driveDc = driveMechanicalWatts / (profile.driveMotorEfficiency * profile.driveGearEfficiency *
      profile.driveSupplyEfficiency);
    var computerDc = profile.computerLoadWatts / profile.computerSupplyEfficiency;
    var weldDc = weldAcWatts / profile.inverterEfficiency;
    packLoadWatts = armDc + driveDc + computerDc + weldDc;
    var chargerWatts = charging && atDock() ? profile.chargerPowerWatts : 0.0;
    var result = battery.advance(packLoadWatts, chargerWatts, timestep, snapshot.sourceTimestampNs);
    latest = new BatteryState(result.batteryId, result.chargeFraction, result.voltageVolts,
      result.currentAmps, result.temperatureCelsius, result.sourceTimestampNs,
      snapshot.receivedTimestampNs, snapshot.sourceClockId, snapshot.receivedClockId,
      result.remainingEnergyWattHours);
    welder.supplyReady = battery.supplyAvailable && reading.powerW <= profile.inverterContinuousWatts;
  }

  function profileWheel(side:String):String {
    return side == "left" ? mobileBase.leftWheel : mobileBase.rightWheel;
  }

  public function beforeReset():Void {
    charging = false;
    welder.supplyReady = true;
  }

  public function reset():Void {
    charging = false;
    battery.reset();
    var initial = battery.reading();
    latest = new BatteryState(initial.batteryId, initial.chargeFraction, initial.voltageVolts,
      initial.currentAmps, initial.temperatureCelsius, initial.sourceTimestampNs,
      initial.sourceTimestampNs, "simulation", "simulation", initial.remainingEnergyWattHours);
    armMechanicalWatts = driveMechanicalWatts = weldAcWatts = packLoadWatts = 0;
    welder.supplyReady = battery.supplyAvailable;
  }

  public function present():Void {}
}
