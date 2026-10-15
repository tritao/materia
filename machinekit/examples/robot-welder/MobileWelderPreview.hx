import haxe.io.Bytes;
import machinekit.assembly.AssemblyPreview;
import machinekit.robot.RobotScene;
import cadkit.modeling.AssemblyModel;
import cadkit.modeling.AssemblyState;
import materia.project.SceneArtifact;
import machinekit.power.BatteryStorage;
import MobileBase;

/** Mobile welding scene. Station planning supplies its mission in P2. */
class MobileWelderPreview {
  public static inline var ASSEMBLY_ID:String = "mobile-welder";
  public static function cell():Bytes return SceneArtifact.encode(design(new MobileWelderCell()));

  public static function plannedCell():Bytes {
    var cell = new MobileWelderCell();
    var scene = design(cell);
    scene.missions = [{steps: new MobileWeldStations(cell, scene).mission()}];
    return SceneArtifact.encode(scene);
  }

  public static function design(cell:MobileWelderCell):materia.project.SceneArtifact.SceneArtifactData {
    var scene = AssemblyPreview.scene(cell, ASSEMBLY_ID);
    var model = new AssemblyModel("mm");
    cell.addTo(model, "");
    var state = new AssemblyState(model.definition(ASSEMBLY_ID));
    var shoulder = 0.25, elbow = Math.PI / 2 - 1.4;
    var ready = [0.0, shoulder, elbow, 0.0, Math.PI / 2 - shoulder + elbow, 0.0];
    for (index in 0...6) state.setJoint('robot/arm/j${index + 1}', ready[index]);
    scene.assemblyState = state.record();
    var arm:RobotArm = cast cell.robot.arm;
    scene.robotTools = RobotScene.robotTools(arm.tool, "robot/arm/tool", cell.equipment());
    scene.robotSensors = RobotScene.robotSensors(cell.robot, "robot/");
    var robot = cell.robot;
    // Reserve 20% of the drive effort; this heavy payload cannot use the light base's acceleration.
    var mass = robot.massProperties().mass;
    var acceleration = 0.8 * 2 * robot.wheelTorque() / (robot.wheel.radius / 1000) / mass;
    var mobile:SceneArtifact.SceneArtifactMobileBase = {robot: "robot", leftWheel: "robot/wheel_l", rightWheel: "robot/wheel_r",
      wheelRadius: robot.wheel.radius / 1000, trackWidth: robot.trackWidth() / 1000,
      maxLinearSpeed: MobileBase.MAX_LINEAR_SPEED, maxAngularSpeed: MobileBase.MAX_ANGULAR_SPEED,
      maxLinearAcceleration: Math.min(MobileBase.MAX_LINEAR_ACCELERATION, acceleration),
      maxAngularAcceleration: Math.min(MobileBase.MAX_ANGULAR_ACCELERATION,
        acceleration * 6 * robot.trackWidth() / 1000 / (Math.pow(robot.length / 1000, 2) + Math.pow(robot.width / 1000, 2))),
      footprintLength: robot.length / 1000, footprintWidth: robot.width / 1000,
      origin: {x: MobileWelderCell.ORIGIN.x / 1000, y: MobileWelderCell.ORIGIN.y / 1000, yaw: MobileWelderCell.ORIGIN.yaw}};
    var storage:BatteryStorage = cast BatteryStorage.of(robot.battery);
    var chargerPose = cell.solvedPoses().get("charger");
    var dock = materia.assembly.AssemblyFrames.compose(chargerPose, cell.charger.dockApproach);
    var dockYaw = Math.atan2(2 * (dock.qw * dock.qz + dock.qx * dock.qy),
      1 - 2 * (dock.qy * dock.qy + dock.qz * dock.qz));
    var driveSupply = robot.driveSupply;
    if (driveSupply == null) throw "The mobile welder needs a modeled drive supply";
    mobile.energy = {
      initialChargeFraction: 1.0, batteryOccurrence: "robot/battery", nominalVolts: storage.nominalVolts,
      capacityWh: storage.capacityWh, fullVoltage: storage.fullVoltage,
      emptyVoltage: storage.emptyVoltage, cutoffVoltage: storage.cutoffVoltage,
      internalResistanceOhms: storage.internalResistanceOhms,
      maxDischargeCurrentA: storage.maxDischargeCurrentA,
      maxChargePowerW: storage.maxChargePowerW, chargeEfficiency: storage.chargeEfficiency,
      inverterContinuousWatts: robot.inverter.continuousWatts,
      inverterEfficiency: robot.inverter.efficiency,
      armSupplyEfficiency: robot.armSupply.efficiency,
      driveSupplyEfficiency: driveSupply.efficiency,
      computerSupplyEfficiency: robot.computerSupply.efficiency,
      armDriveEfficiency: MobileWeldingRobot.ARM_DRIVE_EFFICIENCY,
      driveMotorEfficiency: MobileWeldingRobot.DRIVE_MOTOR_EFFICIENCY,
      driveGearEfficiency: MobileBase.WHEEL_GEARBOX.efficiency,
      computerLoadWatts: MobileWeldingRobot.COMPUTER_LOAD_WATTS,
      chargerOccurrence: "charger", chargerPowerWatts: cell.charger.outputWatts,
      dockPose: {x: dock.x / 1000, y: dock.y / 1000, yaw: dockYaw},
      dockPositionToleranceMeters: cell.charger.dockPositionToleranceMm / 1000,
      dockHeadingToleranceRadians: cell.charger.dockHeadingToleranceRad};
    scene.mobileBase = mobile;
    return scene;
  }
}
