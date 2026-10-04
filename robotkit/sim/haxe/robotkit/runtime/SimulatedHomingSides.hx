package robotkit.runtime;

/** Resolve home switches to distinct shaft holds in one simulation-owned runtime. */
class SimulatedHomingSides implements HomingSideControl {
  final simulation:Simulation;
  final robotIndex:Int;
  final runtime:RobotRuntime;
  final drives:Map<String, SwitchDriveBinding> = new Map();
  final held:Map<Int, Float> = new Map();

  @:allow(robotkit.runtime.Simulation)
  private function new(simulation:Simulation, robotIndex:Int, runtime:RobotRuntime,
      blueprint:RobotRuntimeBlueprint) {
    this.simulation = simulation; this.robotIndex = robotIndex; this.runtime = runtime;
    if (blueprint == null) throw "Homing sides require a compiled blueprint";
    for (contact in blueprint.switches) if (contact.role == "home" && contact.driveJoint != null) {
      if (drives.exists(contact.id)) throw "Duplicate side home switch";
      drives.set(contact.id, SwitchDriveBinding.resolve(blueprint, contact));
    }
  }

  public function hold(switchId:String):Void {
    var drive = drives.get(switchId);
    if (drive == null) throw 'Home switch "$switchId" has no motor-side binding';
    if (held.exists(drive.joint)) return;
    var positions = runtime.physicalPositions();
    if (drive.joint >= positions.length) throw "Homing shaft snapshot is incomplete";
    var position = positions[drive.joint];
    simulation.setSquaringHold(robotIndex, drive.joint, true, position);
    held.set(drive.joint, position);
  }

  public function calibrate(switchIds:Array<String>):Void {
    for (_ in held.keys()) throw "Release all shaft holds before motor calibration";
    runtime.calibrateHomeDrives(switchIds);
  }

  public function releaseAll():Void {
    var joints = [for (joint in held.keys()) joint];
    var failure:Null<String> = null;
    for (joint in joints) {
      try {
        simulation.setSquaringHold(robotIndex, joint, false, held.get(joint));
        held.remove(joint);
      } catch (error:Dynamic) {
        if (failure == null) failure = Std.string(error);
      }
    }
    if (failure != null) throw failure;
  }
}
