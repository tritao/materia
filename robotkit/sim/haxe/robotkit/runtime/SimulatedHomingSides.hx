package robotkit.runtime;

/** Resolve home switches to distinct shaft holds in one simulation-owned runtime. */
class SimulatedHomingSides implements HomingSideControl {
  final simulation:Simulation;
  final robotIndex:Int;
  final runtime:RobotRuntime;
  final drives:Map<String, SwitchDriveBinding> = new Map();
  final held:Map<Int, Float> = new Map();
  var squaring:Array<String> = [];

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

  public function beginSquaring(switchIds:Array<String>):Void {
    if (squaring.length > 0 || switchIds == null || switchIds.length < 2)
      throw "Squaring requires an idle controller and multiple sides";
    var shafts:Array<Int> = [];
    for (id in switchIds) {
      var drive = drives.get(id);
      if (drive == null || shafts.indexOf(drive.joint) >= 0) throw "Squaring requires distinct known side shafts";
      shafts.push(drive.joint);
    }
    squaring = switchIds.copy();
  }

  public function endSquaring():Void {
    releaseAll();
    squaring = [];
  }

  public function hold(switchId:String):Void {
    if (squaring.indexOf(switchId) < 0) throw "Side holds require an explicit squaring move";
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
    if (switchIds == null || switchIds.length != squaring.length || squaring.length == 0)
      throw "Calibrate the complete active squaring group";
    var seen = new Map<String, Bool>();
    for (id in switchIds) {
      if (squaring.indexOf(id) < 0 || seen.exists(id)) throw "Calibration does not match the squaring group";
      seen.set(id, true);
    }
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
