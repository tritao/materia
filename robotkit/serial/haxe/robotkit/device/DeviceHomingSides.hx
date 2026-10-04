package robotkit.device;

import haxe.Int64;
import RobotKitRuntime;
import robotkit.runtime.HomingSideControl;
import robotkit.runtime.HomingControlReadiness;
import robotkit.runtime.NativeRuntimeEndpoint;
import robotkit.runtime.RobotRuntime;
import robotkit.runtime.RobotRuntimeBlueprint;
import robotkit.runtime.SwitchDriveBinding;

/** One owner serializes acknowledged physical side controls for a wired device. */
class DeviceHomingSides implements HomingSideControl implements HomingControlReadiness {
  final runtime:RobotRuntime;
  final endpoint:NativeRuntimeEndpoint;
  final channels:Map<String, Int> = new Map();
  final drives:Map<String, SwitchDriveBinding> = new Map();
  final leaders:Map<String, Int> = new Map();
  final held:Map<String, Bool> = new Map();
  final pending:Array<rk_device_homing_control> = [];
  final skewBound:Float;
  final normalBounds:Array<Float>;
  var sequence:Int64 = Int64.ofInt(0);
  var scope:Int64 = Int64.ofInt(0);
  var outstanding:Int64 = Int64.ofInt(0);
  var squaring:Array<String> = [];
  var calibrating:Null<Array<String>> = null;
  var ending:Bool = false;

  public function new(runtime:RobotRuntime, endpoint:NativeRuntimeEndpoint,
      blueprint:RobotRuntimeBlueprint, binding:DeviceBinding, skewBound:Float) {
    if (runtime == null || endpoint == null || blueprint == null || binding == null ||
        !Math.isFinite(skewBound) || skewBound <= 0 || skewBound > 3.4028234663852886e38) throw "Device homing requires explicit wiring and a finite skew bound";
    this.runtime = runtime; this.endpoint = endpoint; this.skewBound = skewBound;
    normalBounds = [for (value in binding.channels) value.skewBound];
    for (contact in blueprint.switches) if (contact.role == "home" && contact.driveJoint != null) {
      drives.set(contact.id, SwitchDriveBinding.resolve(blueprint, contact));
      for (i in 0...blueprint.jointCount) if (blueprint.identity.jointId(i) == contact.joint)
        leaders.set(contact.id, i);
      for (input in binding.inputs) if (input.wiring.switchId == contact.id)
        channels.set(contact.id, input.actuatorChannel);
    }
  }

  function enqueue(action:Int, first:Int, second:Int):Void {
    if (sequence == Int64.make(0x7fffffff, -1)) throw "Device homing sequence exhausted";
    sequence = sequence + Int64.ofInt(1);
    var control = new rk_device_homing_control();
    control.set_struct_size(rk_device_homing_control.size());
    control.set_action(action); control.set_sequence(sequence); control.set_scope(scope);
    control.set_first(first); control.set_second(second); control.set_skew_bound(skewBound);
    pending.push(control);
  }

  function bindingSkew(index:Int):Float return normalBounds[index];

  function channel(id:String):Int {
    var value = channels.get(id);
    if (value == null) throw "Home switch has no physical input actuator: " + id;
    return value;
  }

  public function beginSquaring(switchIds:Array<String>):Void {
    if (squaring.length != 0 || ending || switchIds == null || switchIds.length != 2)
      throw "Device squaring requires one idle two-side group";
    var first = channel(switchIds[0]), second = channel(switchIds[1]);
    var a = drives.get(switchIds[0]), b = drives.get(switchIds[1]);
    var firstLeader = leaders.get(switchIds[0]), secondLeader = leaders.get(switchIds[1]);
    if (firstLeader == null || secondLeader == null) throw "Squaring pair has no leader axis";
    if (first == second || a == null || b == null || a.joint == b.joint ||
        firstLeader != secondLeader || skewBound < bindingSkew(first) || skewBound < bindingSkew(second)) throw "Device squaring requires distinct shafts on one axis";
    scope = sequence + Int64.ofInt(1);
    squaring = switchIds.copy();
    enqueue(0, first, second);
  }

  public function hold(switchId:String):Void {
    if (squaring.indexOf(switchId) < 0 || ending) throw "Side hold requires an active squaring group";
    if (held.exists(switchId)) return;
    enqueue(2, channel(switchId), channel(switchId));
    held.set(switchId, true);
  }

  public function releaseAll():Void {
    for (id in squaring) if (held.exists(id)) {
      enqueue(3, channel(id), channel(id));
      held.remove(id);
    }
  }

  public function endSquaring():Void {
    if (squaring.length == 0 || ending) return;
    releaseAll();
    enqueue(1, channel(squaring[0]), channel(squaring[1]));
    ending = true;
  }

  public function leaderCapture(switchId:String, sideCapture:Float):Float {
    if (squaring.indexOf(switchId) < 0 || !Math.isFinite(sideCapture)) throw "Leader capture requires active squaring";
    var drive = drives.get(switchId), leader = leaders.get(switchId);
    if (drive == null || leader == null) throw "Missing physical side mapping";
    var q = runtime.snapshot().q.toArray();
    var side = drive.position(q[drive.joint] - runtime.referenceOffset(drive.joint));
    var axis = q[leader] - runtime.referenceOffset(leader);
    var capture = sideCapture - (side - axis);
    if (!Math.isFinite(capture)) throw "Invalid compensated leader capture";
    return capture;
  }

  public function calibrate(switchIds:Array<String>):Void {
    if (ending || calibrating != null || switchIds == null || switchIds.length != 2 ||
        squaring.length != 2 || switchIds[0] == switchIds[1]) throw "Calibrate the complete active device pair";
    for (id in switchIds) if (squaring.indexOf(id) < 0 || held.exists(id))
      throw "Release both physical sides before calibration";
    calibrating = switchIds.copy();
  }

  public function controlsReady():Bool {
    if (outstanding != Int64.ofInt(0)) {
      var status = endpoint.deviceHomingStatus(outstanding);
      if (status == RobotKitRuntimeConstants.RK_ERROR_STALE_STATE) return false;
      RobotRuntime.check(status, "device homing acknowledgment");
      outstanding = Int64.ofInt(0);
    }
    if (pending.length > 0) {
      var control = pending[0];
      RobotRuntime.check(RobotKitRuntime.rk_robot_runtime_device_homing_control(endpoint.nativeHandle(), control),
        "device homing control");
      outstanding = control.get_sequence();
      pending.shift();
      return false;
    }
    var ids = calibrating;
    if (ids != null) {
      if (!runtime.tryCalibrateHomeDrives(ids)) return false;
      calibrating = null;
      // Native counter calibration consumes the next sequence in this stream.
      sequence = sequence + Int64.ofInt(1);
    }
    if (ending) { squaring = []; ending = false; }
    return true;
  }
}
