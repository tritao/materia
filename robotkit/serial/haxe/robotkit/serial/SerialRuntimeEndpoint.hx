package robotkit.serial;
import RobotKitRuntime;
import robotkit.device.DeviceBinding;
import robotkit.runtime.NativeRuntimeEndpoint;
import robotkit.runtime.RuntimeEndpoint;
import robotkit.runtime.RobotRuntime;
import robotkit.runtime.RobotRuntimeBlueprint;
/** Factory for a serial generation-6 endpoint; transport selection stays outside RobotRuntime. */
class SerialRuntimeEndpoint {
  /**
   * Creates a serial runtime for the board `controllerHex` (its 32-digit unique id) wired as
   * `binding` says, within an SI-unit error budget. The blueprint should be compiled from the
   * binding's model so its limits are the device's. The board must be that controller and agree
   * with the configuration, or this throws with the device's reason on stderr.
   */
  public static function create(blueprint:RobotRuntimeBlueprint,
      devicePath:String, controllerHex:String, binding:DeviceBinding, maxTargetError:Float,
      ?baud:Int = 115200, ?linkLossTimeoutNs:haxe.Int64, ?clockSyncBoundNs:haxe.Int64):RuntimeEndpoint {
    if (blueprint == null) throw "Serial runtime requires a compiled blueprint";
    if (devicePath == null || StringTools.trim(devicePath).length == 0)
      throw "Serial runtime requires a device path";
    if (binding == null) throw "Serial runtime requires a device binding";
    if (controllerHex == null || !~/^[0-9a-fA-F]{32}$/.match(controllerHex) ||
        controllerHex.toLowerCase() == "00000000000000000000000000000000")
      throw "Serial runtime requires a nonzero 32-digit controller id";
    if (!Math.isFinite(maxTargetError) || maxTargetError < 0.0)
      throw "Serial runtime requires a finite nonnegative target error budget";
    if (linkLossTimeoutNs == null) linkLossTimeoutNs = haxe.Int64.ofInt(500000000);
    if (clockSyncBoundNs == null) clockSyncBoundNs = haxe.Int64.ofInt(30000000);
    var device = new rk_serial_device_desc();
    device.set_struct_size(rk_serial_device_desc.size());
    device.set_actuator_count(binding.channels.length);
    for (i in 0...16)
      device.set_controller(i, Std.parseInt("0x" + controllerHex.substr(i * 2, 2)));
    for (i in 0...binding.channels.length) {
      var channel = binding.channels[i];
      device.set_actuator_joint(i, channel.jointIndex);
      device.set_actuator_ratio(i, channel.ratio);
      device.set_actuator_offset(i, channel.offset);
      device.set_actuator_steps_per_unit(i, channel.stepsPerUnit);
      device.set_actuator_max_rate(i, channel.maxRate);
      device.set_actuator_direction_setup_ticks(i, channel.directionSetupTicks);
      device.set_actuator_skew_bound(i, channel.skewBound);
      if (channel.actuatorId.length > 63) throw "Serial actuator ID is longer than 63 characters";
      for (byte in 0...channel.actuatorId.length)
        device.set_actuator_ids(i * 64 + byte, channel.actuatorId.charCodeAt(byte));
    }
    var result = RobotKitRuntime.rk_robot_runtime_create_serial6(
      blueprint.nativeValue(), devicePath, baud, device, maxTargetError,
      binding.stepTickHz, linkLossTimeoutNs, clockSyncBoundNs, haxe.Int64.ofInt(100000));
    RobotRuntime.check(result.status, "serialEndpoint.create");
    return new NativeRuntimeEndpoint(result.out_runtime);
  }

  /** Reads the unique id (32 lowercase hex digits) of the board on a serial port. */
  public static function identify(devicePath:String, baud:Int):String {
    if (devicePath == null || StringTools.trim(devicePath).length == 0)
      throw "Identifying a serial device requires a device path";
    var result = RobotKitRuntime.rk_serial_device_identify(devicePath, baud);
    RobotRuntime.check(result.status, "serialEndpoint.identify");
    var text = "";
    for (i in 0...16) text += StringTools.hex(result.out_controller.get_bytes(i), 2).toLowerCase();
    return text;
  }

}
