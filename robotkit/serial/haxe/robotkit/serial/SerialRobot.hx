package robotkit.serial;

import robotkit.core.Robot;
import robotkit.core.RobotCapabilities;
import robotkit.core.RobotCommand;
import robotkit.core.RobotDescription;
import robotkit.core.RobotEvent;
import robotkit.core.RobotFault;
import robotkit.core.RobotId;
import robotkit.core.RobotSnapshot;
import robotkit.core.RobotStatus;
import robotkit.core.SensorFrame;
import robotkit.core.StopMode;
import robotkit.execution.ProcessChannelDeclaration;
import robotkit.runtime.RuntimeRobotAdapter;
import robotkit.world.RobotWorld;

import robotkit.device.DeviceBinding;
import robotkit.device.DeviceLayout;
import robotkit.model.RobotModel;
import robotkit.runtime.RobotRuntime;
import robotkit.runtime.RobotRuntimeBlueprint;
import robotkit.runtime.RobotRuntimeCompiler;

/**
 * RobotWorld adapter that owns and starts a serial runtime for one board, wired to the model's
 * actuators by a device layout.
 */
class SerialRobot implements Robot {
  final adapter:RuntimeRobotAdapter;

  public function new(id:RobotId, model:RobotModel, profile:robotkit.profile.RobotProfile, devicePath:String,
      controllerHex:String, layout:DeviceLayout, maxTargetError:Float, ?baud:Int = 115200,
      ?ownerPeriodNs:haxe.Int64, ?processingAllowanceNs:haxe.Int64,
      ?channels:Array<ProcessChannelDeclaration>, ?stepTickHz:Int = 40000,
      ?linkLossTimeoutNs:haxe.Int64, ?clockSyncBoundNs:haxe.Int64) {
    if (id == null || id.length == 0)
      throw "SerialRobot requires a non-empty logical ID";
    if (model == null) throw "SerialRobot requires a robot model";
    if (layout == null) throw "SerialRobot requires a device layout";
    var binding = DeviceBinding.bind(model, layout, stepTickHz);
    // The runtime plans on the binding's model, whose actuator rates are what the step tick can drive.
    var blueprint:RobotRuntimeBlueprint = RobotRuntimeCompiler.compile(binding.model, profile);
    if (channels != null)
      for (channel in channels) blueprint.channels.push(channel);
    if (ownerPeriodNs != null) blueprint.ownerPeriodNs = ownerPeriodNs;
    if (processingAllowanceNs != null)
      blueprint.serialProcessingAllowanceNs = processingAllowanceNs;
    var runtime = RobotRuntime.create(blueprint, robotkit.serial.SerialRuntimeEndpoint.create(blueprint, devicePath, controllerHex, binding,
      maxTargetError, baud, linkLossTimeoutNs, clockSyncBoundNs));
    adapter = new RuntimeRobotAdapter(id, runtime, model.name,
      [for (link in model.links) link.id], [for (joint in model.joints) joint.id],
      true, true, "serial endpoint fault");
  }

  public function id():RobotId return adapter.id();
  public function status():RobotStatus return adapter.status();
  public function description():RobotDescription return adapter.description();
  public function capabilities():RobotCapabilities return adapter.capabilities();
  public function snapshot():RobotSnapshot return adapter.snapshot();
  public function sensors():Array<SensorFrame> return adapter.sensors();
  public function events(afterOrdinal:haxe.Int64, max:Int):Array<RobotEvent>
    return adapter.events(afterOrdinal, max);
  public function fault():Null<RobotFault> return adapter.fault();
  public function submit(command:RobotCommand):Void adapter.submit(command);
  public function stop(mode:StopMode):Void adapter.stop(mode);
  public function resetSafety():Void adapter.resetSafety();
  public function setChangeListener(listener:Null < RobotId -> Void >):Void
    adapter.setChangeListener(listener);
  public function close():Void adapter.close();
}
