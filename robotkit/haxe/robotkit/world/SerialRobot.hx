package robotkit.world;

import robotkit.model.RobotModel;
import robotkit.runtime.RobotRuntime;
import robotkit.runtime.RobotRuntimeBlueprint;
import robotkit.runtime.RobotRuntimeCompiler;

/** RobotWorld adapter that owns and starts a model-configured serial runtime. */
class SerialRobot implements Robot {
  final adapter:RuntimeRobotAdapter;

  public function new(id:RobotId, model:RobotModel, devicePath:String,
      fingerprintHex:String, maxTargetError:Float, ?baud:Int = 115200) {
    if (id == null || id.length == 0)
      throw "SerialRobot requires a non-empty logical ID";
    if (model == null) throw "SerialRobot requires a robot model";
    var blueprint:RobotRuntimeBlueprint = RobotRuntimeCompiler.compile(model);
    var runtime = RobotRuntime.createSerial(blueprint, devicePath, fingerprintHex,
      maxTargetError, baud);
    adapter = new RuntimeRobotAdapter(id, runtime, model.name,
      [for (link in model.links) link.id], [for (joint in model.joints) joint.id],
      true, true, "serial endpoint fault", false);
  }

  public function id():RobotId return adapter.id();
  public function status():RobotStatus return adapter.status();
  public function description():RobotDescription return adapter.description();
  public function capabilities():RobotCapabilities return adapter.capabilities();
  public function snapshot():RobotSnapshot return adapter.snapshot();
  public function sensors():Array<SensorFrame> return adapter.sensors();
  public function fault():Null<RobotFault> return adapter.fault();
  public function submit(command:RobotCommand):Void adapter.submit(command);
  public function stop(mode:StopMode):Void adapter.stop(mode);
  public function resetSafety():Void adapter.resetSafety();
  public function setChangeListener(listener:Null < RobotId -> Void >):Void
    adapter.setChangeListener(listener);
  public function close():Void adapter.close();
}
