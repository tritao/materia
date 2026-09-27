package robotkit.world;

import robotkit.model.RobotModel;
import robotkit.runtime.RobotRuntime;
import robotkit.runtime.RobotRuntimeBlueprint;
import robotkit.runtime.RobotRuntimeCompiler;
import robotkit.deployment.SerialDeployment;

/** RobotWorld adapter that owns and starts a model-configured serial runtime. */
class SerialRobot implements Robot {
  final adapter:RuntimeRobotAdapter;

  public function new(id:RobotId, model:RobotModel, devicePath:String,
      fingerprintHex:String, maxTargetError:Float, ?baud:Int = 115200,
      ?ownerPeriodNs:haxe.Int64, ?processingAllowanceNs:haxe.Int64,
      ?channels:Array<ProcessChannelDeclaration>, ?stepTickHz:Int = 40000,
      ?linkLossTimeoutNs:haxe.Int64, ?clockSyncBoundNs:haxe.Int64) {
    if (id == null || id.length == 0)
      throw "SerialRobot requires a non-empty logical ID";
    if (model == null) throw "SerialRobot requires a robot model";
    var blueprint:RobotRuntimeBlueprint = RobotRuntimeCompiler.compile(model);
    if (channels != null)
      for (channel in channels) blueprint.channels.push(channel);
    if (ownerPeriodNs != null) blueprint.ownerPeriodNs = ownerPeriodNs;
    if (processingAllowanceNs != null)
      blueprint.serialProcessingAllowanceNs = processingAllowanceNs;
    var runtime = RobotRuntime.createSerial(blueprint, devicePath, fingerprintHex,
      maxTargetError, baud, stepTickHz, linkLossTimeoutNs, clockSyncBoundNs);
    adapter = new RuntimeRobotAdapter(id, runtime, model.name,
      [for (link in model.links) link.id], [for (joint in model.joints) joint.id],
      true, true, "serial endpoint fault");
  }

  /** Opens a serial robot from the same versioned deployment as robotd. */
  public static function fromDeployment(id:RobotId, path:String):SerialRobot {
    var deployment = new SerialDeployment(path);
    return new SerialRobot(id, deployment.robot, deployment.serialPath,
      deployment.fingerprint, deployment.targetError, deployment.baud,
      deployment.ownerPeriodNs, deployment.processingAllowanceNs, deployment.channels,
      deployment.stepTickHz, deployment.linkLossTimeoutNs, deployment.clockSyncBoundNs);
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
