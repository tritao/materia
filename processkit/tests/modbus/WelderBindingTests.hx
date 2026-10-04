import processkit.WelderChannelBinding;
import processkit.WelderOutputs;
import processkit.WelderFeedback;
import processkit.tool.WeldSensor.WeldReading;
import robotkit.execution.ProcessEventValue;

private class Supply implements WelderOutputs implements WelderFeedback {
  public var arcs:Array<Bool> = [];
  public var wires:Array<Float> = [];
  public var voltages:Array<Float> = [];
  public var polls = 0;
  public var fault = 0;
  public function new() {}
  public function setArc(v:Bool):Void arcs.push(v);
  public function setWireSpeed(v:Float):Void wires.push(v);
  public function setVoltage(v:Float):Void voltages.push(v);
  public function reading():WeldReading return {arc:false,currentA:0.0,voltageV:0.0,touch:false,fault:fault,powerW:0.0};
  public function poll(_:Float):Void polls++;
}
class WelderBindingTests {
  public static function run():Void {
    var supply = new Supply();
    var binding = new WelderChannelBinding({arc:"torch",wireSpeed:"wire",voltage:"voltage"}, supply, supply, supply.poll);
    var values:Array<ProcessEventValue> = [Digital(false), Analog(8.0), Analog(24.0)];
    var read = function(channel:String):ProcessEventValue return values[channel == "torch" ? 0 : channel == "wire" ? 1 : 2];
    binding.update(0.0, read); binding.update(0.01, read);
    if (supply.arcs.length != 1 || supply.wires.length != 1 || supply.voltages.length != 1 || supply.polls != 2)
      throw "Unchanged process channels rewrote device outputs or skipped polling";
    values[0] = Digital(true); binding.update(0.02, read);
    values[1] = Analog(6.0); binding.update(0.03, read);
    if (supply.arcs.length != 2 || !supply.arcs[1] || supply.wires[1] != 6.0)
      throw "Process channel transition did not reach the supply";
    binding.safe();
    if (supply.arcs[supply.arcs.length - 1] || supply.wires[supply.wires.length - 1] != 0.0)
      throw "Binding safe failed to stop arc and wire";
    binding.update(0.04, read);
    if (supply.arcs[supply.arcs.length - 1] || supply.wires[supply.wires.length - 1] != 0.0)
      throw "A stale on channel reignited after binding safe";
    values[0] = Digital(false); values[1] = Analog(-1.0);
    var rejected = false;
    try binding.update(0.05, read) catch (_:Dynamic) rejected = true;
    if (!rejected) throw "Binding accepted an invalid process setpoint";
    runtimePublication();
    trace("Welder channel binding: transitions, polling, safe outputs and invalid setpoints passed");
  }

  static function runtimePublication():Void {
    var blueprint = new robotkit.runtime.RobotRuntimeBlueprint(1, 1, 2);
    blueprint.addJoint(new robotkit.runtime.RobotRuntimeJointBlueprint(0,
      RobotKitRuntime.RobotKitRuntimeConstants.RK_RUNTIME_JOINT_REVOLUTE, 0, 1, -1.0, 1.0, 1.0));
    for (channel in processkit.tool.WeldContract.declarations("torch", "wire", "voltage"))
      blueprint.channels.push(channel);
    blueprint.sensors.push(new robotkit.runtime.RobotRuntimeSensorBlueprint("weld", "tool_weld", "tip", "torch_link",
      1, [0.01, 0.02, 0.03], [0.0, 0.0, 0.0, 1.0]));
    var runtime = robotkit.runtime.RobotRuntime.create(blueprint,
      robotkit.runtime.RuntimeEndpoints.inMemory(blueprint));
    var supply = new Supply();
    var binding = new WelderChannelBinding({arc:"torch",wireSpeed:"wire",voltage:"voltage"}, supply, supply, supply.poll);
    var bridge = new processkit.WelderRuntimeBinding(runtime, binding, "weld", "test.supply");
    var frame = bridge.update(0.0, haxe.Int64.ofInt(100));
    if (frame.kind != "tool_weld" || frame.frameId != "tip" || frame.linkId != "torch_link" ||
        frame.values.length != 6 || frame.mountPosition.get(0) != 0.01 || frame.sourceClockId != "test.supply")
      throw "Supply publication lost authored welding sensor metadata";
    supply.fault = 2;
    frame = bridge.update(0.01, haxe.Int64.ofInt(200));
    if (frame.values.get(4) != 2.0 || haxe.Int64.toInt(frame.sequence) != 2 ||
        supply.arcs[supply.arcs.length - 1] || supply.wires[supply.wires.length - 1] != 0.0)
      throw "Supply fault did not publish or safe outputs";
    var rejected = false;
    try bridge.update(0.02, haxe.Int64.ofInt(50)) catch (_:Dynamic) rejected = true;
    if (!rejected) throw "Supply bridge accepted a backward source clock";
    runtime.dispose();
  }
}
