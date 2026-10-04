package processkit.simulation;

import haxe.Int64;
import processkit.modbus.ModbusWelder;
import processkit.WelderChannelBinding;
import processkit.WelderRuntimeBinding;
import processkit.WelderProcessDevice.WelderChannels;
import processkit.tool.WeldSensor;
import processkit.tool.WeldSensor.WeldReading;
import robotkit.runtime.RobotRuntime;

/** The CAD mission drives the real TCP adapter; the fake, when used, owns its independent watchdog. */
class ModbusWelderSupply implements SimulationWelderSupply {
  public final welder:ModbusWelder;
  final bridge:WelderRuntimeBinding;
  final clock:Void->Float;
  final pollOwner:Bool->Void;

  public function new(welder:ModbusWelder, runtime:RobotRuntime, channels:WelderChannels, sensorId:String,
      clock:Void->Float, pollOwner:Bool->Void) {
    this.welder = welder; this.clock = clock; this.pollOwner = pollOwner;
    bridge = new WelderRuntimeBinding(runtime,
      new WelderChannelBinding(channels, welder, welder, welder.poll), sensorId, "robotkit.simulation");
  }
  public function observe(dt:Float, timestamp:Int64, grounded:Bool):WeldReading {
    pollOwner(grounded);
    var frame = bridge.update(clock(), timestamp);
    return WeldSensor.reading(frame.values.toArray());
  }
  public function safe():Void {
    bridge.safe(); welder.poll(clock());
  }
  public function reset():Void safe();
}
