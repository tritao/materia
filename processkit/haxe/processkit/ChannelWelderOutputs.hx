package processkit;

import motionkit.event.EventValue;
import motionkit.program.MotionOp;
import processkit.WelderProcessDevice.WelderChannels;

/**
 * The outputs of a welder that is worked through the robot's own process channels, as the simulated one and a retrofit
 * I/O board are: a channel cannot be written from outside a motion program, so a write is held until the program that
 * runs next carries it out as `SetOutput` operations at its start (`drain`). A device that is safed while the robot
 * stops needs nothing more, since the channels go to their safe values on the stop itself (see `WeldChannelPolicy`); the
 * write is then the same value again.
 */
class ChannelWelderOutputs implements WelderOutputs {
  public final channels:WelderChannels;
  var arc:Null<Bool> = null;
  var wireSpeed:Null<Float> = null;
  var voltage:Null<Float> = null;

  public function new(channels:WelderChannels) {
    if (channels == null) throw "Channel welder outputs need the welder's channels";
    this.channels = channels;
  }

  public function setArc(on:Bool):Void arc = on;
  public function setWireSpeed(metresPerMinute:Float):Void wireSpeed = metresPerMinute;
  public function setVoltage(volts:Float):Void voltage = volts;

  /** The writes held since the last call, as the operations that carry them out. */
  public function drain():Array<MotionOp> {
    // The wire stops before the arc goes off (burnback), and the arc comes on after the wire is set.
    var ops:Array<MotionOp> = [];
    var on = arc, speed = wireSpeed, volts = voltage;
    if (volts != null) ops.push(MotionOp.SetOutput(channels.voltage, EventValue.Analog(volts)));
    if (speed != null) ops.push(MotionOp.SetOutput(channels.wireSpeed, EventValue.Analog(speed)));
    if (on != null) ops.push(MotionOp.SetOutput(channels.arc, EventValue.Digital(on)));
    arc = null;
    wireSpeed = null;
    voltage = null;
    return ops;
  }
}
