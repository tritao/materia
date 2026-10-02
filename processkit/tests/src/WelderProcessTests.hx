import haxe.Int64;
import processkit.WelderFeedback;
import processkit.WelderOutputs;
import processkit.WelderProcessDevice;
import robotkit.tool.WeldArcModel;
import robotkit.tool.WeldSensor.WeldReading;
import robotkit.world.FiredProcessEvent;
import robotkit.world.ProcessEventValue;

/**
 * A welder behind the outputs and feedback a process device speaks: RobotKit's arc model stands in for the supply,
 * the way a retrofit I/O board or a Modbus supply would, with the tip `height` metres above the work.
 */
class ModelWelder implements WelderOutputs implements WelderFeedback {
  public final model = new WeldArcModel({maxCurrentA: 350, efficiency: 0.88, wireDiameterMm: 1.2, stickoutMm: 15});
  public var height = 0.002;
  public var arc = false;
  public var wireSpeed = 0.0;
  public var voltage = 0.0;
  public var log:Array<String> = [];

  public function new() {}

  public function setArc(on:Bool):Void {
    arc = on;
    log.push("arc " + on);
  }

  public function setWireSpeed(metresPerMinute:Float):Void {
    wireSpeed = metresPerMinute;
    log.push("wire " + metresPerMinute);
  }

  public function setVoltage(volts:Float):Void {
    voltage = volts;
    log.push("voltage " + volts);
  }

  public function reading():WeldReading return model.reading;

  /** Lets `seconds` pass over a flat work whose top is the plane z = 0. */
  public function elapse(seconds:Float):Void {
    for (i in 0...Math.round(seconds / 0.01))
      model.step(0.01, {arcCommanded: arc, wireSpeed: wireSpeed, voltageSet: voltage, supplyReady: true,
        tipDistance: height, wireDistance: height});
  }
}

class WelderProcessTests {
  static var assertions = 0;

  static function event(channel:String, value:ProcessEventValue):FiredProcessEvent
    return new FiredProcessEvent(Int64.ofInt(1), channel, value, Int64.ofInt(100), Int64.ofInt(100), 1);

  public static function run():Int {
    assertions = 0;
    var supply = new ModelWelder();
    var channels = {arc: "tool/torch.arc", wireSpeed: "tool/torch.wire_speed", voltage: "tool/torch.voltage"};
    var device = new WelderProcessDevice(supply, supply, channels, {voltage: 24.0});

    check(!device.ready(), "a device that was not prepared is not ready");
    check(device.fault() == null, "and has no fault");
    device.prepare();
    check(device.ready() && supply.voltage == 24.0 && supply.wireSpeed == 0.0 && !supply.arc,
      "prepare sets the voltage and leaves the arc off and the wire still");

    // Fired records work the three outputs: a process run emits the arc as an analog rate.
    device.apply([event(channels.voltage, Analog(26.0)), event(channels.wireSpeed, Analog(8.0)), event(channels.arc, Analog(1.0))]);
    check(supply.voltage == 26.0 && supply.wireSpeed == 8.0 && supply.arc, "the records set voltage, wire speed and arc");
    supply.elapse(0.3);
    check(supply.reading().arc && supply.reading().currentA > 200 && device.ready() && device.fault() == null,
      "the arc burns and the device stays ready");
    device.apply([event(channels.arc, Digital(false))]);
    check(!supply.arc, "a digital off record puts the arc out");
    device.apply([event(channels.arc, Analog(0.0))]);
    check(!supply.arc, "so does an analog rate of zero");

    // Safe: the arc goes first, then the wire.
    device.apply([event(channels.arc, Digital(true)), event(channels.wireSpeed, Analog(8.0))]);
    supply.log = [];
    device.safe();
    check(supply.log.join(",") == "arc false,wire 0" && !supply.arc && supply.wireSpeed == 0.0,
      "safe switches the arc off, then stops the wire: " + supply.log);

    // A fault from the supply is the device's fault and stops it being ready.
    var open = new ModelWelder();
    open.height = 0.05;
    var failing = new WelderProcessDevice(open, open, channels, {voltage: 24.0});
    failing.prepare();
    failing.apply([event(channels.wireSpeed, Analog(8.0)), event(channels.arc, Digital(true))]);
    open.elapse(1.2);
    check(failing.fault() == "weld: the arc did not ignite" && !failing.ready(), "no arc is the device's fault: " + failing.fault());
    failing.safe();
    open.elapse(0.05);
    check(failing.fault() == null, "the fault clears once the arc is off");
    check(!failing.ready(), "but a device made safe has to be prepared again before it is ready");
    failing.prepare();
    check(failing.ready(), "and prepare makes it ready");

    // Refusals.
    expectFailure(function() device.apply([event("other", Analog(1.0))]), "a record on another channel is refused");
    expectFailure(function() device.apply([event(channels.wireSpeed, Digital(true))]), "a wire speed needs an analog value");
    expectFailure(function() device.apply([event(channels.arc, Process("go", 1.0))]), "the arc takes no process command");
    expectFailure(function() new WelderProcessDevice(supply, supply, {arc: "a", wireSpeed: "a", voltage: "v"}, {voltage: 24.0}),
      "channels must be distinct");
    expectFailure(function() new WelderProcessDevice(supply, supply, channels, {voltage: 0.0}), "a setpoint voltage must be positive");
    Sys.println('ProcessKit welder tests passed ($assertions assertions)');
    return assertions;
  }

  static function expectFailure(action:Void -> Void, message:String):Void {
    var failed = false;
    try action() catch (_:Dynamic) failed = true;
    check(failed, message);
  }

  static function check(value:Bool, message:String):Void {
    assertions++;
    if (!value) throw message;
  }
}
