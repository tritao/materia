package processkit.modbus;

import processkit.WelderOutputs;
import processkit.WelderFeedback;
import processkit.tool.WeldSensor.WeldReading;
import processkit.tool.WeldSensor;
import processkit.tool.WeldFault;

/** Async supply adapter. The process owner polls it even while awaiting engagement or stopping. */
class ModbusWelder implements WelderOutputs implements WelderFeedback {
  public final client:ModbusTcpClient;
  public final map:ModbusWelderMap;
  public var faultMessage(default, null):Null<String> = null;
  public var initialized(default, null) = false;
  final outputStart:Int;
  final outputCount:Int;
  final inputStart:Int;
  final inputCount:Int;
  var arc = false;
  var wire = 0.0;
  var voltage = 0.0;
  var job = 0;
  var dirty = true;
  var nextPoll = 0.0;
  var latest:WeldReading = {arc:false, currentA:0.0, voltageV:0.0, touch:false, fault:0, powerW:0.0};

  public function new(client:ModbusTcpClient, map:ModbusWelderMap) {
    if (client == null || map == null || client.unit != map.unit) throw "Modbus welder needs matching client and map";
    this.client = client; this.map = map;
    var outputs = [map.arcAddress, map.wire.address, map.voltage.address, map.leaseAddress];
    if (map.job != null) outputs.push(map.job.address);
    var first = outputs[0], last = outputs[0];
    for (address in outputs) { first = Std.int(Math.min(first, address)); last = Std.int(Math.max(last, address)); }
    // Never overwrite an unmapped vendor register to fill a gap.
    if (last - first + 1 != outputs.length) throw "Modbus welder outputs and lease must form one atomic register block";
    outputStart = first; outputCount = outputs.length;
    var inputs = [map.statusAddress, map.current.address, map.measuredVoltage.address, map.power.address];
    first = inputs[0]; last = inputs[0];
    for (address in inputs) { first = Std.int(Math.min(first, address)); last = Std.int(Math.max(last, address)); }
    if (last - first + 1 > 125) throw "Modbus feedback must fit one coherent read";
    inputStart = first; inputCount = last - first + 1;
    if (map.leaseMs / 1000.0 <= 2.0 * client.timeout + 0.1)
      throw "Modbus device lease is too short for two bounded transactions and the owner tick";
  }
  public function reading():WeldReading return latest;
  public function setArc(on:Bool):Void {
    if (on && (faultMessage != null || !initialized || latest.fault != WeldFault.None)) throw "Faulted Modbus welder cannot ignite";
    if (!on) client.discardQueued();
    arc = on; dirty = true;
  }
  public function setWireSpeed(value:Float):Void {
    map.wire.encode(value); if (value < 0.0) throw "Negative welding wire speed";
    wire = value; dirty = true;
  }
  public function setVoltage(value:Float):Void {
    map.voltage.encode(value); if (value < 0.0) throw "Negative welding voltage";
    voltage = value; dirty = true;
  }
  public function setJob(value:Int):Void {
    var register = map.job;
    if (register == null || value < 0) throw "Modbus job is unsupported or negative";
    register.encode(value); job = value; dirty = true;
  }
  public function safe():Void { wire = 0.0; arc = false; dirty = true; client.discardQueued(); }
  public function disconnect():Void {
    safe(); client.close(); faultMessage = "Modbus welder disconnected";
    latest = {arc:false, currentA:0.0, voltageV:0.0, touch:false, fault:WeldFault.ConnectionLost, powerW:0.0};
  }
  public function poll(now:Float):Void {
    client.poll(now);
    if (client.fault != null) {
      faultMessage = client.fault;
      latest = {arc:false, currentA:0.0, voltageV:0.0, touch:false, fault:WeldFault.ConnectionLost, powerW:0.0};
      return;
    }
    if (faultMessage != null || !client.idle() || (!dirty && now < nextPoll)) return;
    dirty = false; nextPoll = now + Math.min(0.05, map.leaseMs / 4000.0);
    var values:Array<Int> = [];
    for (_ in 0...outputCount) values.push(0);
    values[map.arcAddress - outputStart] = arc ? map.arcMask : 0;
    values[map.wire.address - outputStart] = map.wire.encode(wire);
    values[map.voltage.address - outputStart] = map.voltage.encode(voltage);
    values[map.leaseAddress - outputStart] = arc ? map.leaseMs : 0;
    var jobRegister = map.job;
    if (jobRegister != null) values[jobRegister.address - outputStart] = jobRegister.encode(job);
    client.writeMultiple(outputStart, values, function() {});
    client.read(inputStart, inputCount, function(registers:Array<Int>) {
      var status = registers[map.statusAddress - inputStart];
      var reading:WeldReading = {arc:(status & map.establishedMask) != 0,
        currentA:map.current.decode(registers[map.current.address - inputStart]),
        voltageV:map.measuredVoltage.decode(registers[map.measuredVoltage.address - inputStart]),
        touch:(status & map.touchMask) != 0, fault:(status & map.faultMask) != 0 ? WeldFault.SupplyFault : WeldFault.None,
        powerW:map.power.decode(registers[map.power.address - inputStart])};
      if (!WeldSensor.valid(WeldSensor.values(reading))) throw "Invalid Modbus welding feedback";
      latest = reading;
      initialized = true;
      if (latest.fault != WeldFault.None) safe();
    });
    client.poll(now);
  }
}
