import processkit.modbus.ModbusFrame;
import processkit.modbus.ModbusRegister;
import processkit.modbus.ModbusWelderMap;
import haxe.io.Bytes;

class ModbusTests {
  static var count = 0;
  static function check(ok:Bool):Void { count++; if (!ok) throw "Modbus assertion failed"; }
  static function rejects(f:Void->Void):Void {
    var rejected = false; try f() catch (_:Dynamic) rejected = true; check(rejected);
  }
  public static function main():Void {
    var r = new ModbusRegister(123, 0.01);
    check(r.encode(8.0) == 800); check(r.decode(2400) == 24.0);
    rejects(function() r.encode(-1.0)); rejects(function() r.encode(1000));
    rejects(function() new ModbusRegister(65536)); rejects(function() new ModbusRegister(1, 0));
    for (id in [0, 1, 65535]) {
      var pdu = ModbusFrame.writeHolding(123, 800);
      var bytes = new ModbusFrame(id, 1, pdu).encode();
      check(bytes.length == 12); check(bytes.get(7) == 6);
      var frame = ModbusFrame.decode(bytes); check(frame.transaction == id); check(frame.unit == 1);
      check(ModbusFrame.word(frame.pdu, 1) == 123); check(ModbusFrame.word(frame.pdu, 3) == 800);
      bytes.set(2, 1); rejects(function() ModbusFrame.decode(bytes));
    }
    var read = ModbusFrame.readHolding(65535, 1); check(ModbusFrame.word(read, 1) == 65535);
    rejects(function() ModbusFrame.readHolding(65535, 2)); rejects(function() ModbusFrame.readHolding(0, 126));
    rejects(function() ModbusFrame.decode(Bytes.alloc(7)));
    rejects(function() r.encode(1e30));
    var map = new ModbusWelderMap(1, 100, 1, new ModbusRegister(101, 0.01), new ModbusRegister(102, 0.01),
      200, 1, 2, 4, new ModbusRegister(201, 0.1), new ModbusRegister(202, 0.01),
      new ModbusRegister(203), 103, 200);
    check(map.wire.encode(8) == 800); check(map.current.decode(2400) == 240);
    check(map.leaseMs == 200);
    rejects(function() new ModbusWelderMap(1, 100, 1, map.wire, map.voltage,
      200, 1, 1, 4, map.current, map.measuredVoltage, map.power, 103, 200));
    rejects(function() new ModbusWelderMap(1, 100, 1, map.wire, map.voltage,
      200, 1, 2, 4, map.current, map.measuredVoltage, map.power, 103, 5000));
    trace('Modbus tests passed ($count assertions)');
  }
}
