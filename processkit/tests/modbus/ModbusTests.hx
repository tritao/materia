import processkit.modbus.ModbusFrame;
import processkit.modbus.ModbusFrameStream;
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
    var encoded = new ModbusFrame(42, 1, ModbusFrame.writeHolding(100, 1)).encode();
    for (split in 1...encoded.length) {
      var stream = new ModbusFrameStream();
      stream.feed(Bytes.view(encoded, 0, split)); check(stream.next() == null);
      stream.feed(Bytes.view(encoded, split, encoded.length - split));
      var packet = stream.next(); check(packet != null && packet.transaction == 42);
      check(stream.next() == null);
    }
    var coalesced = Bytes.alloc(encoded.length * 2);
    coalesced.blit(0, encoded, 0, encoded.length); coalesced.blit(encoded.length, encoded, 0, encoded.length);
    var stream = new ModbusFrameStream(); stream.feed(coalesced);
    check(stream.next() != null); check(stream.next() != null); check(stream.next() == null);
    var bad = Bytes.alloc(7); bad.set(5, 255);
    var invalidStream = new ModbusFrameStream(); invalidStream.feed(bad);
    rejects(function() invalidStream.next());
    check(processkit.tool.WeldSensor.valid([0, 0, 0, 0, 4, 0]));
    check(processkit.tool.WeldSensor.valid([0, 0, 0, 0, 5, 0]));
    check(processkit.tool.WeldSensor.faultMessage(4) == "weld: supply fault");
    check(processkit.tool.WeldSensor.faultMessage(5) == "weld: device connection lost");
    trace('Modbus tests passed ($count assertions)');
    ModbusTcpTests.run();
  }
}
