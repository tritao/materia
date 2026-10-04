import nativekit.ffi.NativeKitTypes;
import NativeKitRuntime;
import NativeKitEventValue;
import NativeKitEventBytes;
import robotkit.transport.NativeTransport;
import processkit.modbus.ModbusTcpClient;
import processkit.modbus.ModbusWelder;
import processkit.modbus.ModbusWelderMap;
import processkit.modbus.ModbusRegister;

class ModbusTcpTests {
  public static function run():Void {
    var runtime = NativeKitRuntime.start();
    var listener = NativeTransport.listen(36277);
    var accepted:Array<TransportHandle> = [];
    var subscription = runtime.events.listen(function(event:NativeKitEventValue) {
      switch event {
        case Raw(kind, source, _, _, _, _, data):
          if (kind == EventKind.TransportAccepted && source.rawValue() == listener.borrow().rawValue())
            accepted.push(new TransportHandle(NativeKitEventBytes.readU32(data, 4)));
        case _: return;
      }
    });
    var client = new ModbusTcpClient("127.0.0.1", 36277, 1, 0.3);
    var deadline = Sys.time() + 2.0;
    while (accepted.length == 0 && Sys.time() < deadline) {
      runtime.events.poll(); runtime.events.wait(0.001);
    }
    if (accepted.length == 0) throw "Modbus fake accept timed out";
    var map = new ModbusWelderMap(1, 100, 1, new ModbusRegister(101, 0.01), new ModbusRegister(102, 0.01),
      200, 1, 2, 4, new ModbusRegister(201, 0.1), new ModbusRegister(202, 0.01), new ModbusRegister(203), 103, 1000);
    var fake = new FakeModbusServer(map, accepted[0]);
    var failure:Dynamic = null;
    var stage = "writes";
    try {
      var done:Array<Int> = [];
      client.write(map.leaseAddress, map.leaseMs, function() done.push(1));
      client.write(map.wire.address, map.wire.encode(8), function() done.push(2));
      client.write(map.voltage.address, map.voltage.encode(24), function() done.push(3));
      client.write(map.arcAddress, map.arcMask, function() done.push(4));
      deadline = Sys.time() + 2.0;
      while (done.length < 4 && Sys.time() < deadline) {
        runtime.events.poll(); fake.poll(Sys.time()); client.poll(Sys.time()); runtime.events.wait(0.001);
      }
      if (done.length != 4 || client.fault != null || !fake.arc || fake.wire != 8)
        throw 'Modbus TCP outputs failed: ${client.fault}';
      if (fake.register(map.voltage.address) != 2400) throw "Modbus voltage scale failed";
      stage = "feedback";
      fake.setRegister(map.statusAddress, map.establishedMask | map.faultMask);
      var feedback:Array<Int> = [];
      client.read(map.statusAddress, 1, function(values:Array<Int>) feedback.push(values[0]));
      deadline = Sys.time() + 2.0;
      while (feedback.length == 0 && Sys.time() < deadline) {
        runtime.events.poll(); fake.poll(Sys.time()); client.poll(Sys.time()); runtime.events.wait(0.001);
      }
      if (feedback.length != 1 || feedback[0] != (map.establishedMask | map.faultMask)) throw "Modbus feedback bits failed";
      stage = "drop";
      stage = "welder adapter";
      fake.modelEnabled = true;
      var welder = new ModbusWelder(client, map);
      deadline = Sys.time() + 3.0;
      while (!welder.initialized && Sys.time() < deadline) {
        runtime.events.poll(); fake.poll(Sys.time()); welder.poll(Sys.time()); runtime.events.wait(0.001);
      }
      if (!welder.initialized || welder.reading().arc) throw "Modbus welder did not initialize safely";
      welder.setVoltage(24); welder.setWireSpeed(8); welder.setArc(true);
      deadline = Sys.time() + 3.0;
      while (!welder.reading().arc && Sys.time() < deadline) {
        runtime.events.poll(); fake.poll(Sys.time()); welder.poll(Sys.time()); runtime.events.wait(0.001);
      }
      if (!fake.arc || welder.reading().currentA != 240 || welder.reading().voltageV != 24)
        throw 'Modbus welder feedback failed: ${welder.faultMessage}';
      welder.safe();
      deadline = Sys.time() + 3.0;
      while ((fake.arc || !client.idle()) && Sys.time() < deadline) {
        runtime.events.poll(); fake.poll(Sys.time()); welder.poll(Sys.time()); runtime.events.wait(0.001);
      }
      if (fake.arc || fake.wire != 0) throw "Modbus welder safe did not stop the source";
      welder.setArc(true); welder.setWireSpeed(8);
      deadline = Sys.time() + 3.0;
      while (!fake.arc && Sys.time() < deadline) {
        runtime.events.poll(); fake.poll(Sys.time()); welder.poll(Sys.time()); runtime.events.wait(0.001);
      }
      if (!fake.arc) throw "Modbus restart after safe failed";
      stage = "adapter supply fault";
      fake.supplyFault = true;
      deadline = Sys.time() + 3.0;
      while (welder.reading().fault == 0 && Sys.time() < deadline) {
        runtime.events.poll(); fake.poll(Sys.time()); welder.poll(Sys.time()); runtime.events.wait(0.001);
      }
      if (welder.reading().fault != processkit.tool.WeldFault.SupplyFault || fake.arc)
        throw "Modbus supply fault did not safe the process interface";
      fake.supplyFault = false;
      deadline = Sys.time() + 3.0;
      while (welder.reading().fault != 0 && Sys.time() < deadline) {
        runtime.events.poll(); fake.poll(Sys.time()); welder.poll(Sys.time()); runtime.events.wait(0.001);
      }
      if (fake.arc) throw "Clearing a supply fault reignited the arc";
      welder.setWireSpeed(8); welder.setArc(true);
      deadline = Sys.time() + 3.0;
      while (!fake.arc && Sys.time() < deadline) {
        runtime.events.poll(); fake.poll(Sys.time()); welder.poll(Sys.time()); runtime.events.wait(0.001);
      }
      if (!fake.arc) throw "Explicit restart after supply fault failed";
      stage = "adapter connection drop";
      client.read(map.current.address, 1, function(_:Array<Int>) {});
      client.poll(Sys.time()); fake.drop();
      deadline = Sys.time() + 1.1;
      while (Sys.time() < deadline) {
        runtime.events.poll(); fake.poll(Sys.time()); welder.poll(Sys.time()); runtime.events.wait(0.001);
      }
      if (client.fault == null) throw "Modbus connection drop did not fault the client";
      if (welder.reading().fault == 0) throw "Modbus link loss did not fault the welder process interface";
      if (fake.arc || fake.wire != 0.0) throw "Modbus device lease failed to safe disconnected outputs";
      trace("Modbus TCP fake: writes, feedback bits, connection fault and independent watchdog passed");
    } catch (error:Dynamic) failure = 'Modbus TCP $stage: client=${client.fault}, sent=${client.sentBytes}, requests=${fake.requests}, bytes=${fake.received}, error=$error';
    client.close(); fake.drop(); subscription.dispose(); listener.close(); runtime.dispose();
    if (failure != null) throw failure;
  }
}
