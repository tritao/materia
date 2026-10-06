import processkit.testing.FakeModbusServer;
import processkit.testing.FakeModbusServer.FakeModbusOwner;
import nativekit.ffi.NativeKit;
import nativekit.ffi.NativeKitTypes;
import haxeon.platform.NativeKitRuntime;
import haxeon.platform.NativeKitEventValue;
import haxeon.platform.NativeKitEventBytes;
import robotkit.transport.NativeTransport;
import processkit.modbus.ModbusTcpClient;
import processkit.modbus.ModbusWelder;
import processkit.modbus.ModbusWelderMap;
import processkit.modbus.ModbusRegister;
import processkit.WelderDeviceOwner;

class WelderOwnerTests {
  static function clock():Float return haxe.Int64.toFloat(NativeKit.nk_time_now_ns()) * 1e-9;
  static function await(test:Void->Bool, message:String):Void {
    var deadline = clock() + 3.0;
    while (!test() && clock() < deadline) Sys.sleep(0.002);
    if (!test()) throw message;
  }
  public static function run():Void {
    var native = NativeKitRuntime.start();
    var listener = NativeTransport.listen(36279);
    var accepted:Array<TransportHandle> = [];
    var subscription = native.events.listen(function(event:NativeKitEventValue) {
      switch event {
        case Raw(kind, source, _, _, _, _, data):
          if (kind == EventKind.TransportAccepted && source.rawValue() == listener.borrow().rawValue())
            accepted.push(new TransportHandle(NativeKitEventBytes.readU32(data, 4)));
        case _: return;
      }
    });
    var client = new ModbusTcpClient("127.0.0.1", 36279, 1, 0.3);
    var deadline = clock() + 3.0;
    while (accepted.length == 0 && clock() < deadline) { native.events.poll(); native.events.wait(0.001); }
    if (accepted.length == 0) throw "Welder owner accept timed out";
    var map = new ModbusWelderMap(1, 100, 1, new ModbusRegister(101, 0.01), new ModbusRegister(102, 0.01),
      200, 1, 2, 4, new ModbusRegister(201, 0.1), new ModbusRegister(202, 0.01), new ModbusRegister(203), 103, 1000);
    var server = new FakeModbusServer(map, accepted[0]); server.modelEnabled = true;
    var hardware = new FakeModbusOwner(server, clock);
    var owner = new WelderDeviceOwner(new ModbusWelder(client, map), clock);
    var failure:Null<String> = null;
    try {
      await(owner.ready, "Owner did not initialize");
      var rejected = false; try owner.setArc(true) catch (_:Dynamic) rejected = true;
      if (!rejected) throw "Owner ignited without explicit reset";
      owner.reset(); owner.setVoltage(24); owner.setWireSpeed(8); owner.setArc(true);
      await(function() return owner.reading().arc, "Owner did not establish arc");
      // The planning caller does no work for longer than both response timeout and lease.
      Sys.sleep(1.5);
      if (!hardware.arc() || hardware.wire() != 8 || owner.faultDetail() != null)
        throw "Planner stall starved device polling or watchdog renewal";
      var snapshot = owner.reading(); snapshot.arc = false;
      if (!owner.reading().arc) throw "Feedback snapshot aliases owner state";
      owner.safe(); owner.setWireSpeed(8);
      rejected = false; try owner.setArc(true) catch (_:Dynamic) rejected = true;
      if (!rejected) throw "Stop did not invalidate ignition";
      await(function() return !hardware.arc() && hardware.wire() == 0 && !owner.reading().arc,
        "Stop did not turn the device off");
      owner.reset(); Sys.sleep(0.2);
      if (hardware.arc()) throw "Reset replayed an old ignition";
      owner.setVoltage(24); owner.setWireSpeed(8); owner.setArc(true);
      await(function() return owner.reading().arc, "Explicit restart failed");
      hardware.supplyFault(true);
      await(function() return owner.faultDetail() != null && !hardware.arc(), "Supply fault did not inhibit owner");
      hardware.supplyFault(false);
      Sys.sleep(0.2);
      if (hardware.arc()) throw "Supply fault clearance replayed ignition";
      if (owner.reading().fault == 0) throw "Fault clearance removed the owner latch without reset";
      await(function() { try { owner.reset(); return true; } catch (_:Dynamic) return false; },
        "Explicit reset after healthy supply feedback failed");
      owner.setWireSpeed(8); owner.setArc(true);
      await(function() return owner.reading().arc, "Reset after supply fault failed");
      hardware.drop();
      await(function() return owner.faultDetail() != null, "Link loss did not fault owner");
      await(function() return !hardware.arc() && hardware.wire() == 0, "Independent device watchdog did not expire");
      trace("Welder owner: planner stall, isolated snapshots, stop priority, explicit reset, supply fault and link loss passed");
    } catch (error:Dynamic) failure = Std.string(error);
    owner.close(); hardware.close(); subscription.dispose(); listener.close(); native.dispose();
    if (failure != null) throw failure;
  }
}
