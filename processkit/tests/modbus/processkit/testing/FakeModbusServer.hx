package processkit.testing;

import sys.thread.Mutex;
import sys.thread.Lock;
import sys.thread.Thread;

import haxe.io.Bytes;
import nativekit.ffi.NativeKitTypes;
import robotkit.transport.NativeTransport;
import processkit.modbus.ModbusFrame;
import processkit.modbus.ModbusFrameStream;
import processkit.modbus.ModbusWelderMap;

/** In-process fake on a real TCP socket; watchdog clock advances even after the socket drops. */
class FakeModbusServer {
  /** Electrical contact supplied by the CAD harness; the server owns the arc and watchdog. */
  public var grounded:Bool = true;
  public final map:ModbusWelderMap;
  final transport:TransportHandle;
  final stream = new ModbusFrameStream();
  final registers:Array<Int> = [];
  public var requests(default, null) = 0;
  public var received(default, null) = 0;
  public var modelEnabled = false;
  public var supplyFault = false;
  public var arc(default, null) = false;
  public var wire(default, null) = 0.0;
  var expires = -1.0;
  var disconnected = false;
  public function new(map:ModbusWelderMap, transport:TransportHandle) {
    this.map = map; this.transport = transport;
    for (_ in 0...65536) registers.push(0);
  }
  public function register(address:Int):Int return registers[address];
  public function setRegister(address:Int, value:Int):Void registers[address] = value;
  public function drop():Void { if (!disconnected) NativeTransport.close(transport); disconnected = true; }
  public function poll(now:Float):Void {
    if (expires >= 0.0 && now >= expires) {
      registers[map.arcAddress] = 0; registers[map.wire.address] = 0;
    }
    updateModel(now);
    if (!disconnected) {
      var bytes = NativeTransport.receive(transport, 2048);
      received += bytes.length;
      if (bytes.length > 0) stream.feed(bytes);
      while (true) {
        var request = stream.next(); if (request == null) break;
        if (request.unit != map.unit || request.pdu.length < 5) throw "Invalid fake Modbus request";
        requests++;
        var address = ModbusFrame.word(request.pdu, 1), argument = ModbusFrame.word(request.pdu, 3);
        var reply:Bytes;
        if (request.pdu.get(0) == 6) {
          registers[address] = argument;
          if (address == map.leaseAddress) expires = now + argument / 1000.0;
          reply = request.pdu;
        } else if (request.pdu.get(0) == 16 && argument >= 1 && argument <= 123 &&
            address + argument <= 65536 && request.pdu.length == 6 + argument * 2 && request.pdu.get(5) == argument * 2) {
          for (i in 0...argument) registers[address + i] = ModbusFrame.word(request.pdu, 6 + i * 2);
          if (map.leaseAddress >= address && map.leaseAddress < address + argument)
            expires = now + registers[map.leaseAddress] / 1000.0;
          reply = Bytes.alloc(5); reply.set(0, 16);
          ModbusFrame.putWord(reply, 1, address); ModbusFrame.putWord(reply, 3, argument);
        } else if (request.pdu.get(0) == 3 && argument >= 1 && argument <= 125 && address + argument <= 65536) {
          reply = Bytes.alloc(2 + argument * 2); reply.set(0, 3); reply.set(1, argument * 2);
          for (i in 0...argument) ModbusFrame.putWord(reply, 2 + i * 2, registers[address + i]);
        } else throw "Unsupported fake Modbus function";
        NativeTransport.send(transport, new ModbusFrame(request.transaction, map.unit, reply).encode());
      }
    }
    updateModel(now);
  }
  function updateModel(now:Float):Void {
    if (supplyFault) {
      registers[map.arcAddress] = 0; registers[map.wire.address] = 0; expires = -1.0;
    }
    arc = expires > now && !supplyFault && (registers[map.arcAddress] & map.arcMask) != 0;
    wire = arc ? map.wire.decode(registers[map.wire.address]) : 0.0;
    if (modelEnabled) {
      var established = arc && grounded;
      var current = established ? wire * 30.0 : 0.0;
      var voltage = established ? map.voltage.decode(registers[map.voltage.address]) : 0.0;
      registers[map.statusAddress] = (established ? map.establishedMask : 0) | (supplyFault ? map.faultMask : 0);
      registers[map.current.address] = map.current.encode(current);
      registers[map.measuredVoltage.address] = map.measuredVoltage.encode(voltage);
      registers[map.power.address] = map.power.encode(current * voltage / 0.9);
    }
  }
}

/** Independent fake hardware clock, including while its host is compiling or planning. */
class FakeModbusOwner {
  final server:FakeModbusServer;
  final clock:Void->Float;
  final mutex = new Mutex();
  final ended = new Lock();
  var closing = false;
  var failure:Null<String> = null;
  public function new(server:FakeModbusServer, clock:Void->Float) {
    this.server = server; this.clock = clock; Thread.create(run);
  }
  public function grounded(value:Bool):Void { mutex.acquire(); server.grounded = value; mutex.release(); }
  public function supplyFault(value:Bool):Void { mutex.acquire(); server.supplyFault = value; mutex.release(); }
  public function drop():Void { mutex.acquire(); server.drop(); mutex.release(); }
  public function arc():Bool { mutex.acquire(); var value = server.arc; mutex.release(); return value; }
  public function wire():Float { mutex.acquire(); var value = server.wire; mutex.release(); return value; }
  public function fault():Null<String> { mutex.acquire(); var value = failure; mutex.release(); return value; }
  public function close():Void { mutex.acquire(); closing = true; mutex.release(); ended.wait(); }
  function run():Void {
    while (true) {
      mutex.acquire();
      if (closing) { server.drop(); mutex.release(); break; }
      try server.poll(clock()) catch (error:Dynamic) { failure = Std.string(error); server.drop(); }
      mutex.release(); Sys.sleep(0.002);
    }
    ended.release();
  }
}
