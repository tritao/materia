package app;

import sys.FileSystem;
import app.MateriaProjectRunner.GeneratedAssemblyScene;
import robotkit.world.RobotWorld;
import nativekit.ffi.NativeKit;
import nativekit.ffi.NativeKitTypes;
import NativeKitRuntime;
import NativeKitEventValue;
import NativeKitEventBytes;
import robotkit.transport.NativeTransport;
import processkit.modbus.ModbusTcpClient;
import processkit.modbus.ModbusWelder;
import processkit.modbus.ModbusWelderMap;
import processkit.modbus.ModbusRegister;
import processkit.simulation.ModbusWelderSupply;
import processkit.testing.FakeModbusServer;

/** Focused integration gate: an unchanged CAD weld mission with device feedback driving its bead. */
class WelderDeviceTests {
  public static function main():Void run(FileSystem.fullPath("."));
  public static function run(root:String):Void {
    var selected = Sys.getEnv("WELDER_DEVICE_ONLY");
    if (selected != "modbus") runMission(root, true, null);
    if (selected != "rkd6") runModbus(root);

  }
  static function clock():Float return haxe.Int64.toFloat(NativeKit.nk_time_now_ns()) * 1.0e-9;

  static function runModbus(root:String):Void {
    // CAD compilation can take longer than the device's response deadline.
    // Establish the live connection only after the static artifact is available.
    var manifest = FileSystem.fullPath(root + "/machinekit/examples/robot-welder/materia.seam.project.json");
    var generated = MateriaProjectRunner.loadProject(manifest);
    var native = NativeKitRuntime.start();
    var listener = NativeTransport.listen(36278);
    var accepted:Array<TransportHandle> = [];
    var subscription = native.events.listen(function(event:NativeKitEventValue) {
      switch event {
        case Raw(kind, source, _, _, _, _, data):
          if (kind == EventKind.TransportAccepted && source.rawValue() == listener.borrow().rawValue())
            accepted.push(new TransportHandle(NativeKitEventBytes.readU32(data, 4)));
        case _: return;
      }
    });
    var client = new ModbusTcpClient("127.0.0.1", 36278, 1, 0.3);
    var deadline = clock() + 3.0;
    while (accepted.length == 0 && clock() < deadline) { native.events.poll(); native.events.wait(0.001); }
    if (accepted.length == 0) throw "CAD mission Modbus accept timed out";
    var map = new ModbusWelderMap(1, 100, 1, new ModbusRegister(101, 0.01), new ModbusRegister(102, 0.01),
      200, 1, 2, 4, new ModbusRegister(201, 0.1), new ModbusRegister(202, 0.01), new ModbusRegister(203), 103, 1000);
    var server = new FakeModbusServer(map, accepted[0]); server.modelEnabled = true;
    var supply = new ModbusWelder(client, map);
    runMission(root, false, function(welder:processkit.simulation.SimulatedWelder):processkit.simulation.SimulationWelderSupply {
      var readyUntil = clock() + 3.0;
      while (!supply.initialized && clock() < readyUntil) {
        native.events.poll(); server.poll(clock()); supply.poll(clock()); native.events.wait(0.001);
      }
      if (!supply.initialized) throw 'CAD mission Modbus source did not initialize: ${client.fault}';
      return new ModbusWelderSupply(supply, welder.runtime,
        {arc:welder.arcChannel, wireSpeed:welder.wireSpeedChannel, voltage:welder.voltageChannel}, welder.sensorId,
        clock, function(grounded:Bool) {
          server.grounded = grounded; native.events.poll();
          try server.poll(clock()) catch (error:Dynamic)
            throw 'CAD mission Modbus server failed: client=${client.fault}, requests=${server.requests}, error=$error';
        });
    }, generated);
    deadline = clock() + 2.0;
    while ((server.arc || !client.idle()) && clock() < deadline) {
      native.events.poll(); server.poll(clock()); supply.poll(clock()); native.events.wait(0.001);
    }
    if (server.arc || server.wire != 0.0) throw "Finished CAD mission left the Modbus source on";
    client.close(); server.drop(); subscription.dispose(); listener.close(); native.dispose();
  }

  static function runMission(root:String, virtualDevice:Bool,
      factory:Null<processkit.simulation.SimulatedWelder -> processkit.simulation.SimulationWelderSupply>,
      ?prepared:GeneratedAssemblyScene):Void {
    var manifest = FileSystem.fullPath(root + "/machinekit/examples/robot-welder/materia.seam.project.json");
    var generated = prepared == null ? MateriaProjectRunner.loadProject(manifest) : prepared;
    var session = new ProjectDocumentSession(null, false);
    session.openGeneratedProject(generated, manifest);
    var simulation = new ApplicationSimulation(new RobotWorld());
    simulation.virtualWelder = virtualDevice;
    simulation.welderSupplyFactory = factory;
    simulation.setBackend(ApplicationSimulation.MUJOCO);
    if (!simulation.rebuild(session.sensors, session.scene, session)) throw "Virtual welding cell failed: " + simulation.error;
    var mission = simulation.missionPlayer(), welder = simulation.welder(), beads = simulation.weldBeads();
    if (mission == null || welder == null || beads == null) throw "Missing virtual welding mission";
    var active = simulation.activeSession();
    if (active == null) throw "Missing virtual welding session";
    var wallStart = clock();
    while (!mission.finished && active.simulationTime() < 120.0) {
      if (!virtualDevice) {
        var until = wallStart + active.simulationTime() + simulation.timestep;
        while (clock() < until) Sys.sleep(0.001);
      }
      simulation.step();
      if (mission.failure != null) throw 'RKD6 mission failed at ${active.simulationTime()}: ${mission.failure}';
    }
    if (!mission.finished) throw "RKD6 welding mission timed out";
    var bead = beads.beadOf(0).beads[0];
    var leg = bead.meanLeg(0.3, 0.7);
    if (Math.abs(leg - 0.005) > 0.0005 || Math.abs(bead.extent() - bead.length) > 0.002 || beads.beadOf(0).gaps() != 0)
      throw 'RKD6 bead failed: leg ${leg * 1000}, length ${bead.extent() * 1000}, gaps ${beads.beadOf(0).gaps()}';
    // The program turns its channels off; TCP feedback can still be awaiting its acknowledged off write.
    if (welder.wireSpeed() != 0.0 || (virtualDevice && welder.reading().arc)) throw "Mission left welding outputs on";
    Sys.println('${virtualDevice ? "RKD6" : "Modbus"} CAD mission passed: ${active.simulationTime()} s, leg ${leg * 1000} mm, bead ${bead.extent() * 1000} mm, no gap');
    simulation.clear(); session.dispose();
  }
}
