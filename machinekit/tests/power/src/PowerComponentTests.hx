import machinekit.power.BatteryPack;
import machinekit.power.BatterySimulator;
import machinekit.power.BatteryStorage;
import machinekit.power.Inverter;
import machinekit.power.IsolatedDcConverter;
import machinekit.motion.MotorDriver;
import machinekit.motion.NemaStepper;
import machinekit.motion.PowerSupply;
import machinekit.robotics.RobotFlange;
import machinekit.assembly.MachineAssembly;
import machinekit.welding.WeldingPowerSource;
import haxe.Int64;

class PowerComponentTests {
  static var checks:Int = 0;
  static function check(value:Bool, detail:String):Void {
    if (!value) throw detail;
    checks++;
  }
  static function rejects(action:Void->Void, detail:String):Void {
    var rejected = false;
    try action() catch (_:Dynamic) rejected = true;
    check(rejected, detail);
  }
  static function main():Void {
    var battery = new BatteryPack(48, 25000, 1000, 600, 300, 230, 2);
    var storage = BatteryStorage.of(battery);
    check(storage != null && storage.nominalVolts == 48 && storage.capacityWh == 25000,
      "Energy capacity must be discoverable as a CAD facet");
    var batteryProfile:BatteryStorage = cast storage;
    check(battery.outputVoltage("power2") == 48, "Every declared terminal supplies the nominal DC voltage");
    check(batteryProfile.fullVoltage > batteryProfile.nominalVolts &&
      batteryProfile.cutoffVoltage > batteryProfile.emptyVoltage && batteryProfile.internalResistanceOhms > 0,
      "Battery CAD carries a voltage curve, cutoff, and resistance for its runtime model");
    rejects(() -> { battery.outputVoltage("missing"); }, "Unknown outputs must be rejected");
    var inverter = new Inverter(48, 230, 10000, 600, 400, 300, 45);
    check(Math.abs(inverter.inputPower(9200) - 10000) < 1e-9, "Losses increase battery demand");
    rejects(() -> { inverter.inputPower(10001); }, "A continuous overload is rejected");
    rejects(() -> { inverter.inputPower(Math.NaN); }, "A nonfinite load is rejected");
    check(inverter.bridges().length == 0 && inverter.conversions().length == 1,
      "Inverter is a conversion boundary, not an electrically transparent bridge");
    var cell = new MachineAssembly();
    cell.addComponent("battery", battery);
    cell.addComponent("inverter", inverter);
    cell.addComponent("source", new WeldingPowerSource());
    cell.connectPorts("battery-inverter", "battery", "power1", "inverter", "dc");
    cell.connectPorts("inverter-welder", "inverter", "mains", "source", "mains");
    var input = cell.upstream("source", "mains");
    check(!input.external && input.port.instanceId == "battery", "The ultimate energy source is the battery");
    var path = cell.upstreamChain("source", "mains");
    check(path.indexOf("inverter/mains") >= 0 && path.indexOf("inverter/dc") >= 0,
      "Energy tracing traverses the inverter conversion");
    rejects(() -> { new BatteryPack(48, -1, 1000, 600, 300, 230); }, "Negative capacity is rejected");
    rejects(() -> { new BatteryPack(48, 25000, Math.POSITIVE_INFINITY, 600, 300, 230); }, "Infinite envelope is rejected");
    rejects(() -> { new Inverter(48, 230, 10000, 600, 400, 300, 45, 1.1); }, "Impossible efficiency is rejected");
    rejects(() -> { new BatteryStorage(48, 1000, 40, 35, 38); }, "Invalid battery voltage curves are rejected");

    var profile = new BatteryStorage(48, 100, 54.6, 40, 42, 0.02, 250, 5000, 0.95);
    var discharging = new BatterySimulator("pack", profile, 0.8);
    var loaded = discharging.advance(1000, 0, 36, Int64.ofInt(1_000_000_000));
    var initialVoltage = profile.emptyVoltage + (profile.fullVoltage - profile.emptyVoltage) * 0.8;
    check(loaded.currentAmps > 0 && loaded.voltageVolts < initialVoltage && loaded.chargeFraction < 0.8 &&
      loaded.remainingEnergyWattHours < 80,
      "Measured load integrates positive pack current, energy loss and resistive voltage sag");
    var low = new BatterySimulator("low", profile, 0.02);
    low.advance(100, 0, 1, Int64.ofInt(1_000_000_000));
    check(!low.supplyAvailable, "A low-charge pack below its cutoff cannot supply the welding arc");
    var sagging = new BatterySimulator("sagging", profile, 0.3);
    var sagged = sagging.advance(5000, 0, 1, Int64.ofInt(1_000_000_000));
    check(sagged.remainingEnergyWattHours > 20 && sagged.voltageVolts < profile.cutoffVoltage &&
      !sagging.supplyAvailable,
      "Voltage sag trips the supply even when the pack still has usable stored energy");
    var recovered = sagging.advance(0, 0, 1, Int64.ofInt(2_000_000_000));
    check(recovered.voltageVolts > profile.cutoffVoltage && sagging.supplyAvailable &&
      Math.abs(recovered.remainingEnergyWattHours - sagged.remainingEnergyWattHours) < 1e-9,
      "Removing an unsupported load restores open-circuit voltage without inventing charge");
    var overloaded = new BatterySimulator("overload", profile);
    overloaded.advance(40000, 0, 1, Int64.ofInt(1_000_000_000));
    check(!overloaded.supplyAvailable, "A demand outside the pack's current envelope trips the supply");
    var charging = new BatterySimulator("charging", profile, 0.2);
    var charged = charging.advance(0, 10, 1800, Int64.ofInt(1_000_000_000));
    check(charged.currentAmps < 0 && charged.chargeFraction > 0.2,
      "A dock charger reports negative current and increases stored energy");
    var motor = new MachineAssembly();
    motor.addComponent("base", new RobotFlange(50));
    motor.addComponent("motor", NemaStepper.frame(23));
    motor.addComponent("driver", new MotorDriver("GENERIC-DM542", 2.8, 16));
    motor.addMateOnAxis("turn", "continuous", "base", "face", "motor", "mountFace", {x: 0, y: 0, z: 1});
    motor.addMate("driver-mount", "fixed", "base", "face", "driver", "mount");
    motor.addMotor("drive", "turn", "motor", "driver");
    motor.addComponent("supply", new PowerSupply(24, 10));
    motor.addMate("supply-mount", "fixed", "base", "face", "supply", "mount");
    motor.connectPorts("driver-power", "supply", "power1", "driver", "power");
    var before = motor.actuatorFor("drive").maxRate;
    motor.replaceComponent("supply", new IsolatedDcConverter(48, 24, 10, 1));
    motor.addComponent("battery", new BatteryPack(48, 1000, 300, 200, 100, 10));
    motor.connectPorts("pack", "battery", "power1", "supply", "dc");
    check(Math.abs(before - motor.actuatorFor("drive").maxRate) < 1e-9,
      "Driver voltage uses the converter output, not the upstream 48 V pack");
    check(motor.upstream("driver", "power").port.instanceId == "battery", "Energy still traces to the pack");
    rejects(() -> { motor.replaceComponent("supply", new BatteryPack(48, 1000, 300, 200, 100, 10)); },
      "A replacement missing the mounting interface is rejected");
    check(Math.abs(before - motor.actuatorFor("drive").maxRate) < 1e-9, "Failed replacement does not mutate the assembly");
    rejects(() -> { motor.replaceComponent("missing", new PowerSupply(24, 10)); }, "Replacing an unknown member fails");
    PowerSupplyTests.run();
    Sys.println('Power components: $checks checks passed, existing voltage-resolution checks passed');
  }
}
