import machinekit.power.BatteryPack;
import machinekit.power.BatteryStorage;
import machinekit.power.Inverter;
import machinekit.assembly.MachineAssembly;
import machinekit.welding.WeldingPowerSource;

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
    check(battery.outputVoltage("power2") == 48, "Every declared terminal supplies the nominal DC voltage");
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
    Sys.println('Power components: $checks checks passed');
  }
}
