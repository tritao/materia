package machinekit.welding;

import machinekit.component.BomItem;
import machinekit.component.BomRevision;
import machinekit.component.ComponentNumberKey;
import machinekit.component.Dimension;
import machinekit.component.Procurement.ProcurementCodec;

/** Provisional, individually purchased welding leads; ratings and terminations remain selection inputs. */
class WeldingServiceStock {
  static function line(number:String, description:String, inputs:Array<String>,
    requirements:Array<machinekit.component.Procurement.ProcurementRequirement>):BomItem return {
    partNumber: number,
    description: description,
    quantity: 1,
    material: null,
    procurement: Buy(null, null, null),
    maturity: "Assumed",
    requirements: requirements,
    fingerprint: BomRevision.fingerprint(inputs.join("|"), 1)
  };

  static function positive(value:Float):Float {
    if (!Math.isFinite(value) || value <= 0) throw "Welding service dimensions must be finite and positive";
    return value;
  }

  public static function gasHose(length:Float):BomItem return line(
    "HOSE-GAS-" + Dimension.format(positive(length)) + "M", "Shielding gas hose",
    ["gas-hose", ComponentNumberKey.encode(length)], [
      ProcurementCodec.requirement("length", "m", null, null, length),
      ProcurementCodec.choice("service", "shielding gas")
    ]);

  public static function powerLead(work:Bool, length:Float, area:Float):BomItem return line(
    "CABLE-" + (work ? "WORK" : "WELD") + "-" + Dimension.format(positive(area)) + "-" + Dimension.format(positive(length)) + "M",
    work ? "Welding work lead" : "Welding power cable",
    [work ? "work-lead" : "weld-lead", ComponentNumberKey.encode(length), ComponentNumberKey.encode(area)], [
      ProcurementCodec.requirement("length", "m", null, null, length),
      ProcurementCodec.requirement("conductorArea", "mm2", null, null, area)
    ]);

  public static function controlLead(length:Float, cores:Int):BomItem {
    if (cores <= 0) throw "Control lead needs a positive core count";
    return line(
    "CABLE-CONTROL-" + cores + "-" + Dimension.format(positive(length)) + "M", "Welder control cable",
    ["control-lead", ComponentNumberKey.encode(length), Std.string(cores)], [
      ProcurementCodec.requirement("length", "m", null, null, length),
      ProcurementCodec.requirement("cores", "1", null, null, cores)
    ]);
  }

  public static function torchPack(role:String, length:Float):BomItem {
    if (role == null || role.length == 0) throw "Torch pack needs a declared role";
    return line(
    "HOSEPACK-" + role + "-" + Dimension.format(positive(length)) + "M", "Torch hose pack",
    ["torch-pack", role, ComponentNumberKey.encode(length)], [
      ProcurementCodec.requirement("length", "m", null, null, length),
      ProcurementCodec.choice("power", "included"),
      ProcurementCodec.choice("shieldingGas", "included"),
      ProcurementCodec.choice("wireLiner", "included"),
      ProcurementCodec.choice("control", "included")
    ]);
  }
}
