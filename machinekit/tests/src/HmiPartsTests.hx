import cadkit.modeling.Part;
import haxeon.Equality;
import machinekit.component.ComponentDetail;
import machinekit.component.ComponentParameterType;
import machinekit.component.ComponentValues;
import machinekit.component.MachineComponent;
import machinekit.component.MachineKitComponents;
import machinekit.hmi.*;

class HmiPartsTests {
  static function check(ok:Bool, message:String):Void if (!ok) throw message;
  static function near(a:Float, b:Float, message:String, tolerance:Float = 1e-6):Void check(Math.abs(a - b) <= tolerance, '$message: $a vs $b');
  static function rejects(message:String, body:() -> Void):Void {
    var failed = false;
    try body() catch (_:Dynamic) failed = true;
    check(failed, 'Expected rejection: $message');
  }

  static function halfExtent(f:Footprint):{x:Float, y:Float} return switch f {
    case Circle(d): {x: d / 2, y: d / 2};
    case Rect(w, h): {x: w / 2, y: h / 2};
    case _: {x: 0.0, y: 0.0};
  };

  /** One part's solid against what it says about itself. */
  static function checkPart(part:HmiPart, name:String):Void {
    var solid = part.geometry(Preview);
    check(solid.solidCount() == 1 && solid.volume() > 0, '$name is one solid');
    var b = solid.shape.bounds();
    if (part.aboveFace() > 0) near(b.get_max().get_z(), part.aboveFace(), '$name height above the face');
    else check(b.get_max().get_z() <= 1e-9, '$name stays behind the face');
    near(b.get_min().get_z(), -part.behindFace(), '$name depth behind the face');
    var keep = halfExtent(part.panelKeepOut()), body = halfExtent(part.rearBody()), cut = halfExtent(part.panelCutout());
    var hx = Math.max(keep.x, Math.max(body.x, cut.x)), hy = Math.max(keep.y, Math.max(body.y, cut.y));
    check(Math.max(Math.abs(b.get_min().get_x()), Math.abs(b.get_max().get_x())) <= hx + 1e-6, '$name plan width inside its footprints');
    check(Math.max(Math.abs(b.get_min().get_y()), Math.abs(b.get_max().get_y())) <= hy + 1e-6, '$name plan depth inside its footprints');
    near(part.massProperties().mass, part.massGrams() / 1000.0, '$name declared mass', 1e-9);
    solid.close();
  }

  /** The contract RecipeContractTests applies to every recipe, for these: each option and flag builds a valid solid, and a rebuilt part is identical. */
  static function checkRecipes():Void {
    var registry = MachineKitComponents.standard();
    var checked = 0;
    for (type in registry.all()) {
      if (type.id.indexOf("machinekit.hmi.") != 0) continue;
      var base:MachineComponent = type.create(new ComponentValues());
      var variants:Array<ComponentValues> = [base.values()];
      for (parameter in type.parameters()) switch parameter.type {
        case Choice(options): for (option in options) variants.push(base.values().setToken(parameter.name, option));
        case Bool: variants.push(base.values().setBoolean(parameter.name, !base.values().boolean(parameter.name)));
        case _:
      }
      for (values in variants) {
        var component = type.create(values);
        var rebuilt = type.create(component.values());
        check(type.key(rebuilt.values()) == type.key(component.values()), '${type.id}: values changed after rebuild');
        check(rebuilt.designation == component.designation, '${type.id}: designation changed after rebuild');
        check(Equality.equals(component.connectors(), rebuilt.connectors()), '${type.id}: connectors changed after rebuild');
        check(Equality.equals(component.ports(), rebuilt.ports()), '${type.id}: ports changed after rebuild');
        check(component.bom.partNumber == rebuilt.bom.partNumber && component.bom.description == rebuilt.bom.description, '${type.id}: BOM changed after rebuild');
        for (detail in [ComponentDetail.Preview, ComponentDetail.Envelope]) {
          var solid = component.geometry(detail);
          check(solid.solidCount() == 1 && solid.volume() > 0, '${type.id} ${type.key(values)} ${detail}: one valid solid');
          solid.close();
        }
        var preview = component.geometry(ComponentDetail.Preview), envelope = component.geometry(ComponentDetail.Envelope);
        var inner = preview.shape.bounds(), outer = envelope.shape.bounds();
        check(inner.get_min().get_z() >= outer.get_min().get_z() - 1e-3 && inner.get_max().get_z() <= outer.get_max().get_z() + 1e-3
          && inner.get_max().get_x() <= outer.get_max().get_x() + 1e-3 && inner.get_max().get_y() <= outer.get_max().get_y() + 1e-3, '${type.id}: preview outside its envelope');
        preview.close(); envelope.close();
        checked++;
      }
    }
    check(checked >= 20, "Recipe variants checked: " + checked);
  }

  public static function run():Void {
    checkRecipes();
    var registry = MachineKitComponents.standard();
    for (id in ["hand-wheel", "emergency-stop", "rotary-switch", "push-button", "oled-module"])
      check(registry.category("machinekit.hmi." + id) == "Operator controls", 'machinekit.hmi.$id is filed under Operator controls');

    var yumo = PartPresets.yumoIsm6045(), vallder = PartPresets.vallderMpg(), estop = PartPresets.xb2Es542(), estop2 = PartPresets.xb2Es542(2);
    var compact = PartPresets.compactEStop(), axis = PartPresets.rotarySwitch(12), step = PartPresets.rotarySwitch(3);
    var button = PartPresets.pushButton16(), oled = PartPresets.oled242();
    var entries:Array<{p:HmiPart, n:String}> = [{p: yumo, n: "YUMO wheel"}, {p: vallder, n: "Vallder wheel"}, {p: estop, n: "E-stop"}, {p: estop2, n: "two-block E-stop"},
        {p: compact, n: "compact E-stop"}, {p: axis, n: "axis switch"}, {p: step, n: "step switch"}, {p: button, n: "button"}, {p: oled, n: "OLED"}];
    for (entry in entries) checkPart(entry.p, entry.n);

    // The drawings' numbers come out of the recipes.
    near(yumo.aboveFace(), 38.0, "YUMO above"); near(yumo.behindFace(), 10.0, "YUMO behind");
    check(yumo.dataSource() == "vendor-drawing" && estop.dataSource() == "listing" && axis.dataSource() == "assumed", "Provenance");
    near(vallder.aboveFace(), 41.0, "Vallder above"); near(vallder.behindFace(), 12.0, "Vallder behind");
    near(yumo.maxPanelThickness(), 6.0, "YUMO panel limit"); near(vallder.maxPanelThickness(), 3.0, "Vallder panel limit");
    check(estop.maxPanelThickness() > 1e6, "A part with no stated limit allows any panel");
    near(estop.aboveFace(), 31.0, "E-stop above"); near(estop.behindFace(), 43.0, "E-stop behind");
    near(yumo.panelHoles()[0].diameter, 3.2, "YUMO stud holes"); near(vallder.panelHoles()[0].diameter, 4.8, "Vallder stud holes");
    check(estop2.designation != estop.designation && estop2.bom.description.indexOf("2 NC") >= 0, "A second block is a different part");
    check(axis.designation != step.designation && axis.bom.description.indexOf("12 positions") >= 0 && step.bom.description.indexOf("3 positions") >= 0, "Positions are in the part");
    near(yumo.massProperties().mass * 1000.0, 250.0, "YUMO mass", 1e-6);

    rejects("a ring wider than the flange", () -> new HandWheel(new ComponentValues().setNumber("ringDiameter", 70)));
    rejects("too many contact blocks", () -> new EmergencyStop(new ComponentValues().setInteger("blocks", 4)));
    rejects("a handle outside the dial", () -> new HandWheel(new ComponentValues().setNumber("handleRadius", 30)));
    trace("hmi parts tests passed");
  }
  public static function main():Void run();
}
