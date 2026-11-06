import machinekit.component.Same;
import machinekit.component.ComponentDetail;
import machinekit.component.ComponentParameterType;
import machinekit.component.ComponentValues;
import machinekit.catalog.CatalogMetadata;
import machinekit.component.MachineComponent;
import machinekit.component.MachineKitComponents;
import machinekit.component.PanelMountFacet;
import cadkit.modeling.FootprintGeometry;
import machinekit.hmi.*;
import materia.assembly.AssemblyFrames;

class HmiPartsTests {
  static function m(part:MachineComponent):PanelMountFacet {
    var facet = PanelMountFacet.of(part);
    if (facet == null) throw part.designation + " states no panel mount";
    return facet;
  }
  static function check(ok:Bool, message:String):Void if (!ok) throw message;
  static function near(a:Float, b:Float, message:String, tolerance:Float = 1e-6):Void check(
    Math.abs(a - b) <= tolerance,
    '$message: $a vs $b'
  );
  static function rejects(message:String, body:() -> Void):Void {
    var failed = false;
    try body() catch (_:Dynamic) failed = true;
    check(failed, 'Expected rejection: $message');
  }

  /** One part's solid against what it says about itself. */
  static function checkPart(part:HmiPart, name:String):Void {
    var solid = part.geometry(Preview);
    check(solid.solidCount() == 1 && solid.volume() > 0, '$name is one solid');
    var b = solid.shape.bounds();
    if (m(part).above > 0) near(b.get_max().get_z(), m(part).above, '$name height above the face');
    else check(b.get_max().get_z() <= 1e-9, '$name stays behind the face');
    near(b.get_min().get_z(), -m(part).behind, '$name depth behind the face');
    var keep = FootprintGeometry.halfExtent(m(part).keepOut),
      body = FootprintGeometry.halfExtent(m(part).body), cut = FootprintGeometry.halfExtent(m(part).cutout);
    var hx = Math.max(keep.x, Math.max(body.x, cut.x)), hy = Math.max(keep.y, Math.max(body.y, cut.y));
    check(
      Math.max(Math.abs(b.get_min().get_x()), Math.abs(b.get_max().get_x())) <= hx + 1e-6,
      '$name plan width inside its footprints'
    );
    check(
      Math.max(Math.abs(b.get_min().get_y()), Math.abs(b.get_max().get_y())) <= hy + 1e-6,
      '$name plan depth inside its footprints'
    );
    near(part.massProperties().mass, part.n("massGrams") / 1000.0, '$name declared mass', 1e-9);
    solid.close();
  }

  /**
   * A wired control's terminals: the named Terminal frame at `at` whose +Y is `direction` (the way its wires leave), every
   * listed port on it with the stated number of wires, on the device and on the body that carries them when it is expanded.
   */
  static function checkTerminals(
    part:HmiPart,
    name:String,
    terminal:String,
    at:Array<Float>,
    direction:Array<Float>,
    ports:Array<{name:String, wires:Int}>
  ):Void {
    var c = part.connector(terminal);
    check(c.role == Terminal, '$name: $terminal is a terminal frame');
    near(c.frame.x, at[0], '$name: $terminal x');
    near(c.frame.y, at[1], '$name: $terminal y');
    near(c.frame.z, at[2], '$name: $terminal z');
    var tip = AssemblyFrames.transformPoint(c.frame, 0, 1, 0);
    near(tip.x - c.frame.x, direction[0], '$name: wires leave along x');
    near(tip.y - c.frame.y, direction[1], '$name: wires leave along y');
    near(tip.z - c.frame.z, direction[2], '$name: wires leave along z');
    check(
      [for (p in part.ports()) p.name].join(",") == [for (p in ports) p.name].join(","),
      '$name ports: ' + [for (p in part.ports()) p.name].join(",")
    );
    for (want in ports) {
      var port = part.port(want.name);
      check(port.connector == terminal, '$name: ${want.name} is wired at $terminal');
      switch port.iface {
        case Plug(_, pins):
          check(pins == want.wires, '$name: ${want.name} has ${want.wires} wires, not $pins');
        default:
          check(false, '$name: ${want.name} is a plug');
      }
    }
    var bodies = part.expand();
    if (bodies == null) return;
    var carried = 0;
    for (b in bodies.components()) for (p in b.component.ports()) {
      carried++;
      near(
        b.component.connector(p.connector).frame.z,
        part.connector(p.connector).frame.z,
        '$name: ${b.id} keeps the frame'
      );
    }
    check(carried == ports.length, '$name: the bodies carry every port once, $carried');
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
        case Choice(options):
          for (option in options) variants.push(base.values().setToken(parameter.name, option));
        case CatalogDesignation(index):
          for (designation in index.designations()) variants.push(base.values().setToken(parameter.name, designation));
        case Bool:
          variants.push(base.values().setBoolean(parameter.name, !base.values().boolean(parameter.name)));
        case _:
      }
      for (values in variants) {
        var component = type.create(values);
        var rebuilt = type.create(component.values());
        check(type.key(rebuilt.values()) == type.key(component.values()), '${type.id}: values changed after rebuild');
        check(rebuilt.designation == component.designation, '${type.id}: designation changed after rebuild');
        check(
          Same.connectors(component.connectors(), rebuilt.connectors()),
          '${type.id}: connectors changed after rebuild'
        );
        check(Same.ports(component.ports(), rebuilt.ports()), '${type.id}: ports changed after rebuild');
        check(
          component.bom.partNumber == rebuilt.bom.partNumber && component.bom.description == rebuilt.bom.description,
          '${type.id}: BOM changed after rebuild'
        );
        for (detail in [ComponentDetail.Preview, ComponentDetail.Envelope]) {
          var solid = component.geometry(detail);
          check(
            solid.solidCount() == 1 && solid.volume() > 0,
            '${type.id} ${type.key(values)} ${detail}: one valid solid'
          );
          solid.close();
        }
        var preview = component.geometry(ComponentDetail.Preview),
          envelope = component.geometry(ComponentDetail.Envelope);
        var inner = preview.shape.bounds(), outer = envelope.shape.bounds();
        check(
          inner.get_min().get_z() >= outer.get_min().get_z() - 1e-3
          && inner.get_max().get_z() <= outer.get_max().get_z() + 1e-3 && inner.get_max().get_x()
          <= outer.get_max().get_x() + 1e-3 && inner.get_max().get_y() <= outer.get_max().get_y() + 1e-3,
          '${type.id}: preview outside its envelope'
        );
        preview.close();
        envelope.close();
        checked++;
      }
    }
    check(checked >= 20, "Recipe variants checked: " + checked);
  }

  public static function run():Void {
    // Every glyph of the stroke font builds a valid solid (a stroke that touches itself fails the polygon).
    for (i in 0..."ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789-+./".length) {
      var glyph = "ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789-+./".charAt(i);
      var text = cadkit.modeling.StrokeText.solid(glyph, 3.0, 0.4, 0.0, 0.3, i % 2 == 0);
      check(text.volume() > 0, 'Stroke glyph $glyph has volume');
      text.close();
    }
    checkRecipes();
    var registry = MachineKitComponents.standard();
    for (id in ["hand-wheel", "emergency-stop", "rotary-switch", "push-button", "oled-module"]) check(
      registry.category("machinekit.hmi." + id) == "Operator controls",
      'machinekit.hmi.$id is filed under Operator controls'
    );

    var yumo = PartPresets.yumoIsm6045(),
      vallder = PartPresets.vallderMpg(), estop = PartPresets.xb2Es542(), estop2 = PartPresets.xb2Es542(2);
    var compact = PartPresets.compactEStop(), axis = PartPresets.rotarySwitch(12), step = PartPresets.rotarySwitch(3);
    var button = PartPresets.pushButton16(), oled = PartPresets.oled242();
    var entries:Array<{p:HmiPart, n:String}> = [
      {p: yumo, n: "YUMO wheel"},
      {p: vallder, n: "Vallder wheel"},
      {p: estop, n: "E-stop"},
      {p: estop2, n: "two-block E-stop"},
      {p: compact, n: "compact E-stop"},
      {p: axis, n: "axis switch"},
      {p: step, n: "step switch"},
      {p: button, n: "button"},
      {p: oled, n: "OLED"}
    ];
    for (entry in entries) checkPart(entry.p, entry.n);

    // The drawings' numbers come out of the recipes.
    near(m(yumo).above, 38.0, "YUMO above");
    near(m(yumo).behind, 10.0, "YUMO behind");
    check(
      m(yumo).source == "vendor-drawing" && m(estop).source == "listing" && m(axis).source == "assumed",
      "Provenance"
    );
    near(m(vallder).above, 41.0, "Vallder above");
    near(m(vallder).behind, 12.0, "Vallder behind");
    near(m(yumo).maxPanelThickness, 6.0, "YUMO panel limit");
    near(m(vallder).maxPanelThickness, 3.0, "Vallder panel limit");
    check(m(estop).maxPanelThickness > 1e6, "A part with no stated limit allows any panel");
    near(m(estop).above, 31.0, "E-stop above");
    near(m(estop).behind, 43.0, "E-stop behind");
    near(m(yumo).holes[0].diameter, 3.2, "YUMO stud holes");
    near(m(vallder).holes[0].diameter, 4.8, "Vallder stud holes");
    check(
      estop2.designation != estop.designation && estop2.bom.description.indexOf("2 NC") >= 0,
      "A second block is a different part"
    );
    check(
      axis.designation != step.designation
      && axis.bom.description.indexOf("12 positions") >= 0 && step.bom.description.indexOf("3 positions") >= 0,
      "Positions are in the part"
    );
    // The contacts are ports to wire into a safety chain: one NC dry contact per block.
    var contacts = estop2.ports();
    check(
      contacts.length == 2 && contacts[0].name == "nc1" && contacts[1].name == "nc2" && contacts[0].required == true,
      "Two blocks, two required NC contact ports"
    );
    check(estop.ports().length == 1 && compact.ports().length == 1, "One block, one contact port");
    near(yumo.massProperties().mass * 1000.0, 250.0, "YUMO mass", 1e-6);

    // The numbers come from catalog rows, with their provenance.
    check(
      yumo.values().token("model") == ControlCatalogs.YUMO
      && vallder.values().token("model") == ControlCatalogs.VALLDER,
      "Presets select catalog rows"
    );
    check(
      ControlCatalogs.wheels().designations().join(",") == "CUSTOM,YUMO-ISM6045-100B,VALLDER-1165001",
      "Wheel designations"
    );
    var yumoMeta = ControlCatalogs.wheels().metadata(ControlCatalogs.YUMO);
    check(
      yumoMeta.verifiedFields.indexOf("studCircleDiameter") >= 0 && yumoMeta.conformance == NominalEnvelope,
      "Wheel provenance"
    );
    check(ControlCatalogs.eStops().metadata(ControlCatalogs.EAO).standard == "EN 60947-5-5", "Compact E-stop standard");
    check(
      compact.values().token("model") == ControlCatalogs.EAO && m(compact).behind < 20.0,
      "The compact E-stop row is shallow"
    );
    rejects(
      "a ring wider than the flange",
      () -> new HandWheel(new ComponentValues().setToken("model", ControlCatalogs.CUSTOM).setNumber("ringDiameter", 70))
    );
    rejects("too many contact blocks", () -> new EmergencyStop(new ComponentValues().setInteger("blocks", 4)));

    // Every wired control says where its wires attach, so a board can place its connectors near them.
    var back = [0.0, 0.0, -1.0];
    checkTerminals(
      button,
      "button",
      "terminals",
      [0, 0, -m(button).behind],
      back,
      [{name: "no", wires: 2}, {name: "led", wires: 2}]
    );
    var plain = new PushButton(new ComponentValues().setBoolean("illuminated", false));
    checkTerminals(plain, "plain button", "terminals", [0, 0, -m(plain).behind], back, [{name: "no", wires: 2}]);
    checkTerminals(axis, "axis switch", "terminals", [0, 0, -m(axis).behind], back, [{name: "contacts", wires: 13}]);
    checkTerminals(step, "step switch", "terminals", [0, 0, -m(step).behind], back, [{name: "contacts", wires: 4}]);
    checkTerminals(vallder, "Vallder wheel", "terminals", [0, 0, -vallder.n("terminalDepth")], back, [for (t in [
      "A",
      "B",
      "0V",
      "VCC"
    ]) {
      name:t, wires:1
    }
    ]);
    check(oled.connector("header").frame.x > 27.0, "The OLED's header pins end at the board's right edge");
    checkTerminals(
      oled,
      "OLED",
      "header",
      [oled.connector("header").frame.x, oled.connector("header").frame.y, oled.connector("header").frame.z],
      [1.0, 0.0, 0.0],
      [{name: "header", wires: 7}]
    );
    var blocks = estop2.connector("block2").frame.y - estop2.connector("block1").frame.y;
    near(blocks, estop2.n("blockThickness"), "The E-stop's blocks are a block apart");
    checkTerminals(estop, "E-stop", "block1", [0, 0, -m(estop).behind], back, [{name: "nc1", wires: 2}]);
    rejects(
      "a handle outside the dial",
      () -> new HandWheel(new ComponentValues().setToken("model", ControlCatalogs.CUSTOM).setNumber("handleRadius", 30))
    );
    rejects("an unknown model", () -> new HandWheel(new ComponentValues().setToken("model", "NOT-A-WHEEL")));
    trace("hmi parts tests passed");
  }
  public static function main():Void run();
}
