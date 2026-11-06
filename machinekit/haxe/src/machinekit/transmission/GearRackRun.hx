package machinekit.transmission;

import machinekit.component.*;
import machinekit.component.ComponentRecipeSupport as P;
import cadkit.modeling.Part;
import materia.assembly.AssemblyFrames as F;

/** Whole sourced sections, never a nominal extrusion with invented drilling.
 * Connectors retain section identity. Joint pitch setting is an assembly task. */
class GearRackRun extends MachineComponent {
  static final recipe = new machinekit.cache.Once<ComponentType>();
  public final section:GearRack;
  public final count:Int;
  public final length:Float;
  /** Shop operation after pitch alignment, not a supplier pilot dimension. */
  public final finishPinDiameter:Float;
  static function inputs():Array<ComponentParameter> return [
    P.choice("designation", ["29-15-055", "29-15-105", "29-20-055", "29-20-105"], "29-15-055"),
    P.count("sections", 1),
    P.length("finishPinDiameter", 0)
  ];
  public function new(?overrides:ComponentValues) {
    var v = P.resolve(inputs(), overrides);
    count = v.integer("sections");
    if (count < 1) throw "Rack run requires whole catalogue sections";
    section = GearRack.catalogue(v.token("designation"));
    length = count * section.spec.lengthMm;
    // Each section owns its source mounting and two pilot connectors,
    // plus two run datums. The 1000 mm rows have more mounting holes.
    var maximum = Std.int(98 / (section.spec.holeCount + 2));
    if (count > maximum) throw "Rack run exceeds the assembly connector limit for " + section.spec.designation;
    finishPinDiameter = v.number("finishPinDiameter");
    if (finishPinDiameter != 0
      && finishPinDiameter != 6) throw "Only the reviewed 6 H7 match-ream operation is supported";
    if (finishPinDiameter > 0
      && section.spec.pinPilotMm >= finishPinDiameter) throw "Locating pin needs reaming allowance";
    super("RACK-RUN-" + section.spec.designation + "-" + count, "Joined catalogue rack sections", "steel", false, v);
    for (i in 0...count) for (c in section.mountingConnectors()) addConnector(
      'section${i+1}-' + c.name,
      Mount,
      F.compose(F.translation(0, 0, i * section.spec.lengthMm), c.frame)
    );
    for (i in 0...count) for (pin in 1...3) addConnector(
      'section${i+1}-pinPilot$pin',
      Mount,
      F.compose(F.translation(0, 0, i * section.spec.lengthMm), section.connector('pinPilot$pin').frame)
    );
    var removed = finishPinDiameter == 0 ? 0 : 2 * Math.PI / 4 * (finishPinDiameter
      * finishPinDiameter - section.spec.pinPilotMm * section.spec.pinPilotMm) * section.spec.faceWidthMm * 7.85e-6;
    var m = count * (section.spec.massKg - removed), b = section.spec.faceWidthMm, h = section.spec.overallHeightMm;
    declareMass(
      m,
      new cadkit.modeling.Vector(b / 2, -section.spec.pitchHeightMm + h / 2, length / 2),
      new cadkit.InertiaTensor(
        m * (h * h + length * length) / 12,
        0,
        0,
        m * (b * b + length * length) / 12,
        0,
        m * (b * b + h * h) / 12
      )
    );
    addConnector("back", Mount, section.connector("back").frame);
    addConnector("referenceSide", Mount, section.connector("referenceSide").frame);
  }
  public static function recipeType():ComponentType {
    return recipe.get(() -> new ComponentType(
      "machinekit.transmission.gear-rack-run",
      inputs(),
      v -> new GearRackRun(v),
      false,
      false
    ));
  }
  /** Procurement remains catalogue sections; the run owns geometry/mass once.
     * Match reaming is a shop operation, not a new supplier purchase identity. */
  override function get_bom():BomItem {
    var source = section.bom;
    return {
      partNumber: source.partNumber,
      description: source.description,
      quantity: source.quantity * count,
      material: source.material,
      typeId: source.typeId,
      valuesKey: source.valuesKey,
      fingerprint: source.fingerprint,
      procurement: source.procurement,
      maturity: source.maturity,
      requirements: machinekit.component.Procurement.ProcurementCodec.copyRequirements(source.requirements),
      familyId: source.familyId,
      manufacturingVariant: source.manufacturingVariant,
      manufacturingProcess: source.manufacturingProcess,
      properties: source.properties, measure: source.measure, cuts: source.cuts
    };
  }
  override public function componentType():Null<ComponentType> return recipeType();
  override public function hasGeometry():Bool return true;
  override public function geometry(detail:ComponentDetail = Preview):Part {
    var parts:Array<Part> = [];
    // A source section is identical along the run. Build its detailed CAD
    // once and retain each independently positioned instance in the compound.
    var raw = section.geometry(detail);
    if (finishPinDiameter > 0) {
      var tools:Array<Part> = [];
      for (pin in 1...3) {
        var connector = section.connector('pinPilot$pin').frame;
        var axis = F.transformVector(connector, 0, -1, 0);
        var tool = Part.cylinder(finishPinDiameter / 2, section.spec.faceWidthMm + 0.2);
        tools.push(machinekit.assembly.PosedParts.moved(
          tool,
          machinekit.assembly.AxisBuilder.orient(
            connector.x - axis.x * 0.1,
            connector.y - axis.y * 0.1,
            connector.z - axis.z * 0.1,
            [0.0, 0, 1],
            [axis.x, axis.y, axis.z]
          )
        ));
        tool.close();
      }
      raw = Solids.cut(raw, tools);
    }
    try {
      for (i in 0...count) parts.push(raw.translated(new cadkit.modeling.Vector(0, 0, i * section.spec.lengthMm)));
      raw.close();
    } catch (e:Dynamic) {
      raw.close();
      for (p in parts) p.close();
      throw e;
    }
    // Separate catalogue sections meet at adjustable joints; do not fuse
    // them into an unearned continuous structural bar.
    try {
      var result = new Part(cadkit.Shape.compound([for (p in parts) p.shape]));
      for (p in parts) p.close();
      return result;
    } catch (e:Dynamic) {
      for (p in parts) if (!p.shape.isClosed()) p.close();
      throw e;
    }
  }
  public function nominalRack():HelicalRack return new HelicalRack(new ComponentValues().setNumber(
    "normalModule",
    section.spec.normalModule
  ).setInteger(
    "teeth",
    count * section.spec.teeth
  ).setNumber(
    "faceWidth",
    section.spec.faceWidthMm
  ).setNumber(
    "helixAngle",
    section.helixAngle()
  ).setNumber(
    "barHeight",
    section.spec.pitchHeightMm - 1.25 * section.spec.normalModule
  ).setToken(
    "hand",
    "right"
  ));
}
