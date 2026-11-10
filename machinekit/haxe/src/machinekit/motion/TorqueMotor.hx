package machinekit.motion;

import machinekit.motion.PrecisionErrors;
import machinekit.component.*;
import machinekit.component.ComponentRecipeSupport as P;

/** Frameless direct-drive torque motor. Reference row is Assumed: its outside, bore and height are not a product's. */
class TorqueMotor extends MachineComponent implements machinekit.motion.MotorDrive {
  override public function carriesJournal():Bool return true;

  public static function parameters():Array<ComponentParameter> return EngineeringMetadata.parameters().concat([
    P.quantity("outside", 150, "mm"),
    P.quantity("bore", 80, "mm"),
    P.quantity("height", 45, "mm"),
    P.quantity("continuousTorque", 10, "N m"),
    P.quantity("peakTorque", 30, "N m"),
    P.quantity("ratedSpeed", 60 * 2 * Math.PI / 60, "rad/s"),
    P.quantity("maxSpeed", 120 * 2 * Math.PI / 60, "rad/s"),
    P.quantity("rotorInertia", 0.003, "kg m2"),
    P.quantity("coggingTorque", 0.03, "N m"),
    P.count("poleCount", 24),
    P.count("coggingOrder", 48),
    P.quantity("torqueConstant", 1.2, "N m/A rms"),
    P.quantity("backEmfConstant", 0.6928203230275509, "V/(rad/s)"),
    P.quantity("phaseResistance", 0.3, "ohm")
  ]);
  static final recipe = new machinekit.cache.Once<ComponentType>();
  public static function recipeType():ComponentType {
    return recipe.get(() -> new ComponentType(
      "machinekit.motion.torque-motor",
      parameters(),
      v -> new TorqueMotor(v)
    ).numbered(v -> 'TORQUE-MOTOR-ASSUMED-${Dimension.format(v.number("continuousTorque"))}NM-${Dimension.format(v.number("peakTorque"))}NM')
      .procured(_ -> Buy(null, null, null)).requiring(v -> [
        machinekit.component.Procurement.ProcurementCodec.requirement("outside", "mm", null, null, v.number("outside")),
        machinekit.component.Procurement.ProcurementCodec.requirement("bore", "mm", null, null, v.number("bore")),
        machinekit.component.Procurement.ProcurementCodec.requirement("height", "mm", null, null, v.number("height")),
        machinekit.component.Procurement.ProcurementCodec.requirement("continuousTorque", "N m", v.number("continuousTorque")),
        machinekit.component.Procurement.ProcurementCodec.requirement("peakTorque", "N m", v.number("peakTorque")),
        machinekit.component.Procurement.ProcurementCodec.requirement("ratedSpeed", "rad/s", v.number("ratedSpeed")),
        machinekit.component.Procurement.ProcurementCodec.requirement("maxSpeed", "rad/s", v.number("maxSpeed")),
        machinekit.component.Procurement.ProcurementCodec.requirement("coggingTorque", "N mm", null, v.number("coggingTorque") * 1000)
      ]));
  }
  public static function catalog():machinekit.catalog.Catalog<ComponentValues
    > return EngineeringMetadata.assumedCatalog(
      "Frameless direct-drive torque motor",
      "TORQUE-MOTOR-10NM",
      parameters(),
      "machinekit/examples/flexspline-hobber/docs/PLAN.md (assumed reference design)"
    );
  public function new(?overrides:ComponentValues) {
    var v = P.resolve(parameters(), overrides);
    super("TORQUE-MOTOR-10NM", "Frameless direct-drive torque motor (reference envelope)", "aluminium 6061", false, v);
    if (v.get("material") != null) setMaterial(v.token("material"));
    ComponentEnvelopes.validateAnnular(v);
    addConnector("mount", Mount, Solids.axial(0, 0, 0));
    addConnector("axis", Axis, Solids.axial(0, 0, 0));
    addConnector("top", Mount, Solids.axial(0, 0, parameterNumber("height")));
    addFacet(new EngineeringMetadata(v));
    addFacet(PrecisionErrors.fromValues(v, [{name: "coggingTorque", unit: "N m"}]));
    if (parameterNumber("continuousTorque") <= 0 || parameterNumber("peakTorque") < parameterNumber("continuousTorque")
      || parameterNumber("ratedSpeed") <= 0 || parameterNumber("maxSpeed") <= parameterNumber("ratedSpeed")
      || parameterCount("poleCount") < 2 || parameterCount("poleCount") % 2 != 0
      || parameterCount("coggingOrder") < 1) throw "invalid torque motor ratings";
    for (k in ["rotorInertia", "torqueConstant",
      "backEmfConstant", "phaseResistance"]) if (parameterNumber(k) <= 0) throw "motor rating must be positive";
  }
  override public function componentType():Null<ComponentType> return recipeType();
  public var ratingsBasis(get, never):machinekit.transmission.ValueBasis;
  function get_ratingsBasis():machinekit.transmission.ValueBasis {
    var metadata = EngineeringMetadata.of(this);
    if (metadata == null) throw "Motor engineering metadata missing";
    return metadata.ratingsBasis;
  }
  public var continuousTorque(get, never):Float;
  function get_continuousTorque():Float return parameterNumber("continuousTorque");
  public var peakTorque(get, never):Float;
  function get_peakTorque():Float return parameterNumber("peakTorque");
  public var ratedSpeed(get, never):Float;
  function get_ratedSpeed():Float return parameterNumber("ratedSpeed");
  public var maxSpeed(get, never):Float;
  function get_maxSpeed():Float return parameterNumber("maxSpeed");
  public var rotorInertia(get, never):Float;
  function get_rotorInertia():Float return parameterNumber("rotorInertia");
  public var coggingTorque(get, never):Float;
  function get_coggingTorque():Float return parameterNumber("coggingTorque");
  override public function motorDrive():Null<machinekit.motion.MotorDrive> return this;
  public function electricalRating():Null<machinekit.motion.ServoElectricalRating> return {
    torqueConstant: parameterNumber("torqueConstant"),
    backEmfConstant: parameterNumber("backEmfConstant"),
    phaseResistance: parameterNumber("phaseResistance"),
    mechanicalEfficiency: 0.95
  };
  public function jointTorque(motorTorque:Float):Float {
    if (!Math.isFinite(motorTorque)) throw "invalid torque";
    return motorTorque;
  }
  public function actuator(id:String,
    joint:String, volts:Float, margin:Float, ?current:Float):materia.assembly.AssemblyDefinition.AssemblyActuator {
    return {
      id: id,
      joint: joint,
      drive: "servo",
      maxEffort: peakTorque,
      maxRate: maxSpeed,
      rotorInertia: rotorInertia,
      ratedTorque: continuousTorque,
      peakTorque: peakTorque,
      ratedSpeed: ratedSpeed,
      maxSpeed: maxSpeed,
      torqueSpeed: [0, peakTorque, ratedSpeed, peakTorque, maxSpeed, continuousTorque],
      assumed: ratingsBasis == Assumed ? ["torque motor ratings"] : null
    };
  }
  override public function hasGeometry():Bool return true;
  /** Stator and rotor across the air gap (`RingParts.framelessMotor`). Envelope detail is the annulus, which also sets the mass. */
  override public function geometry(detail:ComponentDetail = Preview):cadkit.modeling.Part return detail
    == Envelope ? ComponentEnvelopes.annular(values()) : RingParts.framelessMotor(
      parameterNumber("outside"),
      parameterNumber("bore"),
      parameterNumber("height")
    );
  override public function massGeometry():Null<cadkit.modeling.Part> return ComponentEnvelopes.annular(values());
}
