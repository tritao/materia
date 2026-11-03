package machinekit.motion;

import cadkit.InertiaTensor;
import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.component.ComponentDetail;
import machinekit.component.ComponentParameter;
import machinekit.component.ComponentRecipeSupport as P;
import machinekit.component.ComponentType;
import machinekit.component.ComponentValues;
import machinekit.component.Dimension;
import machinekit.component.EngineeringMetadata;
import machinekit.component.MachineComponent;
import machinekit.component.Solids;
import machinekit.component.ToolSpec;
import machinekit.motion.ErobActuatorData as D;
import machinekit.motion.ErobActuatorData.ErobInput;
import machinekit.motion.PrecisionErrors;
import machinekit.standard.ClearanceFit;
import machinekit.standard.SocketHeadCapScrew;
import machinekit.transmission.ValueBasis;
import materia.assembly.AssemblyDefinition.AssemblyActuator;
import materia.assembly.AssemblyDefinition.AssemblyEncoder;

/**
 * One physical side of an eRob 110 actuator, which is one purchase: the fixed `housing` (stator, amplifier, front
 * seat) or the turning `output` (the harmonic output flange). `ErobActuator` puts the two together with their
 * revolute joint. Rated values come from the maker's drawing (`ErobActuatorData`) and are `Catalog`; what it does
 * not publish (bearing moment rating and stiffness, brake holding torque) is `Assumed` and only present once an
 * assumption is entered as an input.
 *
 * CAD frame: axis along +Z, output mounting face at z=0, facing +Z. The pilot disc (Ø85) stands 6 mm above the
 * housing's front face (z=-6), which is where the housing's `seat` connector is. The housing runs back to z=-80.2.
 * Connectors: `axis` and `seat` (housing, Ø110 front face), `fixedBolt1..16` (housing, M4 on Ø102), `face` and
 * `outputBolt1..12` (output, M4 on Ø77), all at their faces with +Y along +Z. Bolt 1 of each pattern is on +X; the
 * drawing does not dimension the pattern phase.
 *
 * Electrical: a `power` port (48 V DC bus, `DcBusConsumer`) and a `bus` port (EtherCAT or CANopen), both on the
 * housing. The output encoder is an absolute multi-turn-free encoder read at the output flange, so it sees the load.
 */
class ErobActuatorPart extends MachineComponent implements MotorDrive implements EncoderPart implements DcBusConsumer {
  public static inline final HOUSING = "housing";
  public static inline final OUTPUT = "output";
  /** Share of the actuator's mass that turns with the output flange. Not published; estimated. */
  public static inline final OUTPUT_MASS_FRACTION = 0.12;
  /** Housing front face and the output mounting face, z in the CAD frame. */
  public static inline final SEAT_Z = -6.0;

  public final variant:String;
  public final ratio:Int;
  public final encoderBits:Int;
  public final brake:Bool;
  public final bus:String;
  final row:D.ErobRatioRow;
  final inputs:ComponentValues;

  public static function parameters():Array<ComponentParameter> return [
    P.choice("variant", [HOUSING, OUTPUT], HOUSING),
    P.choice("ratio", [for (r in D.ratios()) Std.string(r.ratio)], "100"),
    P.choice("encoderBits", ["19", "20"], "20"),
    P.flag("brake", true),
    P.choice("bus", ["EtherCAT", "CANopen"], "EtherCAT"),
    P.optionalScalar("momentRating"),
    P.optionalScalar("bearingStiffness"),
    P.optionalScalar("brakeHoldingTorque")
  ];

  public function new(?overrides:ComponentValues) {
    var v = P.resolve(parameters(), overrides);
    variant = v.token("variant");
    ratio = Std.parseInt(v.token("ratio"));
    encoderBits = Std.parseInt(v.token("encoderBits"));
    brake = v.boolean("brake");
    bus = v.token("bus");
    row = D.row(ratio);
    var grade = D.encoderGrade(encoderBits);
    inputs = v.copy();
    var tag = 'EROB110I-V6D-${ratio}-E${encoderBits}${brake ? "B" : ""}-${bus == "EtherCAT" ? "EC" : "CO"}';
    super(
      variant == HOUSING ? tag : tag + "-OUTPUT",
      variant == HOUSING
      ? 'Integrated strain-wave actuator eRob 110 (V6 rev D), $ratio:1, $encoderBits-bit output encoder, $bus, 48 V'
      + (brake ? ", brake" : "") : 'Output flange of eRob 110 actuator $ratio:1',
      variant == HOUSING ? "aluminium 6061" : "steel",
      false,
      v
    );
    addFacet(new EngineeringMetadata(new ComponentValues().setToken("ratingsBasis", "Catalog")
      .setToken("errorsBasis", "Catalog").setToken("source", D.SOURCE)));
    addFacet(new PrecisionErrors([
      new PrecisionError("outputRepeatability", grade.repeatability, "arcsec", Catalog),
      new PrecisionError("outputAbsoluteAccuracy", grade.absolute, "arcsec", Catalog)
    ]));
    for (input in ["momentRating", "bearingStiffness", "brakeHoldingTorque"]) {
      var value = v.optionalNumber(input);
      if (value != null && !(value > 0)) throw 'eRob input $input must be positive';
    }
    if (brake == false && v.optionalNumber("brakeHoldingTorque") != null) throw "eRob without brake has no holding torque";
    var total = brake ? D.MASS_BRAKED : D.MASS, rotating = total * OUTPUT_MASS_FRACTION;
    if (variant == HOUSING) {
      var mass = total - rotating, r = D.BODY_DIAMETER / 2, b = D.BORE_DIAMETER / 2;
      var length = D.BODY_LENGTH + SEAT_Z;
      var iz = mass * (r * r + b * b) / 2, ix = mass * (3 * (r * r + b * b) + length * length) / 12;
      declareMass(mass, new Vector(0, 0, SEAT_Z - length / 2), new InertiaTensor(ix, 0, 0, ix, 0, iz));
      addConnector("axis", Axis, Solids.axial(0, 0, 0));
      addConnector("seat", Mount, Solids.axial(0, 0, SEAT_Z));
      for (i in 0...D.FIXED_BOLT_COUNT) {
        var point = fixedBolt(i);
        addConnector('fixedBolt${i + 1}', Mount, Solids.axial(point.x, point.y, SEAT_Z));
      }
      addPort({name: "power", kind: ElectricalPower, role: Consumer, iface: Unspecified, required: true});
      addPort({name: "bus", kind: Signal, role: Consumer, iface: Unspecified, required: true});
    } else {
      var r = D.OUTPUT_PILOT_DIAMETER / 2, b = D.BORE_DIAMETER / 2, h = D.PILOT_HEIGHT;
      var iz = rotating * (r * r + b * b) / 2, ix = rotating * (3 * (r * r + b * b) + h * h) / 12;
      declareMass(rotating, new Vector(0, 0, -h / 2), new InertiaTensor(ix, 0, 0, ix, 0, iz));
      includedInPurchase(housingDesignation());
      addConnector("axis", Axis, Solids.axial(0, 0, 0));
      addConnector("face", Face, Solids.axial(0, 0, 0));
      for (i in 0...D.OUTPUT_BOLT_COUNT) {
        var point = outputBolt(i);
        addConnector('outputBolt${i + 1}', Mount, Solids.axial(point.x, point.y, 0));
      }
    }
  }

  function housingDesignation():String {
    return 'EROB110I-V6D-${ratio}-E${encoderBits}${brake ? "B" : ""}-${bus == "EtherCAT" ? "EC" : "CO"}';
  }

  /** Bolt `index` (0-based) of the housing's front-face M4 pattern. */
  public static function fixedBolt(index:Int):{x:Float, y:Float} {
    var a = 2 * Math.PI * index / D.FIXED_BOLT_COUNT, r = D.FIXED_PITCH_CIRCLE / 2;
    return {x: r * Math.cos(a), y: r * Math.sin(a)};
  }

  /** Bolt `index` (0-based) of the output flange's M4 pattern. */
  public static function outputBolt(index:Int):{x:Float, y:Float} {
    var a = 2 * Math.PI * index / D.OUTPUT_BOLT_COUNT, r = D.OUTPUT_PITCH_CIRCLE / 2;
    return {x: r * Math.cos(a), y: r * Math.sin(a)};
  }

  // Ratings ------------------------------------------------------------------------------------------------

  public var ratioRow(get, never):D.ErobRatioRow;
  function get_ratioRow():D.ErobRatioRow return row;
  /** Output speed at rated torque on the 48 V bus, rad/s. */
  public function ratedSpeed():Float return row.speedRpm * 2 * Math.PI / 60;
  /** Output rotor inertia seen at the output flange: the motor rotor through the ratio squared, kg m2. */
  public function reflectedInertia():Float return (brake ? D.ROTOR_INERTIA_BRAKED : D.ROTOR_INERTIA) * ratio * ratio;

  /** The assumed or unpublished inputs, each with its value when one was entered. */
  public function unpublishedInputs():Array<ErobInput> return [for (u in D.unpublished()) if (u.name != "brakeHoldingTorque" || brake) {
    name: u.name,
    value: inputs.optionalNumber(u.name),
    unit: u.unit,
    basis: Assumed,
    reason: inputs.optionalNumber(u.name) == null ? u.reason + " (no assumption entered)" : "Assumed: " + u.reason
  }];

  function assumedValue(name:String):Null<Float> return inputs.optionalNumber(name);
  /** Permissible output-bearing moment, N m; null until an assumption is entered. */
  public function momentRating():Null<Float> return assumedValue("momentRating");
  public function bearingStiffness():Null<Float> return assumedValue("bearingStiffness");
  public function brakeHoldingTorque():Null<Float> return brake ? assumedValue("brakeHoldingTorque") : null;

  override public function motorDrive():Null<MotorDrive> return variant == HOUSING ? this : null;
  public function electricalRating():Null<ServoElectricalRating> return null;

  public function actuator(id:String, joint:String, volts:Float, margin:Float, ?current:Float):AssemblyActuator {
    if (variant != HOUSING) throw "Only the eRob housing drives a joint";
    if (volts < D.BUS_VOLTS * (1 - D.BUS_TOLERANCE) - 1e-9
      || volts > D.BUS_VOLTS * (1 + D.BUS_TOLERANCE) + 1e-9) throw 'eRob 110 runs on ${D.BUS_VOLTS} V +-10 %, not $volts V';
    var speed = ratedSpeed();
    return {
      id: id,
      joint: joint,
      maxEffort: row.peakTorque,
      maxRate: speed,
      rotorInertia: reflectedInertia(),
      drive: "servo",
      ratedTorque: row.ratedTorque,
      peakTorque: row.peakTorque,
      ratedSpeed: speed,
      maxSpeed: speed,
      torqueSpeed: [0, row.peakTorque, speed, row.ratedTorque],
      holdingTorque: brakeHoldingTorque(),
      encoderCounts: Math.pow(2, encoderBits),
      assumed: ["torque-speed interpolation between peak torque at standstill and rated torque at the table speed"],
      assumptions: [{quantity: "motor curve", label: "torque-speed interpolation; the drawing gives rated points only"}]
    };
  }

  /** The output encoder: absolute, `2^bits` counts per output turn, so it sees the load and no backlash is hidden. */
  public function encoder(id:String, joint:String):AssemblyEncoder return {
    id: id,
    joint: joint,
    kind: "absolute",
    counts: Math.pow(2, encoderBits)
  };

  public function busRequirement(port:String):DcBusRequirement {
    if (variant != HOUSING || port != "power") throw 'eRob has no DC bus port "$port"';
    return {
      nominalVolts: D.BUS_VOLTS,
      minimumVolts: D.BUS_VOLTS * (1 - D.BUS_TOLERANCE),
      maximumVolts: D.BUS_VOLTS * (1 + D.BUS_TOLERANCE),
      ratedPower: D.RATED_POWER,
      regenerates: true
    };
  }

  /** Raises when `supply` cannot feed this actuator: voltage outside its band, or less than its rated power. */
  public function checkSupply(supply:ElectricalSupplyRating):Void {
    var need = busRequirement("power"), volts = supply.voltage.volts();
    if (volts < need.minimumVolts - 1e-9 || volts > need.maximumVolts + 1e-9)
      throw 'eRob needs ${need.minimumVolts}-${need.maximumVolts} V DC, the supply gives $volts V';
    if (volts * supply.current.amps() + 1e-9 < need.ratedPower)
      throw 'eRob needs ${need.ratedPower} W DC, the supply gives ${volts * supply.current.amps()} W';
  }

  // Geometry -----------------------------------------------------------------------------------------------

  override public function hasGeometry():Bool return true;

  override public function geometry(detail:ComponentDetail = Preview):Part {
    var bore = Part.cylinderSpan(D.BORE_DIAMETER / 2, -D.BODY_LENGTH - 5, 5);
    if (variant == OUTPUT) {
      var disc = Solids.named(Part.cylinderSpan(D.OUTPUT_PILOT_DIAMETER / 2, SEAT_Z, 0), "pilot");
      if (detail == Envelope) return Solids.cut(disc, [Solids.named(bore, "bore")]);
      var screw = SocketHeadCapScrew.metric("M4", 10);
      var tools = [Solids.named(bore, "bore")];
      for (i in 0...D.OUTPUT_BOLT_COUNT) {
        var p = outputBolt(i);
        tools.push(Solids.named(Part.cylinderSpan(screw.spec.tapDrill / 2, -D.OUTPUT_THREAD_DEPTH, 0.1, p.x, p.y), 'outputBolt${i + 1}'));
      }
      return Solids.cut(disc, tools);
    }
    var body = Solids.union([
      Solids.named(Part.cylinderSpan(D.BODY_DIAMETER / 2, -D.BODY_LENGTH, SEAT_Z), "housing"),
      Solids.named(Part.cylinderSpan(D.REAR_SPIGOT_DIAMETER / 2, -D.BODY_LENGTH - D.REAR_SPIGOT_HEIGHT, -D.BODY_LENGTH), "rearSpigot")
    ]);
    if (detail == Envelope) return Solids.cut(body, [Solids.named(bore, "bore")]);
    var screw = SocketHeadCapScrew.metric("M4", 10);
    var tools = [Solids.named(bore, "bore")];
    for (i in 0...D.FIXED_BOLT_COUNT) {
      var p = fixedBolt(i);
      tools.push(Solids.named(Part.cylinderSpan(screw.spec.tapDrill / 2, SEAT_Z - D.FIXED_THREAD_DEPTH, SEAT_Z + 0.1, p.x, p.y), 'fixedBolt${i + 1}'));
    }
    return Solids.cut(body, tools);
  }

  override public function massGeometry():Null<Part> return null;

  /**
   * Tool for the part that receives this side, in this component's frame. Housing: the plate that holds it, which
   * wraps the housing's front `pocketDepth` (at least 8), passes the output pilot through a bore and carries
   * clearance holes for the 16 M4 screws; its top is `runningGap` below the output face so the turning flange clears
   * it. Output: the part bolted to the flange, ahead of z=0: 12 M4 clearance holes (Ø4.6 as drawn), and a bore for the
   * hollow axis that is the pilot's size plus `pilotClearance`.
   */
  public function mountingCutout(pocketDepth:Float = 8, runningGap:Float = 1, pilotClearance:Float = 0.4, depth:Float = 0):Part {
    if (!(pocketDepth >= D.POCKET_DEPTH_MIN) || !(runningGap >= 0) || !(pilotClearance >= 0)) throw "eRob mounting cutout needs a pocket of at least 8 mm and non-negative gaps";
    var tools:Array<Part> = [];
    if (variant == HOUSING) {
      var floor = SEAT_Z, bottom = floor - pocketDepth, top = -runningGap;
      var screw = SocketHeadCapScrew.metric("M4", 10);
      tools.push(Solids.named(Part.cylinderSpan((D.BODY_DIAMETER + 2 * D.BODY_DIAMETER_TOLERANCE + pilotClearance) / 2, bottom - 0.1, floor), "pocket"));
      tools.push(Solids.named(Part.cylinderSpan((D.OUTPUT_PILOT_DIAMETER + pilotClearance + 1) / 2, floor - 0.1, top + 0.1), "outputBore"));
      for (i in 0...D.FIXED_BOLT_COUNT) {
        var p = fixedBolt(i);
        tools.push(Solids.named(Part.cylinderSpan(screw.clearanceDiameter(Medium) / 2, floor - 0.1, top + 0.1, p.x, p.y), 'fixedBolt${i + 1}'));
      }
    } else {
      var thickness = depth > 0 ? depth : 10;
      for (i in 0...D.OUTPUT_BOLT_COUNT) {
        var p = outputBolt(i);
        tools.push(Solids.named(Part.cylinderSpan(D.OUTPUT_CLEARANCE_HOLE / 2, -0.1, thickness + 0.1, p.x, p.y), 'outputBolt${i + 1}'));
      }
      tools.push(Solids.named(Part.cylinderSpan((D.BORE_DIAMETER + pilotClearance) / 2, -0.1, thickness + 0.1), "bore"));
    }
    return Solids.union(tools);
  }

  override public function toolSpecs():Array<ToolSpec> return [new ToolSpec("mountingCutout", variant == HOUSING ? [
    P.length("pocketDepth", D.POCKET_DEPTH_MIN),
    P.length("runningGap", 1),
    P.length("pilotClearance", 0.4)
  ] : [P.toolDepth(10), P.length("pilotClearance", 0.4)])];

  override function buildTool(name:String, values:ComponentValues):Part {
    if (name == "mountingCutout") return variant == HOUSING ? mountingCutout(
      values.number("pocketDepth"),
      values.number("runningGap"),
      values.number("pilotClearance")
    ) : mountingCutout(D.POCKET_DEPTH_MIN, 1, values.number("pilotClearance"), values.number("depth"));
    return super.buildTool(name, values);
  }

  // Recipe -------------------------------------------------------------------------------------------------

  static var recipe:Null<ComponentType>;

  public static function recipeType():ComponentType {
    if (recipe == null) recipe = new ComponentType("machinekit.motion.erob-actuator-part", parameters(), v -> new ErobActuatorPart(v), true)
      .procured(_ -> Buy("ZeroErr", null, null)).requiring(v -> {
        var row = D.row(Std.parseInt(v.token("ratio")));
        var req = machinekit.component.Procurement.ProcurementCodec.requirement;
        var choice = machinekit.component.Procurement.ProcurementCodec.choice;
        return [req("ratio", "1", null, null, row.ratio), req("peakTorque", "N m", row.peakTorque),
          req("ratedTorque", "N m", row.ratedTorque), req("busVoltage", "V", null, null, D.BUS_VOLTS),
          choice("encoderBits", v.token("encoderBits")), choice("brake", v.boolean("brake") ? "true" : "false"),
          choice("bus", v.token("bus"))];
      });
    return recipe;
  }

  override public function componentType():Null<ComponentType> return recipeType();
}
