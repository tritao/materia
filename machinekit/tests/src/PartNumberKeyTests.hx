import machinekit.component.ComponentRecipeSupport;
import machinekit.component.ComponentType;
import machinekit.component.ComponentValues;
import machinekit.component.Dimension;
import machinekit.component.MachineComponent;
import machinekit.component.PartNumberLint;
import machinekit.transmission.TimingBeltProfile;

private class NumberedTestPart extends MachineComponent {
  static var recipe:Null<ComponentType>;
  final size:Float;
  public function new(size:Float) {
    super("TEST-UNUSED", "Test part", "steel");
    this.size = size;
  }
  public static function recipeType():ComponentType {
    if (recipe == null) recipe = new ComponentType("test.numbered", [ComponentRecipeSupport.length("size", 10)],
      v -> new NumberedTestPart(v.number("size")))
      .numbered(v -> 'PART-${Dimension.format(v.number("size"))}');
    return recipe;
  }
  override public function componentType():Null<ComponentType> return recipeType();
  override public function values():ComponentValues return new ComponentValues().setNumber("size", size)
    .setToken("material", materialSpec());
}

class PartNumberKeyTests {
  public static function main():Void run();
  public static function run():Void {
    var a = new NumberedTestPart(10), b = new NumberedTestPart(10.0001);
    if (a.bom.partNumber != "PART-10" || b.bom.partNumber != "PART-10")
      throw "Part numbers must be derived from resolved recipe values";
    if (a.bom.valuesKey == b.bom.valuesKey || a.geometryKey(Preview) == b.geometryKey(Preview))
      throw "Recipe keys must preserve changes below display precision";
    if (PartNumberLint.problems(a.bom.partNumber).length != 0
      || PartNumberLint.problems("test|size:10").length == 0
      || PartNumberLint.problems("PART-null").length == 0
      || PartNumberLint.problems("PART-1.23456").length == 0
      || PartNumberLint.problems("PART-1.23456MM").length == 0
      || PartNumberLint.problems("PART-D1.23456").length == 0
      || PartNumberLint.problems("10.08.02.00203").length != 0)
      throw "Part-number lint must report invalid identities";
    var rail = machinekit.motion.LinearRail.metric("HGH15CA", 420, machinekit.motion.LinearRail.NOMINAL);
    var measured = machinekit.motion.LinearRail.metric("HGH15CA", 420, machinekit.motion.LinearRail.MEASURED);
    if (rail.bom.partNumber != "HGR15R-420-HGH15CA" || measured.bom.partNumber != rail.bom.partNumber
      || rail.bom.valuesKey != measured.bom.valuesKey)
      throw "Rail purchase identity must ignore drawing fidelity";
    if (machinekit.motion.LinearRailBlock.metric("HGH15CA").bom.partNumber != "HGH15CA"
      || machinekit.motion.ScrewSupportUnit.bk12().bom.partNumber != "BK12"
      || new machinekit.motion.BallScrew(400).bom.partNumber != "SFU1605-L400-J50-UNSPEC"
      || new machinekit.motion.BallNut().bom.partNumber != "SFU1605-NUT-PRELOAD")
      throw "Catalogue identities must come from recipe values";
    var pulley = new machinekit.transmission.TimingPulley(HTD5M, 20, 8, 15);
    var otherPulley = new machinekit.transmission.TimingPulley(HTD5M, 20, 10, 15);
    if (pulley.bom.partNumber != "PULLEY-HTD5M-20T-B8-W15" || pulley.bom.partNumber == otherPulley.bom.partNumber)
      throw "Pulley identity must include its profile, tooth count, bore and width";
    var customPulley = machinekit.transmission.TimingPulley.customRecipeType().create().bom.partNumber;
    if (customPulley != "PULLEY-CUSTOM-P2-D0.254-20T-B5-W6" || PartNumberLint.problems(customPulley).length != 0)
      throw "Custom pulley identity must describe its pitch and pitch-line differential";
    var standardSprocket = machinekit.transmission.Sprocket.chainRecipeType().create().bom.partNumber;
    var genericSprocket = machinekit.transmission.Sprocket.genericRecipeType().create().bom.partNumber;
    if (standardSprocket != "SPROCKET-ANSI25-20T-B8-W5"
      || genericSprocket != "SPROCKET-ASSUMED-P6.35-20T-B8-W5-R3.969"
      || PartNumberLint.problems(standardSprocket).length != 0 || PartNumberLint.problems(genericSprocket).length != 0)
      throw "Sprocket identities must describe their chain or pitch and roller dimensions";
    var beltA = machinekit.transmission.TimingBelt.twoPulley(GT2, 20, 20, 100, 6);
    var beltB = machinekit.transmission.TimingBelt.twoPulley(GT2, 20, 20, 100.1, 6);
    if (beltA.bom.partNumber != "BELT-GT2-120T-W6" || beltA.bom.partNumber != beltB.bom.partNumber
      || beltA.bom.valuesKey != beltB.bom.valuesKey || beltA.geometryKey(Preview) == beltB.geometryKey(Preview))
      throw "Belt purchase identity must ignore layout while geometry keys retain it";
    var belts = new machinekit.component.Bom();
    belts.addComponent(beltA); belts.addComponent(beltB);
    if (belts.lines().length != 1 || belts.lines()[0].quantity != 2) throw "Equivalent belts must roll up";
    for (teeth in [12, 16, 24, 36]) {
      var unequal = machinekit.transmission.TimingBelt.twoPulley(HTD5M, 20, teeth, 150, 15);
      if (unequal.bom.partNumber != 'BELT-HTD5M-${unequal.teeth}T-W15')
        throw "Value-only belt tooth count must match the physical pitch loop";
    }
    var drawingScrew = machinekit.motion.HzMotionDrawingScrew.recipeType();
    var bk12 = drawingScrew.create().bom.partNumber;
    var bk15 = drawingScrew.create(new ComponentValues().setToken("endProfile", "BK15")).bom.partNumber;
    if (bk12 != "HZMOTION-DRAWING-BK12-L400" || bk15 != "HZMOTION-DRAWING-BK15-L400"
      || PartNumberLint.problems(bk12).length != 0 || PartNumberLint.problems(bk15).length != 0)
      throw 'Screw end study identities: $bk12, $bk15';
    var leadScrew = machinekit.motion.LeadScrew.recipeType().create().bom.partNumber;
    var leadNut = machinekit.motion.LeadScrewNut.recipeType().create().bom.partNumber;
    if (leadScrew != "SCREW-TR-D8-P2-S1-RH-L100" || leadNut != "NUT-TR-D8-P2-S1-RH-ANTI-4F"
      || PartNumberLint.problems(leadScrew).length != 0 || PartNumberLint.problems(leadNut).length != 0)
      throw "Lead screw and nut identities must describe the thread and interface";
    var ballValues = new ComponentValues().setToken("family", "Ball").setNumber("screwDiameter", 16)
      .setNumber("pitch", 5).setNumber("raceRootDiameter", 13);
    var ballScrew = machinekit.motion.LeadScrew.recipeType().create(ballValues).bom.partNumber;
    ballValues.setNumber("raceRootDiameter", 14);
    var otherRace = machinekit.motion.LeadScrew.recipeType().create(ballValues).bom.partNumber;
    if (ballScrew != "SCREW-BALL-D16-P5-S1-RH-R13-L100" || ballScrew == otherRace)
      throw "Ball screw identity must retain the specified race root";
    ballValues.setNumber("raceRootDiameter", 13).setToken("kind", "BallNut");
    var bronzeBallNut = machinekit.motion.LeadScrewNut.recipeType().create(ballValues);
    var steelBallNut = machinekit.motion.LeadScrewNut.recipeType().create(ballValues);
    steelBallNut.setMaterial("steel");
    if (steelBallNut.bom.partNumber != "NUT-BALL-D16-P5-S1-RH-R13-BALL-4F-STEEL"
      || bronzeBallNut.bom.partNumber == steelBallNut.bom.partNumber
      || PartNumberLint.problems(steelBallNut.bom.partNumber).length != 0)
      throw "A ball nut's selected material must be represented in its purchase identity";
    ballValues.setNumber("pitch", 10).setToken("material", "steel");
    var routerNut = machinekit.motion.LeadScrewNut.recipeType().create(ballValues).bom.partNumber;
    if (routerNut != "NUT-BALL-D16-P10-S1-RH-R13-BALL-4F-STEEL" || PartNumberLint.problems(routerNut).length != 0)
      throw 'The sheet router ball nut must retain a valid material-specific identity: $routerNut';
    var axisServo = new machinekit.motion.AxisServoMotor("GENERIC-HV-SERVO-750W");
    var servo = machinekit.motion.ServoMotor.model("GENERIC-HV-SERVO-750W");
    if (axisServo.bom.partNumber != "SERVO-ASSUMED-HV-SERVO-750W" || axisServo.bom.partNumber != servo.bom.partNumber
      || PartNumberLint.problems(axisServo.bom.partNumber).length != 0)
      throw "Generic servo representations must use the same assumed identity";
    var servos = new machinekit.component.Bom();
    servos.addComponent(axisServo); servos.addComponent(servo);
    if (servos.lines().length != 1 || servos.lines()[0].quantity != 2)
      throw "Servo representations must roll up to the same purchase";
    var bushing = machinekit.standard.Bushing.recipeType().create().bom.partNumber;
    var screw = machinekit.standard.SocketSetScrew.recipeType().create().bom.partNumber;
    var flange = machinekit.standard.HexFlangeBolt.recipeType().create().bom.partNumber;
    var stepper = machinekit.motion.NemaStepper.frame(17, 48).bom.partNumber;
    var chain = machinekit.motion.CableChain.recipeType().create().bom.partNumber;
    var vfd = machinekit.power.Vfd.recipeType().create().bom.partNumber;
    if (bushing != "BUSHING-8x12x10" || !StringTools.startsWith(screw, "ISO4026-M6")
      || flange != "ISO4162-F-FULL-HEAD-M6x20-8.8" || stepper != "STEPPER-ASSUMED-NEMA17-L48"
      || chain != "CHAIN-ASSUMED-40x25-R75-L1635.619" || vfd != "VFD-ASSUMED-7500W-400V-18A-400HZ-STO")
      throw "Values-based standard and assumed identities must retain their requirements";
    var steelBushing = machinekit.standard.Bushing.recipeType().create(new ComponentValues()
      .setToken("material", "steel")).bom.partNumber;
    if (steelBushing != "BUSHING-8x12x10-MACHINED-STEEL"
      || PartNumberLint.problems(steelBushing).length != 0)
      throw 'Bushing material must be part of the purchase identity: $steelBushing';
    var gearbox = machinekit.motion.Gearbox.recipeType().create().bom.partNumber;
    var supply = machinekit.motion.PowerSupply.recipeType().create().bom.partNumber;
    var adapter = machinekit.motion.MotorAdapterPlate.recipeType().create().bom.partNumber;
    var retainer = machinekit.motion.BallScrewRetainer.recipeType().create().bom.partNumber;
    if (gearbox != "GEARBOX-10-0.9-60x40-B8-STATED" || supply != "PSU-ASSUMED-24V10A-120x60x30-1O-0"
      || adapter != "HZMOTION-KK86-80EP-ADAPTER" || retainer != "BALLSCREW-D12-RETAINER-ASSUMED")
      throw "Gearbox, supply, adapter and retainer identities must use recipe values";
    var cobotSupply = machinekit.motion.PowerSupply.recipeType().create(new ComponentValues()
      .setNumber("voltage", 48).setNumber("current", 150).setInteger("outlets", 6)
      .setNumber("width", 240).setNumber("height", 120).setNumber("depth", 100)
      .setNumber("regenerativePower", 6000)).bom.partNumber;
    if (cobotSupply != "PSU-ASSUMED-48V150A-240x120x100-6O-6000"
      || PartNumberLint.problems(cobotSupply).length != 0) throw 'Cobot power supply identity: $cobotSupply';
    var routerSupply = machinekit.motion.PowerSupply.recipeType().create(new ComponentValues()
      .setNumber("voltage", 560).setNumber("current", 22.38).setInteger("outlets", 4)
      .setNumber("regenerativePower", 9000)).bom.partNumber;
    if (PartNumberLint.problems(routerSupply).length != 0) throw 'Router power supply identity: $routerSupply';
    var shaft = machinekit.motion.SteppedShaft.recipeType();
    var plainShaft = shaft.create().bom.partNumber;
    var groovedShaft = shaft.create(new ComponentValues().setToken("grooves",
      '[{"name":"ringA","z0":2,"width":1.1,"diameter":9.6}]')).bom.partNumber;
    if (plainShaft != "SHAFT-10x100" || groovedShaft != "SHAFT-10x100-G1"
      || PartNumberLint.problems(groovedShaft).length != 0) throw 'Stepped shaft identities: $plainShaft, $groovedShaft';
    var rectifier = machinekit.power.ThreePhaseRectifier.recipeType().create().bom.partNumber;
    if (rectifier != "RECT-ASSUMED-400V-50HZ-20A-4O-8000W" || PartNumberLint.problems(rectifier).length != 0)
      throw "Rectifier identity must include the input and output requirements";
    var air = machinekit.pneumatic.AirSupply.recipeType().create().bom.partNumber;
    var valve = machinekit.pneumatic.SolenoidValve.recipeType().create().bom.partNumber;
    var fitting = machinekit.pneumatic.PushInFitting.recipeType().create().bom.partNumber;
    if (air != "AIR-FRL-ASSUMED-600000PA" || valve != "VALVE-ASSUMED-5/2-SINGLE-NB"
      || fitting != "10.08.02.00203" || PartNumberLint.problems(fitting).length != 0)
      throw "Pneumatic identities must distinguish requirements from catalogue designations";
    var chainA = new machinekit.gantry.GantryCableChain(new machinekit.motion.CableChain(), "base", "x", "x", 0);
    var chainB = new machinekit.gantry.GantryCableChain(new machinekit.motion.CableChain(), "frame", "y", "y", 100);
    if (chainA.bom.partNumber != chain || chainB.bom.partNumber != chain || chainA.bom.valuesKey != chainB.bom.valuesKey)
      throw "Chain routing must not change its purchase identity";
    var chains = new machinekit.component.Bom();
    chains.addComponent(chainA); chains.addComponent(chainB);
    if (chains.lines().length != 1 || chains.lines()[0].quantity != 2) throw "Equivalent routed chains must roll up";
    var electrical = [machinekit.power.BrakeResistor.recipeType().create().bom.partNumber,
      machinekit.power.CircuitBreaker.recipeType().create().bom.partNumber,
      machinekit.power.Contactor.recipeType().create().bom.partNumber,
      machinekit.control.SafetyRelay.recipeType().create().bom.partNumber];
    var expected = ["BRK-RESISTOR-ASSUMED-80OHM-1000W-15000J", "BREAKER-ASSUMED-32A-400V-6000A-3P",
      "CONTACTOR-ASSUMED-32A-400V-24V-3P", "SAFETY-RELAY-ASSUMED-24V-2S-2I-2D"];
    for (i in 0...electrical.length) if (electrical[i] != expected[i] || PartNumberLint.problems(electrical[i]).length != 0)
      throw 'Electrical identity from recipe values: ${electrical[i]}';
    var routing = [machinekit.routing.AtcSpindle.recipeType().create().bom.partNumber,
      machinekit.routing.Collet.recipeType().create().bom.partNumber,
      machinekit.routing.ToolHolder.recipeType().create().bom.partNumber,
      machinekit.routing.ToolLengthSetter.recipeType().create().bom.partNumber,
      machinekit.routing.VacuumBlower.recipeType().create().bom.partNumber,
      machinekit.routing.VacuumZoneValve.recipeType().create().bom.partNumber,
      machinekit.motion.PlanetaryReducer.recipeType().create().bom.partNumber,
      machinekit.motion.BrakeServo.recipeType().create().bom.partNumber];
    var routingExpected = ["ATC-SPINDLE-ASSUMED-6000W-24000RPM", "COLLET-ASSUMED-ER32-8MM",
      "ISO30-ER32-HOLDER-ASSUMED-L60", "TOOL-SETTER-ASSUMED-40x60-S5-R0.005",
      "VAC-BLOWER-ASSUMED-5500W-300M3H-25000PA", "VAC-VALVE-ASSUMED-50MM-40000PA-300M3H",
      "PLANETARY-ASSUMED-5R-80NM-5AM-19B", "SERVO-BRAKE-ASSUMED-400W-2.5NM-35MM"];
    for (i in 0...routing.length) if (routing[i] != routingExpected[i] || PartNumberLint.problems(routing[i]).length != 0)
      throw 'Routing identity from recipe values: ${routing[i]}';
    var pinionMount = machinekit.transmission.PreloadedPinionMount.recipeType().create().bom.partNumber;
    if (pinionMount != "PIN-MOUNT-ASSUMED-PLATE-700N" || PartNumberLint.problems(pinionMount).length != 0)
      throw 'Pinion-mount identity: $pinionMount';
    var additional = [machinekit.motion.LimitSwitch.recipeType().create().bom.partNumber,
      machinekit.motion.ProximitySwitch.recipeType().create().bom.partNumber,
      machinekit.motion.SfuNut.familyRecipeType().create().bom.partNumber,
      machinekit.motion.SfuDrawingScrew.drawingRecipeType().create().bom.partNumber,
      machinekit.transmission.HelicalRack.recipeType().create().bom.partNumber,
      machinekit.transmission.HelicalPinion.recipeType().create().bom.partNumber];
    var additionalExpected = ["SWITCH-ASSUMED-ROLLER-MICROSWITCH", "SWITCH-ASSUMED-INDUCTIVE-M8",
      "SFU1605-NUT-R13", "SFU2005-DRAWING-L500-R16",
      "RACK-ASSUMED-M1.5-20T-W20-H0.341-L", "PIN-ASSUMED-M1.5-20T-B14-H0.341-R"];
    for (i in 0...additional.length) if (additional[i] != additionalExpected[i]
      || PartNumberLint.problems(additional[i]).length != 0) throw 'Identity from recipe values: ${additional[i]}';
    var precision = [machinekit.motion.AngularContactBearing.recipeType().create().bom.partNumber,
      machinekit.motion.CrossedRollerBearing.recipeType().create().bom.partNumber,
      machinekit.motion.TorqueMotor.recipeType().create().bom.partNumber,
      machinekit.motion.RotaryEncoderRing.recipeType().create().bom.partNumber,
      machinekit.pneumatic.PneumaticCylinder.recipeType().create().bom.partNumber];
    var precisionExpected = ["AC-BEARING-ASSUMED-25x62x17-P4", "CR-BEARING-ASSUMED-80x120x16-P4",
      "TORQUE-MOTOR-ASSUMED-10NM-30NM", "ENCODER-ASSUMED-8388608CPR-100B", "ISO6432-25x100-ASSUMED-300MMS"];
    for (i in 0...precision.length) if (precision[i] != precisionExpected[i]
      || PartNumberLint.problems(precision[i]).length != 0) throw 'Precision component identity: ${precision[i]}';
    var printing = [machinekit.printing.Thermistor.recipeType().create().bom.partNumber,
      machinekit.printing.HeaterCartridge.recipeType().create().bom.partNumber,
      machinekit.printing.Nozzle.recipeType().create().bom.partNumber,
      machinekit.printing.Hotend.recipeType().create().bom.partNumber,
      machinekit.printing.Extruder.recipeType().create().bom.partNumber,
      machinekit.printing.HeatedBed.recipeType().create().bom.partNumber,
      machinekit.printing.Fan.recipeType().create().bom.partNumber];
    var printingExpected = ["NTC-100K-B3950", "CARTRIDGE-6x20-24V-40W", "BRASS-M6-0.4",
      "GENERIC-V6-STYLE", "GEARED-HOB-7.3", "BED-ALU-235-24V-300W", "FAN-4010-24V"];
    for (i in 0...printing.length) if (printing[i] != printingExpected[i]
      || PartNumberLint.problems(printing[i]).length != 0) throw 'Printing component identity: ${printing[i]}';
    var controls = [machinekit.hmi.HandWheel.recipeType().create().bom.partNumber,
      machinekit.hmi.EmergencyStop.recipeType().create().bom.partNumber,
      machinekit.hmi.RotarySwitch.recipeType().create().bom.partNumber,
      machinekit.hmi.PushButton.recipeType().create().bom.partNumber,
      machinekit.hmi.OledModule.recipeType().create().bom.partNumber];
    var controlsExpected = ["YUMO-ISM6045-100B", "XB2-ES542-1NC", "ROTARY-SWITCH-ASSUMED-12POS",
      "PUSHBUTTON-ASSUMED-WHITE-LIT", "WAVESHARE-25742"];
    for (i in 0...controls.length) if (controls[i] != controlsExpected[i]
      || PartNumberLint.problems(controls[i]).length != 0) throw 'Operator control identity: ${controls[i]}';
    var controllers = [machinekit.control.CncController.recipeType().create().bom.partNumber,
      machinekit.control.RobotController.recipeType().create().bom.partNumber,
      machinekit.control.DigitalSensor.recipeType().create().bom.partNumber,
      machinekit.control.ThermistorInput.recipeType().create().bom.partNumber];
    var controllersExpected = ["CNC-CTRL-ASSUMED-4A-16I-16O-8V", "ROBOT-CTRL-ASSUMED-6A-16I-16O-4V",
      "SENSOR-ASSUMED-REED", "NTC-ADC-ASSUMED-2CH-4700R-3.3V"];
    for (i in 0...controllers.length) if (controllers[i] != controllersExpected[i]
      || PartNumberLint.problems(controllers[i]).length != 0) throw 'Controller identity: ${controllers[i]}';
    var source = machinekit.tooling.ToolingSourceCasting.fromInput({
      id: "recipe-unit-sample", revision: 1,
      lengthUnit: materia.tooling.ToolingEnums.ToolingLengthUnit.Millimetres,
      datumScheme: null, interfaces: [], interfaceContext: null, postCureInserts: [], liftingPoints: [],
      envelope: [{id: "body", height: 1, pose: materia.assembly.AssemblyFrames.identity(),
        profile: [{x: -0.5, y: -0.5}, {x: 0.5, y: -0.5}, {x: 0.5, y: 0.5}, {x: -0.5, y: 0.5}]}],
      features: [], inserts: []
    }).bom.partNumber;
    if (source != "CASTING-RECIPE-UNIT-SAMPLE" || PartNumberLint.problems(source).length != 0)
      throw 'Source casting identity: $source';
    var toolingPanel = machinekit.tooling.ToolingPanel.recipeType().create().bom.partNumber;
    var toolingVolume = machinekit.tooling.ToolingVolume.recipeType().create().bom.partNumber;
    if (toolingPanel != "TOOL-PANEL-PANEL" || toolingVolume != "MOLD-FORMER"
      || PartNumberLint.problems(toolingPanel).length != 0 || PartNumberLint.problems(toolingVolume).length != 0)
      throw 'Tooling drawing identities: $toolingPanel, $toolingVolume';
    var toolingPin = machinekit.tooling.ToolingPin.recipeType();
    var plainPin = toolingPin.create().bom.partNumber;
    var headedPin = toolingPin.create(new ComponentValues().setNumber("headDiameter", 10).setNumber("headLength", 5)).bom.partNumber;
    if (plainPin != "TOOL-PIN-6x40" || headedPin != "TOOL-PIN-6x40-H10x5"
      || PartNumberLint.problems(plainPin).length != 0 || PartNumberLint.problems(headedPin).length != 0)
      throw 'Tooling pin identities: $plainPin, $headedPin';
    var pressParts = [machinekit.pressbrake.ToolClamp.recipeType().create().bom.partNumber,
      machinekit.pressbrake.GaugeFinger.recipeType().create().bom.partNumber,
      machinekit.pressbrake.ToolSegment.recipeType().create().bom.partNumber,
      machinekit.pressbrake.ToolSegment.recipeType().create(new ComponentValues().setToken("horn", "Left")).bom.partNumber];
    var pressExpected = ["TOOL-CLAMP-MANUAL-50x40x60", "PRESS-GAUGE-FINGER-ASSUMED",
      "TOOL-STRAIGHT-88-R06-L100", "TOOL-STRAIGHT-88-R06-L100-L20x25"];
    for (i in 0...pressParts.length) if (pressParts[i] != pressExpected[i]
      || PartNumberLint.problems(pressParts[i]).length != 0) throw 'Press tooling identity: ${pressParts[i]}';
    var pneumaticRod = machinekit.pneumatic.PneumaticCylinderRod.recipeType().create().bom.partNumber;
    var hydraulicRod = machinekit.pressbrake.HydraulicCylinderRod.recipeType().create().bom.partNumber;
    if (pneumaticRod != "PNEUMATIC-ROD-10x115" || hydraulicRod != "HYDRAULIC-ROD-100x190"
      || PartNumberLint.problems(pneumaticRod).length != 0 || PartNumberLint.problems(hydraulicRod).length != 0)
      throw 'Cylinder rod identities: $pneumaticRod, $hydraulicRod';
    var hydraulics = [machinekit.pressbrake.HydraulicCylinder.recipeType().create().bom.partNumber,
      machinekit.pressbrake.HydraulicSupply.recipeType().create().bom.partNumber,
      machinekit.pressbrake.HydraulicValve.recipeType().create().bom.partNumber,
      machinekit.pressbrake.HydraulicManifold.recipeType().create().bom.partNumber];
    var hydraulicsExpected = ["HYD-CYL-ASSUMED-160x100x100", "HYD-SUPPLY-ASSUMED-0PA",
      "HYD-VALVE-ASSUMED-4/3-CLOSED", "HYD-MANIFOLD-ASSUMED-2OUT"];
    for (i in 0...hydraulics.length) if (hydraulics[i] != hydraulicsExpected[i]
      || PartNumberLint.problems(hydraulics[i]).length != 0) throw 'Hydraulic identity: ${hydraulics[i]}';
    var cutter = machinekit.milling.FlatEndMill.recipeType().create();
    var retractedCutter = machinekit.milling.FlatEndMill.recipeType().create(new ComponentValues().setNumber("stickout", 30));
    if (cutter.bom.partNumber != "ENDMILL-ASSUMED-D6-F22-S35" || cutter.bom.partNumber == retractedCutter.bom.partNumber
      || PartNumberLint.problems(cutter.bom.partNumber).length != 0)
      throw "Cutter identity must retain the modeled reach while overall tool length is unknown";
    var spindle = [machinekit.milling.SpindleMotor.recipeType().create().bom.partNumber,
      machinekit.milling.SpindleCartridge.recipeType().create().bom.partNumber,
      machinekit.milling.SpindleCartridge.Er20Holder.recipeType().create().bom.partNumber,
      machinekit.milling.Bt30Cartridge.recipeType().create().bom.partNumber,
      machinekit.milling.Bt30SpindleMotor.recipeType().create().bom.partNumber];
    var spindleExpected = ["SPINDLE-MOTOR-ASSUMED-1100W-LV", "SPINDLE-CARTRIDGE-ASSUMED-ER20-D70",
      "HOLDER-ASSUMED-ER20-D6", "BT30-CARTRIDGE-ASSUMED-D90", "BT30-MOTOR-ASSUMED-2200W-8000RPM"];
    for (i in 0...spindle.length) if (spindle[i] != spindleExpected[i]
      || PartNumberLint.problems(spindle[i]).length != 0) throw 'Spindle identity: ${spindle[i]}';
    var smallCobot = new machinekit.robotics.CobotJoint(0);
    var steelGearbox = smallCobot.gearbox.bom.partNumber;
    if (steelGearbox != "GEARBOX-100-0.85-39x24-B8-ASSUMED-STL" || PartNumberLint.problems(steelGearbox).length != 0)
      throw 'Cobot gearbox material identity: $steelGearbox';
    var robotParts = [machinekit.robotics.GearedArmJoint.recipeType().create().bom.partNumber,
      machinekit.robotics.EndEffectorPlate.recipeType().create().bom.partNumber,
      machinekit.robotics.Pedestal.recipeType().create().bom.partNumber,
      machinekit.robotics.ArmWristFork.recipeType().create().bom.partNumber];
    var robotExpected = ["ARM-JOINT-100x70-P61x36", "EOAT-40-4-12-65-4M5", "PEDESTAL-F40-D60-L150", "ARM-FORK-60x70-W8-D24"];
    for (i in 0...robotParts.length) if (robotParts[i] != robotExpected[i]
      || PartNumberLint.problems(robotParts[i]).length != 0) throw 'Robot part identity: ${robotParts[i]}';
    var cobot = machinekit.robotics.CobotJoint.moduleRecipeType().create().bom.partNumber;
    if (cobot != "COBOT-ASSUMED-S2-180DPS-P0-NN" || PartNumberLint.problems(cobot).length != 0)
      throw 'Cobot joint identity: $cobot';
    var finger = machinekit.robotics.OffsetGripperFinger.family().recipe;
    var leftFinger = finger.create().bom.partNumber;
    var rightFinger = finger.create(new ComponentValues().setToken("side", "right").setToken("variant", "sls-pa12")).bom.partNumber;
    if (leftFinger != "OFFSET-FINGER-L-CNC-ALUMINIUM" || rightFinger != "OFFSET-FINGER-R-SLS-PA12"
      || PartNumberLint.problems(leftFinger).length != 0 || PartNumberLint.problems(rightFinger).length != 0)
      throw 'Finger manufacturing variant identities: $leftFinger, $rightFinger';
    var flangeType = machinekit.robotics.RobotFlange.recipeType();
    var robotFlangeNumber = flangeType.create().bom.partNumber;
    var nonstandardFlange = flangeType.create(new ComponentValues().setInteger("boltCount", 3)).bom.partNumber;
    if (robotFlangeNumber != "ISO9409-STYLE-40-4-M6" || nonstandardFlange != "FLANGE-40-3-M6"
      || PartNumberLint.problems(robotFlangeNumber).length != 0 || PartNumberLint.problems(nonstandardFlange).length != 0)
      throw 'Robot flange identities: $robotFlangeNumber, $nonstandardFlange';
    var millParts = [machinekit.milling.MillMotorCover.recipeType().create().bom.partNumber,
      machinekit.milling.MillZBearingPlate.recipeType().create().bom.partNumber,
      machinekit.milling.MillYNutBridge.recipeType().create().bom.partNumber,
      machinekit.milling.MillScrewPedestal.recipeType().create().bom.partNumber,
      machinekit.milling.MillServoMount.recipeType().create().bom.partNumber,
      machinekit.milling.MillDriveHatch.recipeType().create().bom.partNumber];
    var millPartsExpected = ["MILL-MOTOR-HOOD", "MILL-Z-BEARING-PLATE", "MILL-Y-NUT-BRIDGE",
      "MILL-SCREW-PEDESTAL", "MILL-SERVO-MOUNT", "MILL-Y-DRIVE-HATCH"];
    for (i in 0...millParts.length) if (millParts[i] != millPartsExpected[i]
      || PartNumberLint.problems(millParts[i]).length != 0) throw 'Mill recipe identity: ${millParts[i]}';
    var moreMillParts = [machinekit.milling.MillZCarriage.recipeType().create().bom.partNumber,
      machinekit.milling.MillZHead.recipeType().create().bom.partNumber,
      machinekit.milling.MillCrossSaddle.recipeType().create().bom.partNumber,
      machinekit.milling.MillSpindleBeltGuard.recipeType().create().bom.partNumber,
      machinekit.milling.MillSupportTray.recipeType().create().bom.partNumber,
      machinekit.milling.MillBallNutBracket.recipeType().create().bom.partNumber,
      machinekit.milling.MillZCouplingGuard.recipeType().create().bom.partNumber,
      machinekit.milling.MillZDriveAdapter.recipeType().create().bom.partNumber,
      machinekit.milling.MillZMotorStand.recipeType().create().bom.partNumber];
    var moreMillExpected = ["MILL-Z-CARRIAGE", "MILL-Z-RIBBED-HEAD", "MILL-CROSS-SADDLE",
      "MILL-SPINDLE-BELT-GUARD", "MILL-SUPPORT-TRAY", "MILL-BALL-NUT-BRACKET",
      "MILL-Z-COUPLING-GUARD", "MILL-Z-DRIVE-ADAPTER", "MILL-Z-MOTOR-STAND"];
    for (i in 0...moreMillParts.length) if (moreMillParts[i] != moreMillExpected[i]
      || PartNumberLint.problems(moreMillParts[i]).length != 0) throw 'Mill recipe identity: ${moreMillParts[i]}';
    var zppType = machinekit.milling.ZppComponent.recipeType();
    var zppBody = zppType.create().bom.partNumber;
    var zppJaw = zppType.create(new ComponentValues().setToken("role", "ar-negative-soft-jaw")).bom.partNumber;
    if (zppBody != "ZPP-CV15075-BODY" || zppJaw != "ZPP-AR155-I-AR-NEGATIVE-SOFT-JAW"
      || PartNumberLint.problems(zppBody).length != 0 || PartNumberLint.problems(zppJaw).length != 0)
      throw 'ZPP member identities: $zppBody, $zppJaw';
    var panelType = machinekit.milling.MillPanel.recipeType();
    var steelPanel = panelType.create().bom.partNumber;
    var aluminiumPanel = panelType.create(new ComponentValues().setToken("material", "aluminium 6061")).bom.partNumber;
    var toolingBlock = machinekit.tooling.ToolingBlock.recipeType().create().bom.partNumber;
    if (steelPanel != "MILL-PANEL-100x2x100-MACHINED-STEEL"
      || aluminiumPanel != "MILL-PANEL-100x2x100-ALUMINIUM"
      || toolingBlock != "TOOL-BLOCK-100x80x50-NEUTRAL"
      || PartNumberLint.problems(steelPanel).length != 0 || PartNumberLint.problems(aluminiumPanel).length != 0
      || PartNumberLint.problems(toolingBlock).length != 0)
      throw 'Panel or block identities: $steelPanel, $aluminiumPanel, $toolingBlock';
    var fixture = machinekit.milling.MillFixtureAdapter.recipeType();
    var fixture82 = fixture.create().bom.partNumber;
    var fixture128 = fixture.create(new ComponentValues().setToken("pattern", "128-square")).bom.partNumber;
    if (fixture82 != "MILL-FIXTURE-ADAPTER-82" || fixture128 != "MILL-FIXTURE-ADAPTER-128"
      || PartNumberLint.problems(fixture82).length != 0 || PartNumberLint.problems(fixture128).length != 0)
      throw 'Fixture adapter identities: $fixture82, $fixture128';
    var casting = machinekit.milling.MillCasting.recipeType();
    var castingBase = casting.create().bom.partNumber;
    var castingHead = casting.create(new ComponentValues().setToken("kind", "Head")).bom.partNumber;
    if (castingBase != "MILL-BASE" || castingHead != "MILL-HEAD"
      || PartNumberLint.problems(castingBase).length != 0 || PartNumberLint.problems(castingHead).length != 0)
      throw 'Mill casting identities: $castingBase, $castingHead';
    var studyParts = [machinekit.milling.MillPowerDrawbarPart.recipeType().create().bom.partNumber,
      machinekit.milling.Rke92VisePart.recipeType().create().bom.partNumber,
      machinekit.milling.MillSpindleDrivePart.recipeType().create().bom.partNumber,
      machinekit.milling.MillSpindleDrivePart.recipeType("guard").create().bom.partNumber];
    var studyPartsExpected = ["BT30-STUDY-DRAWBAR", "RKE92-REFERENCE-BODY", "MILL-SPINDLE-DRIVE-CARRIER",
      "MILL-SPINDLE-DRIVE-GUARD"];
    for (i in 0...studyParts.length) if (studyParts[i] != studyPartsExpected[i]
      || PartNumberLint.problems(studyParts[i]).length != 0) throw 'Mill study identity: ${studyParts[i]}';
    var driveParts = [machinekit.milling.MillSpindleMotorMount.recipeType().create().bom.partNumber,
      machinekit.milling.MillYNutMount.recipeType().create().bom.partNumber,
      machinekit.milling.MillYDriveSeat.recipeType().create().bom.partNumber,
      machinekit.milling.MillYDriveSeat.recipeType().create(new ComponentValues().setBoolean("floating", true)).bom.partNumber,
      machinekit.milling.MillSpindleMotorSupport.recipeType().create().bom.partNumber,
      machinekit.milling.MillBeltMotorPlate.recipeType().create().bom.partNumber,
      machinekit.milling.MillCrossTable.recipeType().create().bom.partNumber];
    var drivePartsExpected = ["MILL-SPINDLE-MOTOR-CARRIER", "MILL-Y-NUT-MOUNT", "MILL-Y-BK12-C-SEAT",
      "MILL-Y-BF12-SEAT", "MILL-SPINDLE-MOTOR-SUPPORT", "MILL-X-RETURN-PLATE-ELM2H-0400LA60F", "MILL-CROSS-TABLE"];
    for (i in 0...driveParts.length) if (driveParts[i] != drivePartsExpected[i]
      || PartNumberLint.problems(driveParts[i]).length != 0) throw 'Mill drive identity: ${driveParts[i]}';
    var bridge = machinekit.milling.MillYNutBridge.recipeType();
    var bridgeA = bridge.create(), bridgeB = bridge.create(new ComponentValues().setNumber("rise", 130));
    if (bridgeA.bom.partNumber != bridgeB.bom.partNumber || bridgeA.bom.valuesKey == bridgeB.bom.valuesKey)
      throw "Changing a mill bridge dimension retains its declared identity and changes its recipe key";
    var brakePart = machinekit.motion.ShaftBrakePart.recipeType().create().bom.partNumber;
    var housing = machinekit.motion.FlangeBearingHousing.recipeType().create().bom.partNumber;
    if (brakePart != "BRAKE-ASSUMED-4NM-B19-SHELL-0" || housing != "FLANGE-HOUSING-608-SLIP"
      || PartNumberLint.problems(brakePart).length != 0 || PartNumberLint.problems(housing).length != 0)
      throw 'Brake or bearing-housing identity: $brakePart, $housing';
    var foot = machinekit.structural.LevelingFoot.recipeType().create().bom.partNumber;
    if (foot != "FOOT-ASSUMED-M20-D100-L100" || PartNumberLint.problems(foot).length != 0)
      throw 'Leveling foot identity: $foot';
    var thread = machinekit.standard.FastenerThreads.metric(20, 2.5);
    var footParts = [machinekit.structural.LevelingFoot.of(Pad(100, 20)).bom.partNumber,
      machinekit.structural.LevelingFoot.of(Screw(thread, 100, Nominal)).bom.partNumber,
      machinekit.structural.LevelingFoot.of(Plate(80, 80, 10, 21)).bom.partNumber,
      machinekit.structural.LevelingFoot.of(Nut(Hexagon(30), 15, thread)).bom.partNumber];
    var footPartsExpected = ["FOOT-PAD-ASSUMED-100x20", "FOOT-SCREW-ASSUMED-M20-L100",
      "FOOT-PLATE-ASSUMED-80x80x10-D21", "FOOT-NUT-ASSUMED-M20-H30x15"];
    for (i in 0...footParts.length) if (footParts[i] != footPartsExpected[i]
      || PartNumberLint.problems(footParts[i]).length != 0) throw 'Leveling foot part identity: ${footParts[i]}';
    var panel = machinekit.enclosure.EnclosurePanel.recipeType().create().bom.partNumber;
    var cutPanel = machinekit.enclosure.EnclosureCutPanel.cutRecipeType().create().bom.partNumber;
    if (panel != "P-FR-ACM-4-600x800" || cutPanel != "P-STEEL-2-600x800-R100x100-X0-Z400"
      || PartNumberLint.problems(panel).length != 0 || PartNumberLint.problems(cutPanel).length != 0)
      throw 'Enclosure panel identities: $panel, $cutPanel';
    var pendantPlate = machinekit.pendant.PendantCasePlate.recipeType().create().bom.partNumber;
    if (pendantPlate != "PENDANT-PLATE" || PartNumberLint.problems(pendantPlate).length != 0)
      throw 'Pendant plate identity: $pendantPlate';
    var drawnProfile = machinekit.structural.ProfileMember.drawingRecipeType().create().bom.partNumber;
    if (drawnProfile != "DRAWN-SECTION-20x20-L100" || PartNumberLint.problems(drawnProfile).length != 0)
      throw 'Drawing-profile identity: $drawnProfile';
    var slotProfile = machinekit.structural.ProfileMember.tslotRecipeType();
    var steelProfile = slotProfile.create().bom.partNumber;
    var aluminiumProfile = slotProfile.create(new ComponentValues().setToken("material", "aluminium 6061")).bom.partNumber;
    if (steelProfile != "TSLOT-40-SLOT8-40x40-L100" || aluminiumProfile != "TSLOT-40-SLOT8-40x40-L100-ALUMINIUM"
      || PartNumberLint.problems(steelProfile).length != 0 || PartNumberLint.problems(aluminiumProfile).length != 0)
      throw 'T-slot profile identities: $steelProfile, $aluminiumProfile';
    var insert = machinekit.structural.CastInsert.recipeType().create(new ComponentValues().setToken("role", "generic-mount")).bom.partNumber;
    var cassette = machinekit.structural.LinearMountNutCassette.recipeType().create(new ComponentValues()
      .setToken("sourceFingerprint", haxe.crypto.Sha256.encode("metadata-only; derive production source from MountedElement"))
      .setToken("stations", '[{"connector":"metadata-example","x":0,"y":0}]')).bom.partNumber;
    if (insert != "CAST-INSERT-RAIL-SEAT" || cassette != "NUT-CASSETTE-M6-END-SERVICE"
      || PartNumberLint.problems(insert).length != 0 || PartNumberLint.problems(cassette).length != 0)
      throw 'Source-dependent mount identities: $insert, $cassette';
    var insertRecipe = machinekit.structural.CastInsert.recipeType();
    var leftInsert = insertRecipe.create(new ComponentValues().setToken("role", "generic-mount")
      .setToken("partId", "HOBBER-railLeft")).bom.partNumber;
    var rightInsert = insertRecipe.create(new ComponentValues().setToken("role", "generic-mount")
      .setToken("partId", "HOBBER-railRight")).bom.partNumber;
    if (leftInsert != "CAST-INSERT-HOBBER-RAILLEFT" || rightInsert != "CAST-INSERT-HOBBER-RAILRIGHT"
      || leftInsert == rightInsert) throw "Source member IDs must distinguish inserts with the same label";
    for (id in ["HOBBER-LIFT-ONE", "HOBBER-COLUMN-LIFT-TWO", "MILL-LIFT-ONE"]) {
      var number = insertRecipe.create(new ComponentValues().setToken("role", "lifting-mount")
        .setToken("partId", id)).bom.partNumber;
      if (PartNumberLint.problems(number).length != 0) throw 'Lifting insert identity: $number';
    }
    var bedSeat = "", columnSeat = "";
    for (row in machinekit.hobbing.FoundationLayout.inserts("bed")) if (row.id == "railLeft") bedSeat = row.part.bom.partNumber;
    for (row in machinekit.hobbing.FoundationLayout.inserts("column")) if (row.id == "railLeft") columnSeat = row.part.bom.partNumber;
    if (bedSeat != "CAST-INSERT-HOBBER-BED-RAILLEFT" || columnSeat != "CAST-INSERT-HOBBER-COLUMN-RAILLEFT"
      || bedSeat == columnSeat) throw "Foundation scope must distinguish repeated local insert IDs";
    var structural = [machinekit.structural.RectTubeMember.recipeType().create().bom.partNumber,
      machinekit.structural.BlockBracket.recipeType().create().bom.partNumber,
      machinekit.structural.SlideInSlotNut.recipeType().create().bom.partNumber,
      machinekit.structural.MineralCasting.recipeType().create().bom.partNumber,
      machinekit.structural.MountDrilledPlate.recipeType().create().bom.partNumber];
    var structuralExpected = ["TUBE-30x30x2-L750", "BRACKET-BRACKET", "SLIDE-M6-15.9x15.5x5.9-R8x1.9",
      "MINERAL-650x550x120", "HOST-PAD-100x500x12-MACHINED-STEEL"];
    for (i in 0...structural.length) if (structural[i] != structuralExpected[i]
      || PartNumberLint.problems(structural[i]).length != 0) throw 'Structural identity: ${structural[i]}';
    var leftTube = new machinekit.structural.RectTubeMember(100, 100, 6, 750, null, "RACK-left");
    var rightTube = new machinekit.structural.RectTubeMember(100, 100, 6, 750, null, "RACK-right");
    if (leftTube.bom.partNumber != "TUBE-RACK-LEFT" || rightTube.bom.partNumber != "TUBE-RACK-RIGHT"
      || leftTube.bom.partNumber == rightTube.bom.partNumber)
      throw "Fabricated tube identities must retain their source member IDs";
    var padRecipe = machinekit.structural.MountDrilledPlate.recipeType();
    var leftPad = padRecipe.create(new ComponentValues().setToken("partId", "RACK-left")).bom.partNumber;
    var rightPad = padRecipe.create(new ComponentValues().setToken("partId", "RACK-right")).bom.partNumber;
    if (leftPad != "HOST-PAD-RACK-LEFT" || rightPad != "HOST-PAD-RACK-RIGHT" || leftPad == rightPad)
      throw "Drilled host pads must retain their source member IDs";
    var mountSteel = machinekit.structural.LinearMountSteel.recipeType();
    var mountParts = [mountSteel.create().bom.partNumber,
      mountSteel.create(new ComponentValues().setToken("stock", "weldable-mild-steel").setToken("role", "adjustment-bar")).bom.partNumber,
      machinekit.structural.LinearMountCleat.recipeType().create().bom.partNumber,
      machinekit.structural.LinearMountReplication.recipeType().create().bom.partNumber];
    var mountExpected = ["LM-C45-RECEIVER-28x500x20", "LM-MILD-ADJUSTMENT-BAR-28x500x20", "LINEAR-MOUNT-CLEAT-H32", "LM-REPLICATION-140x540x4"];
    for (i in 0...mountParts.length) if (mountParts[i] != mountExpected[i]
      || PartNumberLint.problems(mountParts[i]).length != 0) throw 'Mount part identity: ${mountParts[i]}';
    var replication = machinekit.structural.LinearMountReplication.recipeType();
    var hole = machinekit.structural.InsertHoles.records(machinekit.structural.InsertHoles.through(6.6, 4))[0];
    hole.id = "source-a";
    var firstHole = machinekit.structural.InsertHoles.fromNeutral([hole]);
    hole.id = "source-b";
    var secondHole = machinekit.structural.InsertHoles.fromNeutral([hole]);
    hole.frame.x = 0.1;
    var changedHole = machinekit.structural.InsertHoles.fromNeutral([hole]);
    var replicationA = new ComponentValues().setToken("holes", firstHole);
    var replicationB = new ComponentValues().setToken("holes", secondHole);
    var replicationChanged = new ComponentValues().setToken("holes", changedHole);
    if (replication.key(replicationA) == replication.key(replicationB)
      || replication.bomKey(replicationA) != replication.bomKey(replicationB)
      || replication.bomKey(replicationA) == replication.bomKey(replicationChanged))
      throw "Replication purchase identity must ignore source IDs but preserve actual hole geometry";
    var cassetteType = machinekit.structural.LinearMountNutCassette.recipeType();
    var cassetteA = new ComponentValues().setToken("stations", '[{"connector":"a","x":0,"y":0}]')
      .setToken("sourceFingerprint", haxe.crypto.Sha256.encode("source-a"));
    var cassetteB = new ComponentValues().setToken("stations", '[{"connector":"b","x":0,"y":0}]')
      .setToken("sourceFingerprint", haxe.crypto.Sha256.encode("source-b"));
    var cassetteChanged = new ComponentValues().setToken("stations", '[{"connector":"b","x":0,"y":1}]')
      .setToken("sourceFingerprint", haxe.crypto.Sha256.encode("source-b"));
    if (cassetteType.key(cassetteA) == cassetteType.key(cassetteB)
      || cassetteType.bomKey(cassetteA) != cassetteType.bomKey(cassetteB)
      || cassetteType.bomKey(cassetteA) == cassetteType.bomKey(cassetteChanged))
      throw "Cassette purchase identity must ignore source IDs but preserve station geometry";
    var motorPlate = machinekit.assembly.MotorPlate.recipeType().create().bom.partNumber;
    var carriage = machinekit.assembly.LinearAxis.Carriage.recipeType().create().bom.partNumber;
    if (motorPlate != "MOTOR-PLATE-17HS19-1684S1-T6" || carriage != "CARRIAGE-70x50-D8-TR-D8-P2-S1-RH"
      || PartNumberLint.problems(motorPlate).length != 0 || PartNumberLint.problems(carriage).length != 0)
      throw 'Axis mount identities: $motorPlate, $carriage';
    var twinRail = machinekit.assembly.ScrewAxisPart.recipeType();
    var sfuAxis = machinekit.assembly.Sfu20AxisPart.recipeType();
    var axisParts = [twinRail.create().bom.partNumber,
      twinRail.create(new ComponentValues().setToken("kind", "carriage")).bom.partNumber,
      sfuAxis.create().bom.partNumber,
      sfuAxis.create(new ComponentValues().setToken("kind", "riserLeft")).bom.partNumber];
    var axisExpected = ["TWIN-RAIL-SCREW-BASE", "TWIN-RAIL-SCREW-CARRIAGE", "SFU20-AXIS-BASE", "SFU20-AXIS-RISERLEFT"];
    for (i in 0...axisParts.length) if (axisParts[i] != axisExpected[i]
      || PartNumberLint.problems(axisParts[i]).length != 0) throw 'Axis part identity: ${axisParts[i]}';
    var routerParts = [machinekit.routing.ToolFork.recipeType().create().bom.partNumber,
      machinekit.routing.ToolRack.recipeType().create().bom.partNumber,
      machinekit.routing.DustShoe.recipeType().create().bom.partNumber,
      machinekit.routing.Spoilboard.recipeType().create().bom.partNumber,
      machinekit.routing.VacuumTable.recipeType().create().bom.partNumber];
    var routerExpected = ["ROUTER-TOOL-FORK", "ROUTER-TOOL-RACK", "ROUTER-DUST-SHOE", "ROUTER-SPOILBOARD",
      "ROUTER-VACUUM-TABLE"];
    for (i in 0...routerParts.length) if (routerParts[i] != routerExpected[i]
      || PartNumberLint.problems(routerParts[i]).length != 0) throw 'Router part identity: ${routerParts[i]}';
    var mandrel = machinekit.hobbing.ExpandingMandrel.recipeType().create().bom.partNumber;
    var rotaryClamp = machinekit.hobbing.RotaryClamp.recipeType().create().bom.partNumber;
    if (mandrel != "MANDREL-ASSUMED-D59-E0.05" || rotaryClamp != "CLAMP-ASSUMED-100NM-120x150"
      || PartNumberLint.problems(mandrel).length != 0 || PartNumberLint.problems(rotaryClamp).length != 0)
      throw 'Mandrel or rotary clamp identity: $mandrel, $rotaryClamp';
    var saddle = machinekit.hobbing.HobberSaddle.recipeType();
    var saddleZ = saddle.create().bom.partNumber;
    var saddleX = saddle.create(new ComponentValues().setToken("design", "GE-X-carriage")).bom.partNumber;
    var spindlePart = machinekit.hobbing.HobSpindlePart.recipeType().create().bom.partNumber;
    var floatingBearing = machinekit.hobbing.FloatingRadialBearing.recipeType().create().bom.partNumber;
    if (saddleZ != "HOBBER-GE-Z-SADDLE" || saddleX != "HOBBER-GE-X-CARRIAGE"
      || spindlePart != "HOB-SPINDLE-SHAFT" || floatingBearing != "FLOAT-BRG-ASSUMED-15x32x9"
      || PartNumberLint.problems(saddleZ).length != 0 || PartNumberLint.problems(saddleX).length != 0
      || PartNumberLint.problems(spindlePart).length != 0 || PartNumberLint.problems(floatingBearing).length != 0)
      throw "Hobber design and bearing identities must reflect their specified roles";
    var hobberParts = [machinekit.hobbing.MandrelSleeve.recipeType().create().bom.partNumber,
      machinekit.hobbing.TableHousing.recipeType().create().bom.partNumber,
      machinekit.hobbing.SpindleHousing.recipeType().create().bom.partNumber,
      machinekit.hobbing.TablePlaten.recipeType().create().bom.partNumber,
      machinekit.hobbing.FlexsplineCupBlank.recipeType().create().bom.partNumber,
      machinekit.hobbing.HobberBracket.recipeType().create().bom.partNumber,
      machinekit.hobbing.OutboardSupport.recipeType().create().bom.partNumber,
      machinekit.hobbing.SwivelHousing.recipeType().create().bom.partNumber,
      machinekit.hobbing.HobArbor.recipeType().create().bom.partNumber];
    var hobberExpected = ["MANDREL-SLEEVE", "HOBBER-TABLE-HOUSING", "HOB-SPINDLE-HOUSING",
      "HOBBER-TABLE-PLATEN", "FLEXSPLINE-CUP-BLANK", "HOBBER-BRACKET", "HOB-OUTBOARD-SUPPORT",
      "SWIVEL-HOUSING", "HOB-ARBOR-TF20"];
    for (i in 0...hobberParts.length) if (hobberParts[i] != hobberExpected[i]
      || PartNumberLint.problems(hobberParts[i]).length != 0) throw 'Hobber part identity: ${hobberParts[i]}';
    var toolService = machinekit.robotics.ToolServiceModule.recipeType().create().bom.partNumber;
    if (toolService != "TOOL-SVC-ASSUMED-24V-5A-2A-8P-1O" || PartNumberLint.problems(toolService).length != 0)
      throw 'Tool service identity: $toolService';
    var hob = machinekit.hobbing.Hob.recipeType().create().bom.partNumber;
    if (hob != "HOB-ASSUMED-M0.3-D32-B16-1S-12G" || PartNumberLint.problems(hob).length != 0)
      throw 'Hob identity: $hob';
    var driverA = new machinekit.motion.MotorDriver("GENERIC-TMC2209", new machinekit.power.PhaseRmsCurrent(1), 1);
    var driverB = new machinekit.motion.MotorDriver("GENERIC-TMC2209", new machinekit.power.PhaseRmsCurrent(1.5), 16);
    if (driverA.bom.partNumber != "DRIVER-ASSUMED-TMC2209" || driverA.bom.partNumber != driverB.bom.partNumber
      || driverA.bom.valuesKey != driverB.bom.valuesKey)
      throw "Driver settings must not change the purchase identity";
    var cabinetA = new machinekit.control.ControlCabinet(new ComponentValues().setInteger("inputs", 8));
    var cabinetB = new machinekit.control.ControlCabinet(new ComponentValues().setInteger("inputs", 16));
    if (cabinetA.bom.partNumber != "CABINET-ASSUMED-800x300x1000-4R"
      || cabinetA.bom.partNumber != cabinetB.bom.partNumber || cabinetA.bom.valuesKey != cabinetB.bom.valuesKey)
      throw "Derived I/O must not change the cabinet purchase identity";
    var purchases = new machinekit.component.Bom();
    purchases.addComponent(driverA); purchases.addComponent(driverB);
    purchases.addComponent(cabinetA); purchases.addComponent(cabinetB);
    if (purchases.lines().length != 2 || purchases.lines()[0].quantity != 2 || purchases.lines()[1].quantity != 2)
      throw "Equivalent purchases must roll up despite different settings";
  }
}
