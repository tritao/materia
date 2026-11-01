import machinekit.component.ComponentRecipeSupport;
import machinekit.component.ComponentType;
import machinekit.component.ComponentValues;
import machinekit.component.MachineComponent;
import machinekit.component.Procurement;
import machinekit.component.Procurement.ProcurementCodec;
import machinekit.component.ProcurementIntent;
import machinekit.component.AssumedCatalogComponent;

private class ProcurementTestPart extends MachineComponent {
  final width:Float;
  public function new(width:Float) {
    super("PROCUREMENT-TEST", "Procurement test part", "steel");
    this.width = width;
  }
  public function intent(value:Procurement, ?terms:Array<machinekit.component.Procurement.ProcurementRequirement>):Void
    addFacet(new ProcurementIntent(value, terms));
  public function assumed():Void addFacet(new machinekit.component.EngineeringMetadata(new ComponentValues()
    .setToken("ratingsBasis", "Assumed").setToken("errorsBasis", "Stated").setToken("source", "")));
  public function stated():Void addFacet(new machinekit.component.EngineeringMetadata(new ComponentValues()
    .setToken("ratingsBasis", "Stated").setToken("errorsBasis", "Assumed").setToken("source", "")));
  override public function values():ComponentValues return new ComponentValues().setNumber("width", width);
}

class ProcurementTests {
  public static function main():Void run();
  public static function run():Void {
    var type = new ComponentType("test.procurement", [ComponentRecipeSupport.length("width", 10)],
      v -> new ProcurementTestPart(v.number("width")))
      .numbered(v -> "TEST-" + machinekit.component.Dimension.format(v.number("width")))
      .procured(_ -> Buy(null, null, "ISO 123"))
      .requiring(v -> [ProcurementCodec.requirement("width", "mm", null, null, v.number("width"))]);
    var values = new ComponentValues().setNumber("width", 12.34567);
    var data = ProcurementCodec.document(type.procurement(values));
    var requirements = type.requirements(values);
    if (data.kind != "Buy" || data.standard != "ISO 123" || requirements.length != 1
      || requirements[0].name != "width" || requirements[0].unit != "mm"
      || requirements[0].nominal != machinekit.component.Dimension.format(12.34567))
      throw "Recipe procurement and structured requirements must use resolved values";
    var selectedInputs = new ComponentType("test.requirement-inputs", [
      ComponentRecipeSupport.length("width", 10), ComponentRecipeSupport.quantity("force", 50, "N"),
      ComponentRecipeSupport.quantity("delay", 0.05, "s"),
      ComponentRecipeSupport.choice("hand", ["left", "right"], "right")
    ], v -> new ProcurementTestPart(v.number("width")))
      .requiringInputs(["width", "hand"], ["force"], ["delay"]);
    var selectedTerms = selectedInputs.requirements(new ComponentValues());
    if (selectedTerms.length != 4 || selectedTerms[0].nominal != "10" || selectedTerms[1].nominal != "right"
      || selectedTerms[2].min != "50" || selectedTerms[3].max != "0.05")
      throw "Declared input requirements must retain values, units and bounds";
    var part = new ProcurementTestPart(10);
    part.intent(Make("machining"));
    var facet = ProcurementIntent.of(part);
    if (facet == null || ProcurementCodec.document(facet.value).kind != "Make")
      throw "A non-recipe part must retain its declared procurement intent";
    var assumed = new ProcurementTestPart(10);
    assumed.intent(Buy(null, null, null), [ProcurementCodec.requirement("width", "mm", null, null, 10)]);
    assumed.assumed();
    if (assumed.bom.maturity != "Assumed" || ProcurementCodec.document(assumed.bom.procurement).kind != "Buy")
      throw "Maturity must remain separate from procurement intent";
    if (machinekit.control.ControlCabinet.recipeType().create().bom.maturity != "Assumed")
      throw "Assumed catalogue components must retain assumed maturity";
    var assembly = new machinekit.assembly.MachineAssembly();
    assembly.addComponent("part", assumed);
    var excluded = new ProcurementTestPart(12);
    excluded.intent(NotSupplied);
    excluded.stated();
    assembly.addComponent("excluded", excluded);
    var document = assembly.bomDocument("test", "abc", "2026-01-01T00:00:00Z");
    var exported = document.procurement[0].item.procurement;
    if (exported == null || exported.kind != "Buy" || document.procurement[0].item.maturity != "Assumed"
      || document.procurement[0].item.requirements.length != 1 || document.procurement[0].item.material != null
      || document.procurement.length != 1 || document.assembly.lines.length != 2)
      throw "BOM export must retain procurement, maturity and structured requirements";
    var saved = new machinekit.assembly.MachineAssembly();
    saved.addComponent("cabinet", machinekit.control.ControlCabinet.recipeType().create());
    saved.addBomItem(assumed.bom);
    var restored = machinekit.assembly.MachineAssembly.decode(saved.encode());
    var restoredLines = [for (item in restored.billOfMaterials().lines()) if (item.partNumber == assumed.bom.partNumber) item];
    if (restoredLines.length != 1 || restoredLines[0].procurement == null
      || ProcurementCodec.document(restoredLines[0].procurement).kind != "Buy"
      || restoredLines[0].maturity != "Assumed" || restoredLines[0].requirements == null
      || restoredLines[0].requirements.length != 1 || restoredLines[0].requirements[0].nominal != "10")
      throw "Assembly serialization must preserve explicit procurement requirements";
    var assumedRecipes = 0;
    for (recipe in machinekit.component.MachineKitComponents.standard().all()) {
      var candidate:MachineComponent;
      try candidate = recipe.create() catch (_:Dynamic) continue;
      if (!Std.isOfType(candidate, AssumedCatalogComponent)) continue;
      assumedRecipes++;
      var item = candidate.bom;
      var requirementTerms = item.requirements;
      if (item.procurement == null || ProcurementCodec.document(item.procurement).kind != "Buy"
        || item.maturity != "Assumed" || requirementTerms == null || requirementTerms.length == 0)
        throw 'Assumed recipe ${recipe.id} must declare Buy and structured requirements';
    }
    if (assumedRecipes < 20) throw "The assumed-recipe test did not cover the registered catalog";
    var casting = machinekit.milling.MillCasting.recipeType().create().bom;
    if (casting.procurement == null || ProcurementCodec.document(casting.procurement).kind != "Make"
      || casting.maturity != "Stated") throw "The mill casting must declare its process and stated maturity";
    var driver = machinekit.motion.MotorDriver.recipeType().create().bom;
    var driverTerms = driver.requirements;
    if (driver.procurement == null || ProcurementCodec.document(driver.procurement).kind != "Buy"
      || driver.maturity != "Assumed" || driverTerms == null
      || [for (term in driverTerms) if (term.name == "maximumCurrent" && term.unit == "A rms" && term.min != null) term].length != 1)
      throw "An assumed driver must declare its selection ratings";
    var rectifier = machinekit.power.ThreePhaseRectifier.recipeType().create().bom;
    var rectifierTerms = rectifier.requirements;
    if (rectifier.procurement == null || ProcurementCodec.document(rectifier.procurement).kind != "Buy"
      || rectifier.maturity != "Stated" || rectifierTerms == null
      || [for (term in rectifierTerms) if (term.name == "lineVoltage" && term.unit == "V" && term.nominal != null) term].length != 1)
      throw "A rectifier must keep stated ratings and electrical units";
    var servo = machinekit.motion.ServoMotor.model("GENERIC-SERVO-200W");
    var servoKind = ProcurementCodec.document(servo.bom.procurement);
    var servoTerms = servo.bom.requirements;
    if (servoKind.kind != "Buy" || servo.bom.maturity != "Assumed" || servoTerms == null
      || [for (term in servoTerms) if (term.name == "peakTorque" && term.min != null && term.unit == "N m") term].length != 1
      || [for (term in servoTerms) if (term.name == "encoderResolution" && term.unit == "count/rev") term].length != 1)
      throw "An assumed servo must be a Buy with structured rating requirements";
    var selected = machinekit.motion.LeadshineServo.catalog().designations()[0];
    var catalogServo = machinekit.motion.ServoMotor.model(selected);
    var catalogKind = ProcurementCodec.document(catalogServo.bom.procurement);
    if (catalogKind.kind != "Buy" || catalogKind.manufacturer != "Leadshine" || catalogKind.mpn != selected
      || catalogServo.bom.maturity != "Catalog") throw "A sourced catalogue servo must retain its selection and maturity";
    for (candidate in [machinekit.motion.ShaftBrakePart.recipeType().create(),
      machinekit.motion.ShaftEncoder.recipeType().create(), machinekit.milling.SpindleMotor.recipeType().create(),
      machinekit.milling.SpindleCartridge.recipeType().create(),
      machinekit.milling.SpindleCartridge.Er20Holder.recipeType().create()]) {
      var item = candidate.bom;
      if (item.procurement == null || item.maturity != "Assumed" || item.requirements == null
        || item.requirements.length == 0) throw 'Missing assumed selection requirements for ${item.partNumber}';
    }
    var holder = machinekit.milling.SpindleCartridge.Er20Holder.recipeType().create().bom;
    if (ProcurementCodec.document(holder.procurement).kind != "NotSupplied")
      throw "The ER20 holder is user tooling";
    for (candidate in [machinekit.motion.BallScrew.ballRecipeType().create(),
      machinekit.motion.BallNut.ballRecipeType().create(), machinekit.motion.LeadScrew.recipeType().create(),
      machinekit.motion.LeadScrewNut.recipeType().create()]) {
      var item = candidate.bom;
      if (item.procurement == null || ProcurementCodec.document(item.procurement).kind != "Buy"
        || item.maturity != "Assumed" || item.requirements == null || item.requirements.length == 0)
        throw 'Screw drive components require their assumed selection inputs: ${item.partNumber}';
    }
    var rail = machinekit.motion.LinearRail.recipeType().create().bom;
    if (rail.procurement == null || ProcurementCodec.document(rail.procurement).kind != "Buy"
      || rail.maturity != "Catalog") throw "A catalog rail must retain its rating source";
    var fastener = machinekit.standard.SocketHeadCapScrew.recipeType().create().bom;
    if (fastener.procurement == null || ProcurementCodec.document(fastener.procurement).standard != "ISO 4762"
      || fastener.maturity != "Catalog")
      throw "A standard fastener must carry its declared standard";
    var flangeBolt = machinekit.standard.HexFlangeBolt.metric("M6", 10).bom;
    if (flangeBolt.procurement == null || ProcurementCodec.document(flangeBolt.procurement).standard != "ISO 4162"
      || flangeBolt.maturity != "Catalog") throw "A flange bolt must retain its standard and catalogue maturity";
    for (recipe in [machinekit.standard.FlatWasher.recipeType(), machinekit.standard.HexNut.recipeType(),
      machinekit.standard.ParallelKey.recipeType(), machinekit.standard.RetainingRing.recipeType(),
      machinekit.standard.SocketSetScrew.recipeType()]) {
      var item = recipe.create().bom;
      if (item.procurement == null || item.maturity != "Catalog")
        throw 'A standard part must retain its catalogue maturity: ${recipe.id}';
    }
    var gearbox = new machinekit.motion.Gearbox(50, 0.9, 60, 40, 8, true).bom;
    if (gearbox.procurement == null || ProcurementCodec.document(gearbox.procurement).kind != "Buy"
      || gearbox.maturity != "Assumed" || gearbox.requirements == null || gearbox.requirements.length < 5)
      throw "An assumed gearbox must carry selection requirements";
    var stepper = machinekit.motion.NemaStepper.model("17HS19-1684S1").bom;
    if (stepper.procurement == null || ProcurementCodec.document(stepper.procurement).mpn != "17HS19-1684S1"
      || stepper.maturity != "Catalog") throw "A named stepper must retain its catalogue evidence";
    var assumedStepper = machinekit.motion.NemaStepper.model("17HS08-1004S").bom;
    if (assumedStepper.maturity != "Assumed" || assumedStepper.requirements == null
      || [for (term in assumedStepper.requirements) if (term.name == "holdingTorque" && term.min == "0.13") term].length != 1)
      throw "The pancake stepper's assumed ratings must remain requirements";
    var openBuildsStepper = machinekit.motion.NemaStepper.model("OPENBUILDS-NEMA23-56").bom;
    if (openBuildsStepper.procurement == null || ProcurementCodec.document(openBuildsStepper.procurement).manufacturer != null
      || openBuildsStepper.maturity != "Assumed") throw "A geometry reference is not a manufacturer part selection";
    var genericStepper = machinekit.motion.NemaStepper.frame(17, 50).bom;
    if (genericStepper.procurement == null || genericStepper.maturity != "Assumed"
      || genericStepper.requirements == null || genericStepper.requirements.length != 2)
      throw "A generic stepper must retain its selection requirements";
    for (candidate in [machinekit.motion.LimitSwitch.recipeType().create(),
      machinekit.motion.ProximitySwitch.recipeType().create(), machinekit.control.DigitalSensor.recipeType().create()]) {
      var item = candidate.bom;
      if (item.procurement == null || ProcurementCodec.document(item.procurement).kind != "Buy"
        || item.maturity != "Assumed" || item.requirements == null || item.requirements.length == 0)
        throw 'An assumed sensor must declare its selection requirements: ${item.partNumber}';
    }
    var robotFlange = machinekit.robotics.RobotFlange.recipeType().create().bom;
    if (robotFlange.procurement == null || ProcurementCodec.document(robotFlange.procurement).kind != "Make"
      || robotFlange.maturity != "Assumed" || robotFlange.requirements == null || robotFlange.requirements.length < 5)
      throw "A modeled robot flange must retain its assumed interface dimensions";
    for (candidate in [machinekit.robotics.Pedestal.recipeType().create(),
      machinekit.robotics.EndEffectorPlate.recipeType().create(), machinekit.robotics.ArmWristFork.recipeType().create()]) {
      var item = candidate.bom;
      if (item.procurement == null || ProcurementCodec.document(item.procurement).kind != "Make"
        || item.maturity != "Assumed" || item.requirements == null || item.requirements.length == 0)
        throw 'A modeled robot structural part must retain its dimensions: ${item.partNumber}';
    }
    var cobotJoint = machinekit.robotics.CobotJoint.moduleRecipeType().create().bom;
    if (cobotJoint.procurement == null || ProcurementCodec.document(cobotJoint.procurement).kind != "Buy"
      || cobotJoint.maturity != "Assumed" || cobotJoint.requirements == null
      || [for (term in cobotJoint.requirements) if (term.name == "ratedOutputTorque" && term.min == "56") term].length != 1)
      throw "A cobot joint must carry its assumed output rating";
    var wristRotor = new machinekit.robotics.CobotWristRotor("TEST-WRIST-ROTOR", new machinekit.robotics.CobotJoint(2)).bom;
    if (wristRotor.procurement == null || ProcurementCodec.document(wristRotor.procurement).kind != "Make"
      || wristRotor.maturity != "Assumed" || wristRotor.requirements == null || wristRotor.requirements.length != 2)
      throw "An assumed wrist rotor must declare its manufacturing requirements";
    var battery = new machinekit.power.BatteryPack(24, 240, 200, 120, 80, 3).bom;
    if (battery.procurement == null || battery.maturity != "Assumed" || battery.requirements == null
      || [for (term in battery.requirements) if (term.name == "continuousCurrent" && term.min == "10") term].length != 1)
      throw "A battery must carry its assumed continuous current";
    var converter = new machinekit.power.IsolatedDcConverter(48, 24, 10, 2).bom;
    var charger = new machinekit.power.BatteryCharger(48, 3000, 300).bom;
    var inverter = new machinekit.power.Inverter(48, 230, 3000, 300, 200, 100, 5).bom;
    for (item in [converter, charger, inverter]) if (item.procurement == null
      || ProcurementCodec.document(item.procurement).kind != "Buy" || item.requirements == null || item.requirements.length == 0)
      throw 'Power equipment needs explicit selection requirements: ${item.partNumber}';
    for (candidate in [machinekit.motion.DriveWheel.recipeType().create(), machinekit.motion.CasterWheel.recipeType().create()]) {
      var item = candidate.bom;
      if (item.procurement == null || ProcurementCodec.document(item.procurement).kind != "Buy"
        || item.maturity != "Assumed" || item.requirements == null || item.requirements.length < 5)
        throw 'A wheel must carry its dimensional selection requirements: ${item.partNumber}';
    }
    if (ProcurementCodec.document(new machinekit.printing.PrintedPart().bom.procurement).kind != "NotSupplied")
      throw "The printed workpiece must not be procured with the printer";
    for (candidate in [machinekit.printing.Hotend.recipeType().create(), machinekit.printing.Nozzle.recipeType().create(),
      machinekit.printing.HeaterCartridge.recipeType().create(), machinekit.printing.Thermistor.recipeType().create(),
      machinekit.printing.Fan.recipeType().create(), machinekit.printing.Extruder.recipeType().create(),
      machinekit.printing.HeatedBed.recipeType().create(), machinekit.control.ThermistorInput.recipeType().create()]) {
      var item = candidate.bom;
      if (item.procurement == null || ProcurementCodec.document(item.procurement).kind != "Buy"
        || item.maturity != "Assumed" || item.requirements == null || item.requirements.length == 0)
        throw 'A reference printer part must declare selection requirements: ${item.partNumber}';
    }
    var spurGear = machinekit.transmission.SpurGear.recipeType().create().bom;
    if (spurGear.procurement == null || ProcurementCodec.document(spurGear.procurement).kind != "Make")
      throw "A generated spur gear must declare its manufacturing process";
    for (candidate in [machinekit.structural.LinearMountSteel.recipeType().create(),
      machinekit.structural.MountDrilledPlate.recipeType().create(), machinekit.assembly.LinearAxis.Carriage.recipeType().create(),
      machinekit.motion.FlangeBearingHousing.recipeType().create()]) {
      var item = candidate.bom;
      if (item.procurement == null || ProcurementCodec.document(item.procurement).kind != "Make")
        throw 'A machined mount must declare its process: ${item.partNumber}';
    }
    for (candidate in [machinekit.motion.LinearScale.recipeType().create(), machinekit.motion.SfuDrawingScrew.drawingRecipeType().create()]) {
      var item = candidate.bom;
      if (item.procurement == null || ProcurementCodec.document(item.procurement).kind != "Buy"
        || item.maturity != "Assumed" || item.requirements == null || item.requirements.length == 0)
        throw 'A reference motion component must declare its requirements: ${item.partNumber}';
    }
    var scaleTerms = machinekit.motion.LinearScale.recipeType().create().bom.requirements;
    if (scaleTerms == null || [for (term in scaleTerms) if (term.name == "resolution" && term.unit == "count/mm") term].length != 1)
      throw "Linear scale resolution must be expressed per millimetre";
    for (candidate in [machinekit.motion.TorqueMotor.recipeType().create(), machinekit.motion.AngularContactBearing.recipeType().create(),
      machinekit.motion.CrossedRollerBearing.recipeType().create(), machinekit.motion.RotaryEncoderRing.recipeType().create()]) {
      var item = candidate.bom;
      if (item.procurement == null || item.maturity != "Assumed" || item.requirements == null || item.requirements.length == 0)
        throw 'Precision reference part needs selection requirements: ${item.partNumber}';
    }
    var ringTerms = machinekit.motion.RotaryEncoderRing.recipeType().create().bom.requirements;
    if (ringTerms == null || [for (term in ringTerms) if (term.name == "angularAccuracy" && term.unit == "arcsec" && term.max == "2") term].length != 1)
      throw "Ring encoder accuracy must retain arcsecond precision";
    var rotary = machinekit.motion.PurchasedRotaryPart.recipeType().create().bom;
    if (rotary.procurement == null || ProcurementCodec.document(rotary.procurement).kind != "Buy" || rotary.maturity != "Catalog")
      throw "The purchased rotary table must retain its rated source basis";
    var floatingBearing = machinekit.hobbing.FloatingRadialBearing.recipeType().create().bom;
    if (floatingBearing.procurement == null || ProcurementCodec.document(floatingBearing.procurement).kind != "Buy"
      || floatingBearing.maturity != "Assumed" || floatingBearing.requirements == null || floatingBearing.requirements.length == 0)
      throw "A floating bearing must declare its assumed selection data";
    var hobShaft = machinekit.hobbing.HobSpindlePart.recipeType().create().bom;
    if (hobShaft.procurement == null || ProcurementCodec.document(hobShaft.procurement).kind != "Make"
      || hobShaft.maturity != "Assumed" || hobShaft.requirements == null || hobShaft.requirements.length != 3)
      throw "A hob spindle part must retain its assumed manufacturing dimensions";
    for (candidate in [machinekit.hobbing.MandrelSleeve.recipeType().create(), machinekit.hobbing.FlexsplineCupBlank.recipeType().create(),
      machinekit.hobbing.Hob.recipeType().create()]) {
      var item = candidate.bom;
      if (item.procurement == null || ProcurementCodec.document(item.procurement).kind != "NotSupplied"
        || item.maturity != "Assumed" || item.requirements == null || item.requirements.length == 0)
        throw 'Hobber tooling and workpieces must remain in the engineering BOM: ${item.partNumber}';
    }
    var tube = machinekit.structural.RectTubeMember.recipeType().create().bom;
    if (tube.procurement == null || ProcurementCodec.document(tube.procurement).kind != "Buy"
      || tube.maturity != "Stated") throw "An unmodified tube member is purchased stock";
    var profile = machinekit.structural.ProfileMember.tslotRecipeType().create().bom;
    if (profile.procurement == null || ProcurementCodec.document(profile.procurement).kind != "Buy")
      throw "A structural profile member must declare purchased stock";
    var customProfile = new machinekit.structural.ProfileMember(new machinekit.structural.RectTube(30, 30, 2), 100).bom;
    if (customProfile.procurement == null || ProcurementCodec.document(customProfile.procurement).kind != "Buy")
      throw "A non-recipe profile member must declare purchased stock";
    for (variant in ["cnc-aluminium", "sls-pa12"]) {
      var finger = machinekit.robotics.OffsetGripperFinger.family().create(variant).bom;
      if (finger.procurement == null || ProcurementCodec.document(finger.procurement).process != finger.manufacturingProcess)
        throw 'Manufacturing intent must use the selected variant process: $variant';
    }
    var insert = machinekit.structural.CastInsert.recipeType().create(new ComponentValues().setToken("role", "generic-mount")).bom;
    if (insert.procurement == null || ProcurementCodec.document(insert.procurement).kind != "Make")
      throw "A fabricated casting insert must declare its manufacturing process";
    var socket = machinekit.structural.FixingSockets.pfeiferStudy(
      {outsideDiameter:8, reliefDiameter:6.2, tailWidth:11, tailThickness:2.2, tailWave:1.4, tailBlend:0.5}, 0.5).bom;
    if (socket.procurement == null || ProcurementCodec.document(socket.procurement).kind != "Buy"
      || ProcurementCodec.document(socket.procurement).manufacturer != "PFEIFER"
      || ProcurementCodec.document(socket.procurement).mpn != machinekit.structural.FixingSockets.PFEIFER_REFERENCE
      || socket.maturity != "Assumed" || socket.requirements == null || socket.requirements.length == 0)
      throw "A catalogue insert must retain its manufacturer and reference";
    for (candidate in [machinekit.pneumatic.PneumaticCylinder.recipeType().create(),
      machinekit.pneumatic.PneumaticCylinderRod.recipeType().create(), machinekit.pneumatic.AirSupply.recipeType().create(),
      machinekit.pneumatic.SolenoidValve.recipeType().create(), machinekit.pneumatic.PneumaticManifold.recipeType().create()]) {
      var item = candidate.bom;
      if (item.procurement == null || ProcurementCodec.document(item.procurement).kind != "Buy"
        || item.maturity != "Assumed" || item.requirements == null || item.requirements.length == 0)
        throw 'A reference pneumatic component must declare selection requirements: ${item.partNumber}';
    }
    var hoseRoute = [new cadkit.modeling.Vector(0, 0, 0), new cadkit.modeling.Vector(0, 0, 100)];
    var stockHose = machinekit.pneumatic.RoutedHose.stock("10.07.09.00001", hoseRoute).bom;
    if (stockHose.procurement == null || ProcurementCodec.document(stockHose.procurement).manufacturer != "Schmalz"
      || stockHose.maturity != "Catalog") throw "A stock hose must retain its catalogue identity";
    var customHose = new machinekit.pneumatic.RoutedHose("TEST-HOSE", hoseRoute, 4, 2, 0.011).bom;
    if (customHose.procurement == null || customHose.maturity != "Assumed" || customHose.requirements == null
      || customHose.requirements.length != 4) throw "A custom hose must retain its tube requirements";
    var fitting = machinekit.pneumatic.PushInFitting.recipeType().create().bom;
    var cup = machinekit.pneumatic.SuctionCup.model("10.01.01.11401").bom;
    var generator = machinekit.pneumatic.VacuumGenerator.model("10.02.01.00563").bom;
    for (item in [fitting, cup, generator]) if (item.procurement == null || item.maturity != "Catalog"
      || ProcurementCodec.document(item.procurement).manufacturer != "Schmalz")
      throw "Catalog vacuum components must retain manufacturer evidence";
    for (candidate in [machinekit.pneumatic.SuctionCup.recipeType().create(),
      machinekit.pneumatic.VacuumGenerator.recipeType().create(), machinekit.pneumatic.VacuumPressureSensor.recipeType().create()]) {
      var item = candidate.bom;
      if (item.procurement == null || item.maturity != "Assumed" || item.requirements == null || item.requirements.length == 0)
        throw 'A custom vacuum component needs structured port requirements: ${item.partNumber}';
    }
    var weldingParts:Array<MachineComponent> = [new machinekit.welding.WeldingPowerSource(), new machinekit.welding.WireFeeder(),
      new machinekit.welding.GasCylinder(), new machinekit.welding.WeldingTorch(),
      new machinekit.welding.WeldingTorchNeck(), new machinekit.welding.WeldingTorchNozzle(),
      new machinekit.welding.WorkClamp()];
    for (candidate in weldingParts) {
      var item = candidate.bom;
      if (item.procurement == null || item.maturity != "Assumed" || item.requirements == null || item.requirements.length == 0)
        throw 'A welding component must declare selection requirements: ${item.partNumber}';
    }
    if (ProcurementCodec.document(new machinekit.welding.WorkClamp().bom.procurement).kind != "NotSupplied")
      throw "The work clamp must be excluded from procurement";
    for (candidate in [machinekit.pressbrake.HydraulicCylinder.recipeType().create(),
      machinekit.pressbrake.HydraulicCylinderRod.recipeType().create(), machinekit.pressbrake.HydraulicSupply.recipeType().create(),
      machinekit.pressbrake.HydraulicValve.recipeType().create(), machinekit.pressbrake.HydraulicManifold.recipeType().create(),
      machinekit.pressbrake.GaugeFinger.recipeType().create()]) {
      var item = candidate.bom;
      if (item.procurement == null || item.maturity != "Assumed" || item.requirements == null || item.requirements.length == 0)
        throw 'A reference press-brake part must declare its requirements: ${item.partNumber}';
    }
    for (candidate in [machinekit.pressbrake.ToolSegment.recipeType().create(), machinekit.pressbrake.ToolClamp.recipeType().create()])
      if (candidate.bom.procurement == null || ProcurementCodec.document(candidate.bom.procurement).kind != "NotSupplied")
        throw "Press-brake tools and clamps must remain in the engineering BOM only";
    var driverBody = new machinekit.screwdriving.ScrewdriverHousing("driver", 120, 25, true, 20, 62.8).bom;
    if (driverBody.procurement == null || driverBody.maturity != "Assumed" || driverBody.requirements == null
      || [for (term in driverBody.requirements) if (term.name == "maxTorque" && term.min == "20") term].length != 1)
      throw "The screwdriver body must carry its required torque";
    var missingRequirement = new ProcurementTestPart(11);
    missingRequirement.intent(Buy(null, null, null));
    missingRequirement.assumed();
    var incomplete = new machinekit.assembly.MachineAssembly();
    incomplete.addComponent("incomplete", missingRequirement);
    var missingRejected = false;
    try incomplete.bomDocument("incomplete", "abc", "2026-01-01T00:00:00Z") catch (_:Dynamic) missingRejected = true;
    if (!missingRejected) throw "An assumed purchase without requirements must fail export";
    var undeclared = new machinekit.assembly.MachineAssembly();
    undeclared.addBomItem({partNumber:"UNDECLARED", description:"Undeclared", quantity:1, material:null});
    var intentRejected = false;
    try undeclared.bomDocument("undeclared", "abc", "2026-01-01T00:00:00Z") catch (_:Dynamic) intentRejected = true;
    if (!intentRejected) throw "Missing procurement intent must fail export";
    var noMaturity = new machinekit.assembly.MachineAssembly();
    noMaturity.addBomItem({partNumber:"NO-MATURITY", description:"No maturity", quantity:1, material:null,
      procurement:Buy(null, null, null)});
    var maturityRejected = false;
    try noMaturity.bomDocument("no-maturity", "abc", "2026-01-01T00:00:00Z") catch (_:Dynamic) maturityRejected = true;
    if (!maturityRejected) throw "Missing engineering maturity must fail export";
    if (ProcurementCodec.document(NotSupplied).kind != "NotSupplied") throw "Not-supplied intent";
    var rejected = false;
    try ProcurementCodec.document(Make("")) catch (_:Dynamic) rejected = true;
    if (!rejected) throw "A made part requires a process";
    var precisionRejected = false;
    try ProcurementCodec.requirement("inertia", "kg m2", null, null, 0.00001) catch (_:Dynamic) precisionRejected = true;
    if (!precisionRejected) throw "Requirements must not silently round a nonzero value to zero";

  }
}
