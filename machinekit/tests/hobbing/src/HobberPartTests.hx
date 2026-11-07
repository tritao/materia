import machinekit.structural.FoundationIntegrity;
import machinekit.structural.MineralFoundation;
import machinekit.motion.*;
import machinekit.hobbing.*;
import machinekit.component.*;
import machinekit.structural.MineralCasting;
import machinekit.structural.CastInsert;
import machinekit.assembly.MachineAssembly;
import materia.assembly.AssemblyDefinition.AssemblyJointType;

class HobberPartTests {
  static var assertions = 0;
  static function check(ok:Bool, label:String):Void {
    assertions++;
    if (!ok) throw label;
  }
  static function near(a:Float, b:Float, tolerance:Float, label:String):Void check(
    Math.abs(a - b) <= tolerance,
    '$label: $a vs $b'
  );
  static function rejects(fn:Void -> Void, label:String):Void {
    var failed = false;
    try fn() catch (_:Dynamic) failed = true;
    check(failed, label);
  }
  static function members():Array<MachineComponent> return [
    new TorqueMotor(),
    new AngularContactBearing(),
    new CrossedRollerBearing(),
    new RotaryEncoderRing(),
    new HobSpindlePart(),
    new HobSpindlePart(new ComponentValues().setToken("role", "pullStud")),
    new FlexsplineCupBlank(),
    new MandrelSleeve(),
    new Hob(),
    new HobArbor(),
    new OutboardSupport(),
    new ExpandingMandrel(),
    new RotaryClamp(),
    new TablePlaten(),
    new TableHousing(),
    new SpindleHousing(),
    new SwivelHousing(),
    new MineralCasting(),
    new CastInsert(new ComponentValues().setToken("role", "generic-mount"))
  ];
  public static function run():Void {
    var registry = MachineKitComponents.standard(), parts = members();
    rejects(() -> ManufacturingRoute.foldedSheet(0, 1, 2), "formed sheet rejects zero thickness");
    rejects(() -> ManufacturingRoute.foldedSheet(2, 0, 2), "formed sheet needs bends");
    for (role in HobberChipCover.MADE) {
      var guard = new HobberChipCover(new ComponentValues().setToken("role", role)), shape = guard.geometry();
      check(shape.solidCount() == 1 && shape.volume() > 0, "one connected sheet " + role);
      var sheetRoute:ManufacturingRoute = ManufacturingRoute.of(guard);
      check(sheetRoute != null && sheetRoute.setups == 0, "sheet production does not invent machining setups " + role);
      if (role != "Xrear") {
        var form = sheetRoute.sheet;
        if (form == null) throw "missing formed-sheet specification";
        check(Std.string(sheetRoute.kind) == "FoldedSheet", "formed sheet route " + role);
        near(form.thickness, 2, 0, "shield sheet thickness");
        near(form.insideRadius, 2, 0, "shield bend radius");
      } else check(Std.string(sheetRoute.kind) == "CutPlate", "rear lid remains flat cut plate");
      check(!HobberChipCover.recipeType().materialEditable, "guard steel fixed for stated mass " + role);
      shape.close();
    }
    var plug = new machinekit.standard.PanelHolePlug();
    var plugMetadata = EngineeringMetadata.of(plug);
    check(plugMetadata != null && plugMetadata.ratingsBasis == Assumed, "panel plug procurement envelope is assumed");
    check(plug.fastensByForm() && plug.mountingFace() != null, "panel plug has a retained port interface");
    check(HobberChipCover.plugs().length == 4, "all four foot tool ports close with plugs");
    check(HobberChipCover.coverage().length > 0, "end overlaps checked at overtravel");
    var supportBearing = new SpindleBearing();
    near(supportBearing.row.bore, 15, 0, "SKF 7002 journal size");
    near(supportBearing.row.outside, 32, 0, "SKF 7002 housing size");
    near(supportBearing.row.width, 9, 0, "SKF 7002 width");
    near(supportBearing.massProperties().mass, 0.03, 1e-12, "SKF catalogue mass retained independently of race envelope");
    near(supportBearing.row.pairPreload, 20, 0, "GA pair unmounted preload from SKF sheet");
    near(supportBearing.row.pairAxialStiffness, 17e6, 0, "SKF pair axial stiffness in N/m");
    var bearingMetadata = EngineeringMetadata.of(supportBearing);
    check(bearingMetadata != null && bearingMetadata.ratingsBasis == Catalog, "bearing ratings have catalogue provenance");
    var bearingRecipe = supportBearing.componentType();
    check(bearingRecipe != null && !bearingRecipe.materialEditable, "SKF steel bearing cannot become another material under same SKU");
    var supportRecipe = OutboardSupport.recipeType(), interfaceRecipe = OutboardBearingPart.recipeType();
    check(!supportRecipe.materialEditable && !interfaceRecipe.materialEditable, "precision support retains specified steel and thermal design");
    for (role in ["sleeve", "gland", "cap", "innerSpacer"]) {
      var part = new OutboardBearingPart(new ComponentValues().setToken("role", role));
      var solid = part.geometry();
      check(solid.volume() > 0 && Math.isFinite(solid.volume()), "outboard " + role + " solid valid");
      check(solid.solidCount() == 1, "outboard " + role + " one made part");
      var route = ManufacturingRoute.of(part);
      check(route != null && route.setups <= 2, "outboard " + role + " within two setups");
      solid.close();
    }

    var rendered = new Hob(new ComponentValues().setBoolean("renderTeeth", true));
    check(ComponentDisplayMesh.of(rendered) != null, "detailed display has the CAD tessellation facet");
    var detailed = new Hob(new ComponentValues().setBoolean("renderTeeth", true).setBoolean("detailed", true));
    check(ComponentDisplayMesh.of(detailed) == null,
      "detailed CAD uses its existing mesh without rebuilding the solid");
    var journal = new BallScrew(200,
      60), rebuiltJournal:BallScrew = cast BallScrew.ballRecipeType().create(journal.values());
    near(rebuiltJournal.inputJournalLength, 60, 0, "custom bearing journal survives recipe reconstruction");
    near(
      journal.axialComplianceBetween(40, 80),
      20 / (200000 * Math.PI * 12 * 12 / 4) + 20 / (200000 * Math.PI * 13 * 13 / 4),
      1e-15,
      "axial load crosses the actual machined journal/race boundary"
    );
    near(
      (cast BallScrew.ballRecipeType().create(new ComponentValues().setNumber("length",
      200)):BallScrew).inputJournalLength,
      50,
      0,
      "old screw values retain the 50 mm journal"
    );
    rejects(() -> new BallScrew(200, 24), "journal too short for bearing seat refused");
    rejects(() -> new BallScrew(200, 190), "journal leaving no race refused");
    for (p in parts) {
      var type = p.componentType();
      if (type == null) throw "missing recipe";
      check(registry.has(type.id), 'registered ${p.designation}');
      var rebuilt = type.create(p.values());
      check(type.key(p.values()) == type.key(rebuilt.values()), 'recipe roundtrip ${p.designation}');
      check(p.bom.quantity == 1 && p.bom.material != null && p.bom.typeId == type.id, 'BOM ${p.designation}');
      var errors = PrecisionErrors.of(p), copyErrors = PrecisionErrors.of(rebuilt);
      if (Std.isOfType(p, MineralCasting) || Std.isOfType(p, CastInsert)) check(
        errors == null && copyErrors == null,
        "structural parts have no motion error assumptions"
      );
      else check(
        errors != null && copyErrors != null && errors.describe() == copyErrors.describe(),
        'error bounds roundtrip ${p.designation}'
      );
    }
    near(new MineralCastBed().body.parameterNumber("depth"), 800, 0, "current bed uses moving-column footprint");
    near(
      new MineralCastBed().body.parameterNumber("width"),
      machinekit.hobbing.HobberXLayout.front().bedLength,
      0,
      "bed radial length covers loading retract"
    );
    var column = new MineralCastColumn();
    near(column.body.parameterNumber("height"), 500, 0, "moving column height");
    check(column.components().filter(entry -> entry.id == "basePlate").length == 0,
      "column sole uses separate supported corners, without the replaced rear strip");
    for (pad in FoundationLayout.columnSolePads()) {
      var matches=column.components().filter(entry -> entry.id == pad.id);
      check(matches.length == 1,"one physical column sole pad "+pad.id);
      var insert:CastInsert=cast matches[0].component;
      check(insert.parameterToken("stockMaterial") == "c45-en-1.0503-ground", "sole precision comes from bought ground stock "+pad.id);
      check(insert.parameterToken("finish") == "as-supplied", "sole does not prescribe custom grinding before measurement "+pad.id);
      var spec=insert.insertSpec();
      var service=spec.holes.filter(hole -> StringTools.startsWith(hole.id,"soleJoint"));
      check(service.length == 2,"sole joint drilling reserved before anchor selection "+pad.id);
      switch spec.anchorage {
        case Bolted(rules,_):
          var demand=rules.demand.servicePreloads.filter(load -> StringTools.startsWith(load.holeId,"soleJoint"));
          check(demand.length == 2,"both sole service preloads size the cast anchors "+pad.id);
          check(demand[0].preloadN+demand[1].preloadN>30000,"anchor demand includes structural sole preload "+pad.id);
        case _: throw "Column sole lacks declared bolted cast anchors: "+pad.id;
      }
    }
    var largeBearing = new CrossedRollerBearing(new ComponentValues().setNumber("outside", 180).setNumber("bore", 150));
    check(
      largeBearing.bom.partNumber != new CrossedRollerBearing().bom.partNumber,
      "compact swivel bearing has a distinct BOM size"
    );
    for (p in parts) {
      var metadata = EngineeringMetadata.of(p);
      check(
        metadata != null || Std.isOfType(p, MineralCasting) || Std.isOfType(p, CastInsert),
        "engineering provenance optional on structural parts and present on rated motion parts"
      );
      if (metadata != null) {
        var rebuilt = p.componentType().create(p.values());
        var copied = EngineeringMetadata.of(rebuilt);
        check(
          copied != null && copied.describe() == metadata.describe(),
          "engineering metadata persists independently of geometry"
        );
      }
    }
    var motor = new TorqueMotor(), actuator = motor.actuator("table", "C", 48, 1);
    near(motor.jointTorque(7), 7, 0, "direct drive has no reduction");
    near(actuator.maxEffort, 30, 1e-12, "peak torque limits joint effort");
    near(actuator.maxRate, 120 * 2 * Math.PI / 60, 1e-12, "motor max speed limits joint speed");
    check(actuator.ratedTorque == 10 && actuator.assumed != null,
      "continuous and assumed ratings survive motor export");
    check(motor.parameterCount("poleCount") == 24 && motor.coggingTorque == 0.03, "poles and cogging carried by motor");
    var encoder = new RotaryEncoderRing();
    near(encoder.quantum(), 2 * Math.PI / 8388608, 1e-16, "23-bit encoder quantum");
    near(encoder.angularAccuracy, 2 * Math.PI / (180 * 3600), 1e-16, "encoder accuracy radians");
    var table = new RotaryTable(), spindle = new HobSpindle(), head = new SwivelHead();
    var assemblies:Array<MachineAssembly> = [table, spindle, head];
    for (assembly in assemblies) {
      assembly.validate();
      var saved = assembly.encode(), restored = MachineAssembly.decode(saved, registry);
      check(assembly.components().length == restored.components().length, "all assembly members survive save/load");
      var originalErrors = PrecisionErrors.inAssembly(assembly), restoredErrors = PrecisionErrors.inAssembly(restored);
      check(
        originalErrors.length > 0 && originalErrors.length == restoredErrors.length,
        "error sources carried into assembly description"
      );
      for (i in 0...originalErrors.length) {
        check(originalErrors[i].part == restoredErrors[i].part, "error source remains on its physical part");
        near(originalErrors[i].error.bound, restoredErrors[i].error.bound, 0, "error bound preserves units/value");
        check(originalErrors[i].error.basis == restoredErrors[i].error.basis, "error provenance preserved");
      }
      check(
        assembly.billOfMaterials().lines().length == restored.billOfMaterials().lines().length,
        "subassembly BOM survives"
      );
    }
    var editedBearing = new CrossedRollerBearing(new ComponentValues().setNumber("radialRunout", 0.007).setToken(
      "errorsBasis",
      "Stated"
    ));
    var editedErrors = MachineAssembly.decode(new RotaryTable(null, editedBearing).encode(), registry);
    var changed = PrecisionErrors.inAssembly(editedErrors).filter(e -> e.part == "bearing"
      && e.error.quantity == "radialRunout");
    check(
      changed.length == 1 && changed[0].error.bound == 0.007 && changed[0].error.basis == Stated,
      "edited part error and provenance survive assembly description"
    );
    var c = table.definition().joints.filter(j -> j.id == "C")[0];
    check(
      c.type == Continuous && c.limits.velocity == table.motor.maxSpeed && c.limits.effort == table.motor.peakTorque,
      "C limits from installed motor"
    );
    var faster = new TorqueMotor(new ComponentValues().setNumber("peakTorque", 42).setNumber("maxSpeed", 20));
    var edited = new RotaryTable(faster);
    check(
      edited.definition().joints.filter(j -> j.id == "C")[0].limits.effort == 42,
      "edited motor changes derived C limit"
    );
    check(spindle.motor.rating.designation == HobSpindle.MOTOR && spindle.motor.ratingsBasis == Catalog
      && (cast spindle.motor : machinekit.motion.LeadshineServo).assumedWinding,
      "spindle is driven by the catalogue ELM2H-1000LA80F with an assumed winding");
    near(spindle.motor.rating.ratedTorque * spindle.motor.rating.ratedSpeed, 1000, 2, "spindle servo is 1 kW");
    check((cast spindle.motor : machinekit.motion.LeadshineServo).clearanceBoxes().length == 3
      && (cast spindle.motor : machinekit.motion.LeadshineServo).clearanceBoxes().filter(b -> b.name != "body" && b.y1 == 55).length == 2,
      "spindle servo clearance has its body and both connectors, reaching LH 55 from the axis");
    near(spindle.maxSpeed(), 4000 * 2 * Math.PI / 60, 1e-9, "B has a deliberate 4000 rpm operating cap below the bought 6000 rpm rating");
    check(
      spindle.definition().joints.filter(j -> j.id == "B")[0].limits.velocity == spindle.maxSpeed(),
      "B joint carries the cartridge speed limit"
    );
    check(spindle.encoder.countsPerRevolution == HobSpindle.ENCODER_COUNTS && spindle.encoder.absolute
      && spindle.motorEncoder.countsPerRevolution == spindle.motor.rating.encoderCounts,
      "B master is the cartridge's own 19-bit absolute encoder; the servo keeps its own for the drive loop");
    near(spindle.ratio, 1.0, 1e-9, "the belt turns the motor once per spindle turn");
    var spindleState = new cadkit.modeling.AssemblyState(spindle.definition());
    var beltBefore = spindleState.worldPose("belt"), pulleyBefore = spindleState.worldPose("spindlePulley"),
      motorBefore = spindleState.worldPose("motorPulley");
    spindleState.setJoint("B", Math.PI / 3);
    var beltAfter = spindleState.worldPose("belt"), pulleyAfter = spindleState.worldPose("spindlePulley"),
      motorAfter = spindleState.worldPose("motorPulley");
    var beltDot = beltBefore.qx * beltAfter.qx + beltBefore.qy * beltAfter.qy
      + beltBefore.qz * beltAfter.qz + beltBefore.qw * beltAfter.qw;
    check(Math.abs(beltBefore.x - beltAfter.x) + Math.abs(beltBefore.y - beltAfter.y)
      + Math.abs(beltBefore.z - beltAfter.z) < 1e-8 && Math.abs(Math.abs(beltDot) - 1) < 1e-10,
      "the closed belt envelope stays on the housing when B turns");
    var pulleyDot = pulleyBefore.qx * pulleyAfter.qx + pulleyBefore.qy * pulleyAfter.qy
      + pulleyBefore.qz * pulleyAfter.qz + pulleyBefore.qw * pulleyAfter.qw;
    var motorDot = motorBefore.qx * motorAfter.qx + motorBefore.qy * motorAfter.qy
      + motorBefore.qz * motorAfter.qz + motorBefore.qw * motorAfter.qw;
    check(Math.abs(pulleyDot) < 0.99 && Math.abs(motorDot) < 0.99,
      "both pulleys rotate through the belt transmission without rotating the loop envelope");
    near(spindle.jointTorque(2), 2.0, 1e-9, "the 1:1 belt passes the motor torque unchanged");
    check(spindle.motorPulley.teeth == 40 && spindle.spindlePulley.teeth == 40 && Math.abs(spindle.belt.length - 450) < 1e-6 && spindle.belt.teeth == 90,
      "HTD5M 40T on the motor and on the spindle, one 450 mm (90 tooth) belt");
    near(HobSpindle.centres(), 125, 1e-6, "the 450-5M belt sits on 125 mm centres");
    near(HobSpindle.tensionAt(HobSpindle.spanFrequency(250)), 250, 1e-9, "span frequency and tension convert both ways");
    check(spindle.motorPulley.boreDiameter == spindle.motor.rating.shaftDiameter && spindle.spindlePulley.boreDiameter == 20,
      "pulley bores are the motor shaft and the Ø20 stub");
    check(HobSpindle.pulleyFront(spindle.motor.rating.shaftLength) - HobSpindle.PULLEY_WIDTH >= HobSpindle.SHAFT_REAR,
      "the cartridge stub reaches past the pulley");
    var cradle = new machinekit.hobbing.HobberSaddle("GE-motor-cradle"), cradleHoles = cradle.holes();
    check(machinekit.component.ManufacturingRoute.of(cradle) != null && cradleHoles.filter(h -> h.kind == "tapped").length == 1
      && cradleHoles.filter(h -> h.kind == "dowel").length == 0 && cradleHoles.filter(h -> h.kind == "clearance" && h.thread == "M8").length == 0
      && cradleHoles.filter(h -> h.kind == "clearance" && h.thread == "M6" && h.slotLength == 2 * HobSpindle.TENSION_TRAVEL).length == 4,
      "the servo cradle is a routed milled part with the jack screw's tapped M8, no floor bolts or dowels of its own (the machine derives them) and the flange's four M6 vertical slots");
    var purchase = Bt30SpindleData.catalog(), row = purchase.get(Bt30SpindleData.DESIGNATION), meta = purchase.metadata(Bt30SpindleData.DESIGNATION);
    near(row.number("outside"), 80, 0, "selected BT30 cartridge has the catalogue 80 mm body");
    near(row.number("maxRpm"), 6000, 0, "option A has catalogue 6000 rpm maximum");
    check(meta.source == Bt30SpindleData.SOURCE, "cartridge provenance follows the selected listing");
    check(meta.verifiedFields.indexOf("height") < 0 && meta.verifiedFields.indexOf("radialRunout") < 0,
      "unpublished length and accuracy are not claimed as verified supplier values");
    check(spindle.billOfMaterials().quantity(Bt30SpindleData.DESIGNATION) == 1, "one complete cartridge purchase");
    check(spindle.billOfMaterials().quantity("BT30-D80-INCLUDED-ROTOR") == 0, "rotor is included in the cartridge purchase");
    check(spindle.billOfMaterials().lines().filter(l -> l.partNumber.indexOf("7305") >= 0 || l.partNumber.indexOf("6205") >= 0).length == 0,
      "no separately purchased invented cartridge internals");
    check(spindle.interfaceName() == spindle.arbor.interfaceName(), "BT30 interfaces agree");
    check(spindle.interfaceName() == "BT30", "cartridge uses the standard BT30 taper");
    near(Bt30SpindleData.TAPER_DIAMETER, 31.75, 0, "BT30 gauge diameter");
    near(Bt30SpindleData.TAPER_LENGTH, 48.4, 0, "BT30 shank length");
    near(Bt30SpindleData.TAPER_RATIO, 7.0 / 24, 1e-12, "BT30 is a 7/24 diametral taper");
    near(Bt30SpindleData.HOLDER_LENGTH, 20, 0, "BT30 holder flange is 20 mm thick after the 2 mm gap");
    near(Bt30SpindleData.DRIVE_SLOT_WIDTH, 16.1, 0, "BT30 holder drive slot width");
    near(Bt30SpindleData.DRIVE_SLOT_FLOOR, 16.3, 0, "BT30 drive slot floor from axis");
    near(spindle.arbor.hobStart(), 40, 0, "hob shoulder is at 40 from gauge");
    near(spindle.arbor.hobEnd(), 80, 0, "hob retaining nut bears at 80 from gauge");
    near(spindle.arbor.parameterNumber("supportDiameter"), 15, 0, "outboard journal is 15 mm");
    near(spindle.arbor.parameterNumber("supportLength"), 29, 0, "outboard journal carries pair, inner spacer and M15 nut");
    var arb = spindle.arbor.geometry(), rotor = spindle.components().filter(c -> c.id == "shaft")[0].component.geometry();
    check(arb.solidCount() == 1, "BT30 arbor face key is rooted in one connected solid");
    // Cross-section probes prove the male taper has material immediately inside
    // its surface and none outside, independently of the generator's profile.
    for (z in [-40.0, -30.0, -20.0, -10.0, -1.0]) {
      var radius = 31.75 / 2 + z * 7 / 48;
      for (a in [0.0, Math.PI / 2, Math.PI, 3 * Math.PI / 2]) {
        var inside = cadkit.modeling.Part.cylinderSpan(0.03, z, z + 0.05, (radius - 0.1) * Math.cos(a), (radius - 0.1) * Math.sin(a));
        var outside = cadkit.modeling.Part.cylinderSpan(0.03, z, z + 0.05, (radius + 0.1) * Math.cos(a), (radius + 0.1) * Math.sin(a));
        var hit = arb.intersect(inside), miss = arb.intersect(outside);
        check(hit.volume() > 0.0001, "BT30 male taper material inside the nominal cone");
        check(miss.volume() < 1e-8, "BT30 male taper clears outside the nominal cone");
        hit.close(); miss.close(); inside.close(); outside.close();
      }
    }
    arb.close(); rotor.close();
    check(
      spindle.definition().joints.filter(j -> j.id == "B")[0].child == "shaft",
      "B drives shaft; arbor is retained to its tool face"
    );
    var cartridgePoses = spindle.solvedPoses();
    for (id in [
      "shaft", "encoder", "motor", "motorShaft", "motorPulley", "spindlePulley", "belt", "motorEncoder", "hobLocknut", "pullStud"
    ]) check(
      cartridgePoses.exists(id),
      "cartridge load-path part present: " + id
    );
    var arborBounds = spindle.arbor.geometry();
    near(
      arborBounds.shape.bounds().get_min().get_z(),
      -48.4,
      1e-6,
      "arbor includes the complete BT30 taper behind the gauge face"
    );
    arborBounds.close();
    rejects(
      () -> new SpindleHousing(new ComponentValues().setNumber("height", 190)),
      "unsupported cartridge dimensions refused"
    );
    var a = head.definition().joints.filter(j -> j.id == "A")[0];
    check(a.limits.lower == -Math.PI / 4 && a.limits.upper == Math.PI / 4, "swivel range");
    near(head.clamp.holdingTorque(), 100, 0, "stationary clamp rating separate from drive effort");
    var hob = new Hob(), rebuilt:Hob = cast hob.componentType().create(hob.values());
    var original = hob.design.rakeFaceRegion(3);
    var copied = rebuilt.design.rakeFaceRegion(3);
    check(original.length == copied.length, "hob recipe regenerates rake region");
    for (i in 0...original.length) near(
      Math.abs(original[i].x
      - copied[i].x) + Math.abs(original[i].y - copied[i].y) + Math.abs(original[i].z - copied[i].z),
      0,
      1e-12,
      "recovered hob rake points"
    );
    var derived = Hob.fromDesign(hob.design, hob.target);
    near(derived.design.leadAngle, hob.design.leadAngle, 1e-14, "part built from HobDesign");
    for (catalog in [
      TorqueMotor.catalog(),
      AngularContactBearing.catalog(),
      CrossedRollerBearing.catalog(),
      RotaryEncoderRing.catalog()
    ]) {
      var designation = catalog.designations()[0],
        row = catalog.get(designation), metadata = catalog.metadata(designation);
      check(
        row.token("ratingsBasis") == "Assumed" && row.token("errorsBasis") == "Assumed",
        "reference catalogue row explicitly assumed"
      );
      var fields = metadata.verifiedFields;
      check(fields != null && fields.length == 0, "no unverified fields called catalogue facts");
    }
    rejects(() -> {new RotaryTable(new TorqueMotor(new ComponentValues().setNumber("outside", 200)));
    }, "incompatible motor envelope rejected");
    rejects(() -> {new TorqueMotor(new ComponentValues().setNumber("peakTorque", 1));
    }, "peak below continuous torque rejected");
    rejects(() -> {new TorqueMotor(new ComponentValues().setNumber("maxSpeed", Math.NaN));
    }, "nonfinite motor speed rejected");
    rejects(() -> {new AngularContactBearing(new ComponentValues().setNumber("bore", 100));
    }, "bearing walls must exist");
    rejects(() -> {new RotaryEncoderRing(new ComponentValues().setInteger("countsPerRevolution", 0));
    }, "zero encoder counts rejected");
    rejects(() -> {new ExpandingMandrel(new ComponentValues().setNumber("oilPressure", 20000000));
    }, "mandrel overpressure rejected");
    rejects(() -> {new CrossedRollerBearing(new ComponentValues().setNumber("radialRunout", -1));
    }, "negative error bounds rejected");
    rejects(() -> {new TorqueMotor(new ComponentValues().setToken("ratingsBasis", "Catalog"));
    }, "catalogue claim requires source");
    var foundations:Array<MineralFoundation> = [new MineralCastBed(), new MineralCastColumn()];
    for (foundation in foundations) {
      foundation.validate();
      check(!FoundationIntegrity.check(foundation).hasErrors(), "insert positions and face planes conform");
      var weight = foundation.castingByWeight(),
        shape = foundation.body.geometry(), mass = foundation.body.massProperties();
      near(weight.kg, shape.volume() * 2400e-9, 1e-8, "casting purchased by native volume/density mass");
      var tensor = mass.inertia;
      if (tensor == null) throw "casting inertia absent";
      check(tensor.xx > 0 && tensor.yy > 0 && tensor.zz > 0, "casting inertia is volumetric");
      shape.close();
      check(foundation.billOfMaterials().lines().length > 4, "casting and steel inserts separately itemised");
      check(foundation.body.bom.description.indexOf(" kg") >= 0, "casting BOM shows purchasing weight");
      var bad = FoundationIntegrity.checkMounts(foundation, [{name: "bad rail mount", parent: "casting"}]);
      check(bad.hasErrors() && bad.items[0].subject == "bad rail mount", "bad precision mount named");
      var fixture = new MachineAssembly();
      fixture.include("foundation", foundation);
      fixture.addComponent("rail", LinearRail.metric("HGH15CA", 200));
      fixture.addMate("rail mount", "fixed", "foundation/railLeft", "top", "rail", "base");
      check(
        !FoundationIntegrity.checkMounts(
          fixture,
          [{name: "rail mount", parent: "foundation/railLeft"}],
          ["rail mount" => "rail mount"]
        ).hasErrors(),
        "precision rail mount actually lands on steel"
      );
      check(
        FoundationIntegrity.checkMounts(
          fixture,
          [{name: "rail mount", parent: "foundation/railLeft"}],
          ["rail mount" => "missing joint"]
        ).hasErrors(),
        "declared steel support does not excuse a missing physical connection"
      );
      fixture.addComponent("rogueRail", LinearRail.metric("HGH15CA", 200));
      fixture.addMate("rogue rail mount", "fixed", "foundation/casting", "top", "rogueRail", "base");
      check(
        FoundationIntegrity.checkMounts(fixture, []).items.filter(i -> i.subject == "rogue rail mount").length == 1,
        "unlisted precision mount on casting is detected"
      );
      var poses = foundation.solvedPoses(), pose = poses.get("railLeft");
      if (pose == null) throw "missing rail insert";
      poses.set(
        "railLeft",
        materia.assembly.AssemblyFrames.compose(materia.assembly.AssemblyFrames.translation(0, 0, 0.03), pose)
      );
      check(FoundationIntegrity.check(foundation, poses).hasErrors(), "insert out of position tolerance rejected");
      var rebuilt = MachineAssembly.decode(foundation.encode(), registry);
      check(rebuilt.components().length == foundation.components().length, "foundation child parts saved separately");
    }
    var physical = materia.project.MaterialLibrary.require("mineral-cast").physical;
    check(
      physical.youngsModulusPa == 35e9 && physical.thermalExpansionPerK == 14e-6 && physical.propertiesAssumed == true,
      "assumed bulk properties explicit"
    );
    check(
      physical.dampingLossFactor == 0.02 && physical.thermalConductivityWPerMK == 2,
      "damping and conductivity explicit"
    );
    Sys.println('HobberPartTests: $assertions assertions passed');
  }
  static function geometry():Void {
    for (p in members()) {
      var shape = p.geometry(Preview);
      try {
        check(shape.volume() > 0 && Math.isFinite(shape.volume()), 'finite solid volume ${p.designation}');
        check(shape.solidCount() == 1, 'single valid envelope ${p.designation}');
        var bounds = shape.shape.bounds();
        check(bounds.get_max().get_z() > bounds.get_min().get_z(), 'nondegenerate envelope ${p.designation}');
        if (Std.isOfType(p, MineralCasting)) near(
          p.massProperties().mass,
          shape.volume() * 1e-9 * 2400,
          1e-9,
          "mineral cast uses declared reference density"
        );
        Sys.println('${p.designation}: volume_mm3=${shape.volume()}');
      } catch (e:Dynamic) {
        shape.close();
        throw e;
      }
      shape.close();
    }
    var assemblies:Array<MachineAssembly> = [new RotaryTable(), new HobSpindle(), new SwivelHead()];
    for (assembly in assemblies) {
      var poses = assembly.solvedPoses(), members = assembly.components(), cache = new machinekit.assembly.PosedParts();
      var housing = members.filter(m -> m.id == "housing")[0];
      try {
        for (m in members) if (m.id != "housing") near(
          cache.overlap(housing.component, poses.get("housing"), m.component, poses.get(m.id)),
          0,
          1e-6,
          'housing seat clears ${m.id}'
        );
        if (Std.isOfType(assembly, HobSpindle)) for (i in 0...members.length) for (j in i + 1...members.length) near(
          cache.overlap(members[i].component, poses.get(members[i].id), members[j].component, poses.get(members[j].id)),
          0,
          1e-6,
          "cartridge parts do not penetrate: " + members[i].id + " / " + members[j].id
        );
      } catch (e:Dynamic) {
        cache.close();
        throw e;
      }
      cache.close();
    }
    Sys.println('Hobber geometry: $assertions assertions passed');
  }
  public static function main():Void {
    if (Sys.args().length > 0 && Sys.args()[0] == "hob-mesh") {
      HobMeshTests.run();
      return;
    }
    if (Sys.args().length > 0 && Sys.args()[0] == "hob-detail") {
      HobGeometryTests.run();
      return;
    }
    if (Sys.args().length > 0 && Sys.args()[0] == "geometry") geometry();
    else run();
  }
}
