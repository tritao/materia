import materia.casting.CastingFeature.CastingDimension;
import haxe.Json;
import materia.assembly.AssemblyFrames as F;
import materia.tooling.ToolingCodec;
import materia.tooling.ToolingEnums.ToolingLengthUnit;
import materia.tooling.ToolingEnums.CoatedFace;
import materia.tooling.ToolingEnums.StockDirection;
import materia.casting.InsertSpec.InsertRole;
import materia.casting.InsertSpec.InsertFinish;
import materia.tooling.ToolingDocument;
import materia.tooling.ToolingRecipe;
import materia.tooling.ToolingReview;
import materia.tooling.casting.CastingToolingInput;
import materia.casting.CastingFeature.CastingFeatureKind;
import materia.tooling.casting.CastingSetup.CoreStrategy;

/** Pure records/codec tests: this target has no CAD or MachineKit dependency. */
class ToolingTests {
  static var assertions = 0;

  static function items(value:Dynamic):Array<Dynamic> return cast value;

  static function check(ok:Bool, label:String):Void {
    assertions++;
    if (!ok) throw label;
  }

  static function rejects(fn:Void -> Void, expected:String):Void {
    var message:Null<String> = null;
    try fn() catch (e:Dynamic) message = Std.string(e);
    check(message != null && message.indexOf(expected) >= 0, 'Expected "$expected", got "$message"');
  }

  static function relievedFixing():Void {
    var hole = TestInsertHoles.through(5, 25)[0];
    hole.frame = F.identity();
    hole.extent = Blind;
    hole.thread = {
      designation: "M6",
      majorDiameter: 6,
      pitch: 1,
      starts: 1,
      hand: Right,
      internal: true,
      toleranceClass: "6H",
      profile: Metric
    };
    hole.tapDrill = 5;
    hole.threadDepth = 6;
    hole.clearanceDiameter = 6.6;
    hole.limits.threaded = {minEngagement: 6, minTipGap: 1};
    var hardware = fixture().recipe.hardware;
    check(
      materia.tooling.casting.InsertFixturePlanner.select(
        hole,
        F.identity(),
        F.fromRotationMatrix(0, 0, 0, [1, 0, 0, 0, -1, 0, 0, 0, -1]),
        hardware
      ) == null,
      "short blind thread has no tip clearance without an explicit relief"
    );
    hole.relief = {diameter: 6.2, start: 6};
    var fixing = materia.tooling.casting.InsertFixturePlanner.select(
      hole,
      F.identity(),
      F.fromRotationMatrix(0, 0, 0, [1, 0, 0, 0, -1, 0, 0, 0, -1]),
      hardware
    );
    check(
      fixing != null && fixing.engagement == 6 && fixing.clearanceDiameter == 6.6 && fixing.plug == null,
      "source relief permits short-thread fixing without a buried plug"
    );
    hole.limits.threaded.minEngagement = 7;
    check(
      materia.tooling.casting.InsertFixturePlanner.select(
        hole,
        F.identity(),
        F.fromRotationMatrix(0, 0, 0, [1, 0, 0, 0, -1, 0, 0, 0, -1]),
        hardware
      ) == null,
      "deep relief never contributes thread engagement"
    );
  }

  static function boardLimitedStrategy():Void {
    var document = fixture(), feature = document.input.features[0];
    feature.kind = ServiceOpening;
    for (rule in document.recipe.featureRules) rule.useEPSAboveBoardThickness = true;

    var selected = materia.tooling.casting.CastingStrategies.resolve(document)[0];
    check(
      selected.strategy == SacrificialEPS,
      "A deep body-open pocket uses EPS by recipe, without a feature-ID override"
    );
    feature.geometry.solid = materia.casting.CastingFeature.CastingGeometrySolid.Cut(materia.casting.CastingSolid.Box(
      10,
      80,
      61
    ));
    selected = materia.tooling.casting.CastingStrategies.resolve(document)[0];
    check(
      selected.strategy == SacrificialEPS,
      "A thin dimension without an opening cannot turn a deep pocket into a board former"
    );
    feature.geometry.solid = materia.casting.CastingFeature.CastingGeometrySolid.Cut(materia.casting.CastingSolid.Box(
      100,
      80,
      11
    ));
    selected = materia.tooling.casting.CastingStrategies.resolve(document)[0];
    check(selected.strategy == ReusableFormer, "A shallow opening fits the declared board");
    feature.geometry.pose.z = -100;
    feature.geometry.solid = materia.casting.CastingFeature.CastingGeometrySolid.Cut(materia.casting.CastingSolid.Box(
      100,
      80,
      110
    ));
    selected = materia.tooling.casting.CastingStrategies.resolve(document)[0];
    check(
      selected.strategy == ReusableFormer,
      "Stock selection uses the finished 10 mm cavity, not the 110 mm intent cut"
    );
    feature.geometry.pose.z = -1;
    feature.geometry.solid = materia.casting.CastingFeature.CastingGeometrySolid.Cut(materia.casting.CastingSolid.Box(
      100,
      80,
      61
    ));
    document.recipe.former.stocks[0].measuredThickness = 70;
    selected = materia.tooling.casting.CastingStrategies.resolve(document)[0];
    check(
      selected.strategy == ReusableFormer,
      "Changing declared stock thickness changes the rule, not source geometry"
    );
    var raw:Array<Dynamic> = cast haxe.Json.parse(haxe.Json.stringify(document.recipe.featureRules));
    Reflect.setField(raw[0], "useEPSAboveBoardThickness", "yes");
    rejects(() -> materia.tooling.casting.CastingStrategies.snapshotRules(raw), "Invalid casting strategy board limit");
  }

  static function epsFillPortal():Void {
    var document = fixture();
    var feature = materia.casting.CastingPatterns.box(
      "multiple-portals",
      Lightening,
      [for (face in ["body.y-min",
      "body.y-max", "body.z-min", "body.z-max"]) materia.casting.CastingPatterns.face(face)],
      100,
      180,
      100
    );
    document.input.features = [feature];
    document.input.inserts = [];

    for (rule in document.recipe.featureRules) rule.preferredAxis = X;
    document.setup.sourceFingerprint = ToolingCodec.inputFingerprint(document.input);
    var selected = materia.tooling.casting.CastingStrategies.resolve(document);
    check(
      selected[0].withdrawal != null && selected[0].withdrawal.z == 1,
      "EPS with a fill-face opening uses that declared portal despite other open sides"
    );
    feature.openings.reverse();
    check(
      ToolingCodec.canonical(selected)
      == ToolingCodec.canonical(materia.tooling.casting.CastingStrategies.resolve(document)),
      "EPS fill access does not depend on opening order"
    );
    feature.kind = Clearance;
    feature.draft.axis = {x: 1, y: 0, z: 0};
    rejects(() -> materia.tooling.casting.CastingStrategies.resolve(document), "Ambiguous feature withdrawal");
  }

  static function routedSocketFaces():Void {
    var rotations = [
      [1.0, 0, 0, 0, 1, 0, 0, 0, 1],
      [1.0, 0, 0, 0, -1, 0, 0, 0, -1],
      [1.0, 0, 0, 0, 0, -1, 0, 1, 0],
      [1.0, 0, 0, 0, 0, 1, 0, -1, 0],
      [0.0, 0, 1, 0, 1, 0, -1, 0, 0],
      [0.0, 0, -1, 0, 1, 0, 1, 0, 0]
    ];
    for (rotation in rotations) for (positive in [false, true]) {
      var r = fixture().recipe;
      r.minimumLigament = 2;
      var panel = materia.tooling.panel.PanelJoints.blank(
        "socket-receiver",
        100,
        100,
        r.panel,
        r.routerDiameter,
        r.minimumLigament
      );
      panel.contactFace = positive
        ? materia.tooling.ToolingEnums.ContactFace.PositiveZ : materia.tooling.ToolingEnums.ContactFace.NegativeZ;
      panel.labelSide = positive
        ? materia.tooling.ToolingEnums.MachiningSide.NegativeZ : materia.tooling.ToolingEnums.MachiningSide.PositiveZ;
      var receiver:materia.tooling.panel.PanelAssemblyPlan.PanelPlacement = {
        panel: panel,
        pose: F.fromRotationMatrix(31, -27, 92, rotation),
        withdrawal: {x: 0, y: 0, z: 1}
      };
      var frame = materia.tooling.casting.CoreSupports.socketFrame(receiver);
      var cavity = materia.tooling.casting.CastingMoldPlanner.box(
        60,
        40,
        30,
        F.compose(frame, F.translation(50, positive ? 50 : -50, 0))
      );
      var built = materia.tooling.casting.CoreSupports.routedSocket(cavity, "side-pocket", r, receiver);
      var core = built.core,
        operation = receiver.panel.operations.filter(op -> op.id == core.support.socketOperationId)[0];
      check(
        operation.side == (positive
        ? materia.tooling.ToolingEnums.MachiningSide.PositiveZ : materia.tooling.ToolingEnums.MachiningSide.NegativeZ),
        "Socket opens on the actual receiving face"
      );
      check(
        built.penetration.panelId == panel.id && built.penetration.coreId == core.id,
        "Every socket penetration names its core and receiver"
      );
      var actual = materia.tooling.casting.MoldCores.singleBox(core.cavity, core.id);
      check(
        Math.abs(actual.x - cavity.x) + Math.abs(actual.y - cavity.y) + Math.abs(actual.z - cavity.z) + Math.abs(actual
        .width - cavity.width) + Math.abs(actual.depth - cavity.depth) + Math.abs(actual.height - cavity.height) < 1e-6,
        "Routed support preserves the exact source cavity on every Cartesian face"
      );
      var foot:materia.tooling.casting.CoreSupport.CoreFoot = cast core.support.foot;
      check(foot != null && core.support.panelIds[0] == panel.id, "Routed EPS foot belongs to its actual panel");
      var origin = F.transformPoint(
        F.inverse(frame),
        foot.cutting.stockToMold.x,
        foot.cutting.stockToMold.y,
        foot.cutting.stockToMold.z
      );
      check(
        origin.z < 0 && origin.z > -r.panel.measuredThickness,
        "Foot stock occupies only the receiving panel socket"
      );
    }
  }

  static function fixtureFaces():Void {
    var document = fixture(), hole = document.input.inserts[0].holes[0];
    hole.diameter = 5;
    hole.tapDrill = 5;
    hole.depth = 12;
    hole.extent = Blind;
    hole.threadDepth = 9;
    hole.thread = {
      designation: "M6",
      majorDiameter: 6,
      pitch: 1,
      starts: 1,
      hand: Right,
      internal: true,
      toleranceClass: "6H",
      profile: Metric
    };
    hole.clearanceDiameter = 6.6;
    hole.limits.threaded = {minEngagement: 6, minTipGap: 1};
    hole.frame = F.fromRotationMatrix(0, 0, 20, [1, 0, 0, 0, -1, 0, 0, 0, -1]);
    var rotations = [
      [1.0, 0, 0, 0, 1, 0, 0, 0, 1],
      [1.0, 0, 0, 0, -1, 0, 0, 0, -1],
      [1.0, 0, 0, 0, 0, -1, 0, 1, 0],
      [1.0, 0, 0, 0, 0, 1, 0, -1, 0],
      [0.0, 0, 1, 0, 1, 0, -1, 0, 0],
      [0.0, 0, -1, 0, 1, 0, 1, 0, 0]
    ];
    for (rotation in rotations) {
      var pose = F.fromRotationMatrix(31, -27, 92, rotation);
      var face = F.compose(pose, F.translation(0, 0, 20));
      var selected = materia.tooling.casting.InsertFixturePlanner.select(hole, pose, face, document.recipe.hardware);
      check(
        selected != null && selected.engagement == 6 && selected.clearanceDiameter == 6.6,
        "Source fixing works on every Cartesian support face"
      );
      var wrongFace = F.compose(face, F.fromRotationMatrix(0, 0, 0, [1, 0, 0, 0, -1, 0, 0, 0, -1]));
      check(
        materia.tooling.casting.InsertFixturePlanner.select(hole, pose, wrongFace, document.recipe.hardware) == null,
        "Opposite face orientation cannot select the source thread"
      );
      check(
        materia.tooling.casting.InsertFixturePlanner.select(
          hole,
          pose,
          F.compose(face, F.translation(0, 0, 1)),
          document.recipe.hardware
        ) == null,
        "A different parallel support plane cannot select the source thread"
      );
    }
  }

  static function fixturePanelFaces():Void {
    var rectangle = materia.tooling.panel.PanelContours.rectangle(0, 0, 100, 40);
    check(
      materia.casting.CastingProfileGeometry.containsRect(rectangle, 20, 0, 40, 40),
      "Receiver footprint may touch its straight panel boundary"
    );
    check(
      !materia.casting.CastingProfileGeometry.containsRect(rectangle, 20, -0.01, 40, 40.01),
      "A real footprint breakout remains rejected"
    );
    var rotations = [
      [1.0, 0, 0, 0, 1, 0, 0, 0, 1],
      [1.0, 0, 0, 0, -1, 0, 0, 0, -1],
      [1.0, 0, 0, 0, 0, -1, 0, 1, 0],
      [1.0, 0, 0, 0, 0, 1, 0, -1, 0],
      [0.0, 0, 1, 0, 1, 0, -1, 0, 0],
      [0.0, 0, -1, 0, 1, 0, 1, 0, 0]
    ];
    var sample = fixture().input.inserts[0];
    var size = materia.casting.InsertSpec.InsertBodyCodec.box(sample.body, sample.id);
    var shapes = [
      materia.casting.CastingSolid.Box(size.width, size.depth, size.height),
      materia.casting.CastingSolid.Cylinder(16, size.height),
      materia.casting.CastingSolid.Cylinder(8, size.height)
    ];
    for (shape in shapes) for (rotation in rotations) {
      var document = fixture(), source = document.input.inserts[0];
      source.body = materia.casting.InsertSpec.InsertBody.Primitive(shape);
      var diameter:Null<Float> = switch shape {
        case Cylinder(d, _):
          d;
        case Box(_, _, _):
          null;
      };
      var thin = diameter != null && diameter == 8;
      if (thin) {
        source.holes[0].diameter = 6;
        source.holes[0].clearanceDiameter = 6.6;
      }
      var transform = F.fromRotationMatrix(31, -27, 92, rotation), stock = F.compose(transform, source.pose);
      var face = F.compose(stock,
        F.translation(0, 0, size.height)), thickness = document.recipe.panel.measuredThickness;
      var pose = F.compose(
        F.compose(face, F.fromRotationMatrix(0, 0, 0, [1, 0, 0, 0, -1, 0, 0, 0, -1])),
        F.translation(-50, -50, -thickness)
      );
      var panel = materia.tooling.panel.PanelJoints.blank("face-fixture", 100, 100, document.recipe.panel, 6, 2);
      panel.contactFace = materia.tooling.ToolingEnums.ContactFace.PositiveZ;
      panel.operations.push(materia.tooling.panel.PanelJoints.operation(
        "mask-access",
        materia.tooling.panel.PanelPlan.PanelOperationKind.Drill,
        materia.tooling.panel.PanelContours.circle(50, 50, thin ? 3.3 : 4.5),
        thickness,
        thin ? 6.6 : 9,
        materia.tooling.ToolingEnums.MachiningSide.PositiveZ,
        true
      ));
      var location = materia.tooling.casting.DatumFixtureLocation.onPanel(
        source,
        transform,
        face,
        {panel: panel, pose: pose, withdrawal: {x: 0, y: 0, z: 1}},
        []
      );
      check(
        location.unsupportedReasons.length == 0 && location.report.rank == 3,
        "Every Cartesian panel face supplies only the three real plane constraints"
      );
      check(location.contacts.length == 3, "Clearance access hole receives no lateral locating credit");
      var placement = {panel: panel, pose: pose, withdrawal: {x: 0.0, y: 0.0, z: 1.0}};
      check(
        materia.tooling.casting.DatumFixtures.panelOnFace([placement], face) == placement,
        "Receiver selection follows every Cartesian mounting plane"
      );
      check(
        materia.tooling.casting.DatumFixtures.panelOnFace([placement], F.compose(face, F.translation(0, 0, 1))) == null,
        "Parallel receiver off the mounting plane is rejected"
      );
      rejects(
        () -> materia.tooling.casting.DatumFixtures.panelOnFace([placement, placement], face),
        "multiple receiver panels"
      );
      panel.contactFace = materia.tooling.ToolingEnums.ContactFace.NegativeZ;
      placement.pose = F.compose(face, F.translation(-50, -50, 0));
      check(
        materia.tooling.casting.DatumFixtures.panelOnFace([placement], face) == placement,
        "Negative-Z wall contact selects the same exact mounting plane"
      );
      var wallLocation = materia.tooling.casting.DatumFixtureLocation.onPanel(source, transform, face, placement, []);
      check(
        wallLocation.unsupportedReasons.length == 0 && wallLocation.report.rank == 3,
        "Wall receiver preserves three real plane constraints"
      );
      for (contact in location.contacts) {
        var point = F.transformPoint(F.inverse(stock), contact.point.x, contact.point.y, contact.point.z);
        check(Math.abs(point.z - size.height) < 1e-7, "Contact witness lies on the actual steel mouth plane");
        check(
          diameter != null ? point.x * point.x + point.y * point.y <= diameter * diameter / 4 + 1e-7
          : Math.abs(point.x) <= size.width / 2 && Math.abs(point.y) <= size.depth / 2,
          "Contact witness lies on real steel, including circular mouths"
        );
        check(point.x * point.x + point.y * point.y > (thin ? 9 : 16), "Contact witness avoids the source bore");
      }
    }
  }

  static function unresolvedStockFixing():Void {
    var d = fixture(), insert = d.input.inserts[0], hole = insert.holes[0];
    insert.body = materia.casting.InsertSpec.InsertBody.Primitive(materia.casting.CastingSolid.Box(40, 30, 21));
    insert.interfaces = [{
      id: "finished-top",
      geometry: materia.casting.InterfaceRequirement.PrecisionGeometry.Plane(F.translation(0, 0, 20)),
      route: materia.casting.InterfaceRequirement.InterfaceRoute.Ground,
      allowanceMm: 1,
      finishingProcessErrorMm: materia.casting.CastingInput.Unresolved("Finishing process error allocation"),
      permittedFinishes: [materia.casting.InterfaceRequirement.InterfaceFinish.Ground],
      positionToleranceMm: 0.01,
      datumReference: "primary",
      relationshipIds: []
    }];
    hole.frame = F.fromRotationMatrix(0, 0, 20, [1, 0, 0, 0, -1, 0, 0, 0, -1]);
    hole.diameter = 5;
    hole.depth = 12;
    hole.extent = Blind;
    hole.threadDepth = 9;
    hole.tapDrill = 5;
    hole.clearanceDiameter = 6.6;
    hole.thread = {
      designation: "M6",
      majorDiameter: 6,
      pitch: 1,
      starts: 1,
      hand: Right,
      internal: true,
      toleranceClass: "6H",
      profile: Metric
    };
    hole.limits.threaded = {minEngagement: 6, minTipGap: 1};
    switch insert.origin {
      case Fabricated(manufacturing):
        manufacturing.holeRoutes = [];
      case Catalogue(_):
        throw "Fabricated fixture expected";
    }
    var pose = F.fromRotationMatrix(0, 0, 21, [1, 0, 0, 0, -1, 0, 0, 0, -1]);
    var face = F.fromRotationMatrix(0, 0, 0, [1, 0, 0, 0, -1, 0, 0, 0, -1]);
    var decision = materia.tooling.casting.InsertFixturePlanner.missingFixing(insert, pose, face, d.recipe.hardware);
    check(
      decision.change == null && decision.check.rule == "tooling.fixture.hole-stage.seat",
      "unknown hole timing through finishing stock needs a route, not a new source thread"
    );
    check(
      materia.tooling.casting.InsertFixturePlanner.holes(insert).length == 0,
      "hypothetical route evaluation does not create a physical hole"
    );
    TestInsertHoles.beforeCasting(insert);
    var precast = materia.tooling.casting.InsertFixturePlanner.holes(insert);
    check(
      precast.length == 1 && materia.tooling.casting.InsertFixturePlanner.select(
        precast[0],
        pose,
        face,
        d.recipe.hardware
      ) != null,
      "declaring the source hole pre-cast clears the route blocker with its raw-stock entry"
    );
  }

  static function sharedFixturePlate():Void {
    var d = fixture(), hole = d.input.inserts[0].holes[0];
    d.input.inserts[0].pose.z = 80;
    d.input.features.filter(feature -> feature.owner == "seat")[0].geometry.pose.z = 80;
    hole.extent = Blind;
    hole.depth = 16;
    hole.threadDepth = 12;
    hole.tapDrill = 6.8;
    hole.thread = {
      designation: "M8",
      majorDiameter: 8,
      pitch: 1.25,
      starts: 1,
      hand: Right,
      internal: true,
      toleranceClass: "6H",
      profile: Metric
    };
    hole.diameter = 6.8;
    hole.clearanceDiameter = 9;
    hole.limits.threaded = {minEngagement: 8, minTipGap: 1};
    d.setup.sourceFingerprint = ToolingCodec.inputFingerprint(d.input);
    var options:materia.tooling.casting.InsertFixturePlanner.InsertPlateOptions = {
      id: "test-carrier",
      insertIds: ["seat"],
      contactZ: 0,
      edgeMargin: 50,
      locatorEndInset: 40,
      locatorSideInset: 20,
      screwStock: [{
        thread: cast hole.thread,
        lengths: [25, 30, 35],
        threadedLength: 28,
        washerThickness: 1.6,
        driverDiameter: 16,
        driverToolDiameter: 6
      }]
    };
    var plan = materia.tooling.casting.InsertFixturePlanner.plate(d, options);
    check(
      plan.sites.length == 1 && plan.sites[0].sourceHoleId == hole.id && plan.sites[0].screwLength == 30,
      "shared plate derives fixing site and selects stock screw from source depth budget"
    );
    check(
      plan.penetrations.filter(pen -> pen.kind == InsertSealed && pen.sourceHoleId == hole.id
      && pen.sealingFace != null && pen.plug == null && pen.sealMaterial == d.recipe.hardware.sealMaterial).length == 1,
      "shared carrier records the source-thread penetration and its insert sealing face"
    );
    check(
      plan.assembly.members.length == 2
      && plan.assembly.members[1].pose.z == options.contactZ - d.recipe.panel.measuredThickness,
      "shared plate thickness and placement derive from measured stock"
    );
    check(
      !materia.tooling.ToolingValidation.ToolingReadiness.evaluate(plan.validation).ready,
      "nominal fixture geometry does not qualify seal and accuracy"
    );
    var counterbored = ToolingCodec.copy(d);
    counterbored.input.inserts[0].holes[0].counterbore = {diameter: 12, depth: 4};
    counterbored.input.inserts[0].holes[0].threadDepth = 14;
    counterbored.setup.sourceFingerprint = ToolingCodec.inputFingerprint(counterbored.input);
    options.screwStock[0].lengths = [30, 32];
    var bored = materia.tooling.casting.InsertFixturePlanner.plate(counterbored, options);
    check(
      bored.sites[0].screwLength == 32 && bored.sites[0].insertion >= 12,
      "carrier screw selection reaches full engagement beyond the source counterbore"
    );
    options.screwStock[0].lengths = [30];
    options.screwStock[0].threadedLength = 4;
    rejects(() -> materia.tooling.casting.InsertFixturePlanner.plate(d, options), "No stock fixture screw satisfies");
    options.screwStock[0].threadedLength = 28;
    options.screwStock[0].lengths = [25, 35];
    materia.tooling.casting.InsertFixturePlanner.validatePlate(plan);
    check(plan.options.screwStock[0].lengths.length == 3, "fixture keeps a detached copy of declared stock choices");
    plan.sites[0].screwLength = 35;
    rejects(() -> materia.tooling.casting.InsertFixturePlanner.validatePlate(plan), "Insert fixture plan differs");
    plan.sites[0].screwLength = 30;
    rejects(() -> materia.tooling.casting.InsertFixturePlanner.plate(d, options), "No stock fixture screw satisfies");
    options.screwStock[0].lengths = [30];
    options.insertIds = ["seat", "seat"];
    rejects(() -> materia.tooling.casting.InsertFixturePlanner.plate(d, options), "Duplicate fixture insert");
    options.insertIds = ["seat"];
    var side = materia.casting.InsertHole.InsertHoleCodec.snapshot([hole])[0];
    side.id = "side.service";
    side.frame = F.fromRotationMatrix(20, 0, 10, [0, 0, -1, 0, 1, 0, 1, 0, 0]);
    side.depth = 12;
    side.threadDepth = 10;
    d.input.inserts[0].holes.push(side);
    TestInsertHoles.beforeCasting(d.input.inserts[0]);
    d.setup.sourceFingerprint = ToolingCodec.inputFingerprint(d.input);
    var withMask = materia.tooling.casting.InsertFixturePlanner.plate(d, options);
    check(
      withMask.sites.length == 1 && withMask.masks.length == 1 && withMask.masks[0].protection.sourceHoleId == side.id,
      "unused source hole retains protection, including a valid derived ID for a dotted hole ID"
    );
    d.input.inserts[0].holes = [];
    TestInsertHoles.beforeCasting(d.input.inserts[0]);
    d.setup.sourceFingerprint = ToolingCodec.inputFingerprint(d.input);
    var absent = materia.tooling.casting.InsertFixturePlanner.plate(d, options);
    check(
      absent.sites.length == 0 && absent.changeRequests.length == 1
      && absent.validation.filter(row -> row.rule == "tooling.fixture.source-hole.seat").length == 1,
      "missing source thread blocks fixture without inventing a hole"
    );
  }

  static function nominalProduct():Void {
    var document = fixture();
    var product:materia.casting.InsertOrigin.CatalogueInsert = {
      manufacturer: "fixture supplier",
      reference: "fixture-insert",
      revision: "fixture-v1",
      holePolicyId: document.input.inserts[0].holes[0].limits.policyId,
      document: "fixture-drawing",
      geometryBasis: materia.casting.InsertOrigin.InsertGeometryBasis.NominalStudy,
      geometryFingerprint: null,
      evidence: []
    };
    document.input.inserts[0].origin = Catalogue(product);
    document.setup.sourceFingerprint = ToolingCodec.inputFingerprint(document.input);
    var review = ToolingReview.inspect(document);
    check(
      review.filter(row -> row.ruleId == "tooling.insert.geometry.seat" && row.status == "pending").length == 1,
      "input review retains nominal product geometry as pending"
    );
    var source = materia.tooling.casting.CastingMoldPlanner.sourceSupport(document.input);
    check(
      !materia.tooling.ToolingValidation.ToolingReadiness.evaluate(source).ready,
      "exactly supported box geometry does not clear nominal product qualification"
    );
    var fingerprint = ToolingCodec.inputFingerprint(document.input);
    product.reference = "different-fixture-product";
    check(
      ToolingCodec.inputFingerprint(document.input) != fingerprint,
      "product identity enters the source fingerprint"
    );
    product.reference = "fixture-insert";
    product.revision = "fixture-v2";
    check(
      ToolingCodec.inputFingerprint(document.input) != fingerprint,
      "catalogue revision enters the source fingerprint"
    );
    product.revision = "fixture-v1";
    product.holePolicyId = "other-policy";
    rejects(() -> ToolingCodec.copy(document), "Catalogue insert hole policy differs from product declaration");
    product.holePolicyId = document.input.inserts[0].holes[0].limits.policyId;
    document.input.inserts[0].origin = null;
    rejects(() -> ToolingCodec.copy(document), "Missing insert origin");
  }

  static function fixture():ToolingDocument {
    var input:CastingToolingInput = {
      id: "sample-foundation",
      revision: 1,
      lengthUnit: ToolingLengthUnit.Millimetres,
      datumScheme: null,
      interfaces: [],
      interfaceContext: null,
      postCureInserts: [],
      liftingPoints: [],
      envelope: [{
        id: "body",
        profile: [{x: -100, y: -90}, {x: 100, y: -90}, {x: 100, y: 90}, {x: -100, y: 90}],
        height: 100,
        pose: F.identity()
      }],
      features: [
        {
          id: "pocket",
          kind: materia.casting.CastingFeature.CastingFeatureKind.Lightening,
          owner: null,
          geometry: {
            solid: materia.casting.CastingFeature.CastingGeometrySolid.Cut(materia.casting.CastingSolid.Box(100,
      80, 61)),
      pose: F.translation(0, 0, -1)
    },
      openings: [materia.casting.CastingPatterns.face("body.z-min")],
      draft: {degrees: 0, axis: {x: 0, y: 0, z: 1}},
      cornerRadius: 0,
      adjustable: [CastingDimension.Width, CastingDimension.Depth, CastingDimension.Height]
    },
      {
        id: "insert-seat",
        kind: InsertRecess,
        owner: "seat",
        geometry: {
          solid: materia.casting.CastingFeature
      .CastingGeometrySolid.Insert(materia.casting.InsertSpec.InsertBody.Primitive(materia.casting.CastingSolid.Box(
        40,
        30,
        20
      ))),
      pose: F.translation(80, 0, 90)
    },
      openings: [
        materia.casting.CastingPatterns.face("body.z-max"),
        materia.casting.CastingPatterns.face("body.x-max")
      ],
      draft: {degrees: 0, axis: {x: 0, y: 0, z: 1}},
      cornerRadius: 0,
      adjustable: [CastingDimension.Width, CastingDimension.Depth, CastingDimension.Height]
    }
    ],
      inserts: [{
        id: "seat",
        body: materia.casting.InsertSpec.InsertBody.Primitive(materia.casting.CastingSolid.Box(40, 30, 20)),
        anchorage: materia.casting.InsertAnchorage.Undeclared,
        origin: materia.casting.InsertOrigin.Fabricated(materia.casting.InsertManufacturing
      .InsertManufacturingCodec.draft(materia.casting.InsertMaterial.InsertMaterials.specification(C45Bright))),
      interfaces: [],
      holes: TestInsertHoles.through(8, 20),
      pose: F.translation(80, 0, 90),
      role: InsertRole.GuideSeat,
      label: "rail seat",
      positionTolerance: 0.01,
      material: materia.casting.InsertMaterial.InsertMaterials.specification(C45Bright),
      finish: InsertFinish.GroundAfterCasting
    }]
    };
    TestInsertHoles.beforeCasting(input.inserts[0]);
    var stock:materia.tooling.panel.PanelStockSpec = {
      materialSpec: "melamine-faced particleboard, study",
      nominalThickness: 18,
      measuredThickness: 17.8,
      thicknessVariation: 0.2,
      width: 2440,
      height: 1220,
      coatedFace: CoatedFace.Both,
      allowedRotations: [0, 90],
      directionConstraint: StockDirection.Free
    };
    var formerStocks = [stock];
    if (stock.measuredThickness < 25) {
      var thicker = materia.tooling.ToolingCodec.stockSnapshot(stock);
      thicker.nominalThickness = 25;
      thicker.measuredThickness = 25;
      formerStocks.push(thicker);
    }
    var recipe:ToolingRecipe = {
      former: {
        pocketScrew: {diameterMm: 4, lengthMm: 30, jigAngleDegrees: 15, edgeDistanceMm: 20, maximumPitchMm: 100},
        stocks: formerStocks,
        finishingStockMm: 0.5,
        toleranceMm: 0.1,
        cutters: [
          {id: "former-straight-6", tipDiameterMm: 6, taperDegrees: 0, fluteLengthMm: 30, stickOutMm: 35},
          {id: "former-taper-3deg-6", tipDiameterMm: 6, taperDegrees: 3, fluteLengthMm: 30, stickOutMm: 35}
        ]
      },
      featureRules: materia.tooling.casting.CastingStrategies.standardRules(),
      id: "eps-panel-study",
      revision: 1,
      panel: stock,
      core: {
        materialSpec: "EPS grade unselected",
        width: 1000,
        depth: 500,
        height: 100,
        densityKgM3: null,
        compressionStrengthPa: null,
        compressionStrainFraction: null,
        maximumTemperatureC: null,
        evidence: []
      },
      jointLiningAllowance: 0.0,
      jointMachiningAllowance: 0.05,
      coreWireAxis: materia.tooling.ToolingRecipe.CoreWireAxis.Y,
      adhesiveLineThickness: 0.2,
      mixDensityKgM3: 2400,
      steelDensityKgM3: 7850,
      handlingLoadFactor: 1.5,
      hardware: {
        plugDepth: 5,
        plugMaterial: "steel",
        sealMargin: 3,
        sealMaterial: "pending compatible sealing compound",
        minimumBridgeWidth: 40,
        locatingPinDiameter: 6,
        supportScrewDiameter: 6,
        supportClearanceDiameter: 7,
        secondaryPinTravel: 2,
        padDepth: 54,
        labelOffset: 8,
        datumThickness: 8,
        datumPostWidth: 20,
        formerFlangeThickness: 12,
        formerFlangeMargin: 10,
        fixturePinDiameter: 3,
        fixturePinEmbed: 3,
        fixtureReceiver: materia.tooling.ToolingRecipe.FixtureReceiver.IndependentDatums,
        fixturePinSpacing: 8,
        fixtureSlotToolDiameter: 3
      },
      jointClearance: 0.3,
      linerThickness: 0.1,
      routerDiameter: 6,
      minimumLigament: 15,
      resinSpec: "binder unselected",
      releaseSystem: materia.tooling.ReleaseSystem.ReleaseSystems.unresolved(
        2,
        5,
        1,
        materia.tooling.ReleaseSystem.EPSCoveringKind.Tape
      ),
      adhesiveSpec: "adhesive unselected",
      processEvidence: [],
      designReview: {
        minimumCornerRadiusMm: null,
        minimumInsertCoverMm: null,
        minimumInsertEdgeMm: null,
        minimumFixingAccessMm: null,
        minimumLiftingPoints: null,
        removalClearanceMm: null
      }
    };
    return {
      input: input,
      recipe: recipe,
      setup: {
        id: "upside-down",
        revision: 1,
        sourceFingerprint: ToolingCodec.inputFingerprint(input),
        fillHeightMm: 100,
        freeboardMm: 0,
        sourceToMold: F.fromRotationMatrix(0, 0, 100, [1, 0, 0, 0, -1, 0, 0, 0, -1])
      }
    };
  }

  static function withdrawalTie():Void {
    var d = fixture();
    d.input.features = d.input.features.filter(f -> f.kind
      == materia.casting.CastingFeature.CastingFeatureKind.InsertRecess);
    d.input.features.push(materia.casting.CastingPatterns.box(
      "through-opening",
      ServiceOpening,
      [materia.casting.CastingPatterns.face("body.z-min"), materia.casting.CastingPatterns.face("body.z-max")],
      20,
      20,
      100
    ));

    var through = d.input.features.filter(feature -> feature.id == "through-opening")[0];
    through.draft.axis = {x: 1, y: 0, z: 0};
    d.setup.sourceFingerprint = ToolingCodec.inputFingerprint(d.input);
    rejects(() -> ToolingCodec.copy(d), "Ambiguous feature withdrawal");
    through.draft.axis = {x: 0, y: 0, z: -1};
    d.setup.sourceFingerprint = ToolingCodec.inputFingerprint(d.input);
    var nominated = materia.tooling.casting.CastingStrategies.resolve(ToolingCodec.copy(d));
    var expected = materia.assembly.AssemblyFrames.transformVector(d.setup.sourceToMold, 0, 0, -1);
    var actual = nominated[0].withdrawal;
    check(
      actual != null
      && Math.abs(actual.x - expected.x) + Math.abs(actual.y - expected.y) + Math.abs(actual.z - expected.z) < 1e-8,
      "source axis chooses the intended end and follows the source-to-mold orientation without an ID-keyed override"
    );
    check(
      materia.tooling.casting.CastingStrategies.standardRules().filter(rule -> rule.priority != 0).length == 0,
      "generic strategy rules contain no machine-specific ownership priorities"
    );
  }

  static function vocabularyRoundTrips():Void {
    var d = fixture();
    for (role in materia.casting.InsertSpec.InsertRoles.values()) {
      d.input.inserts[0].role = role;
      check(
        ToolingCodec.decodeInput(ToolingCodec.encodeInput(d.input)).inserts[0].role == role,
        "Every insert role token survives the source codec"
      );
    }
    for (finish in materia.casting.InsertSpec.InsertFinishes.values()) {
      d.input.inserts[0].finish = finish;
      check(
        ToolingCodec.decodeInput(ToolingCodec.encodeInput(d.input)).inserts[0].finish == finish,
        "Every insert finish token survives the source codec"
      );
    }
    for (grade in materia.casting.InsertMaterial.InsertMaterials.values()) {
      var material = materia.casting.InsertMaterial.InsertMaterials.specification(grade);
      check(
        materia.casting.InsertMaterial.InsertMaterials.snapshot(haxe.Json.parse(haxe.Json.stringify(material))).grade
        == grade,
        "Every stock token survives its codec"
      );
    }
    for (kind in materia.casting.CastingFeature.CastingFeatureKinds.values()) {
      var feature = materia.casting.CastingPatterns.box(
        "vocabulary",
        kind,
        [],
        10,
        10,
        10,
        0,
        0,
        0,
        null,
        materia.casting.CastingFeature.CastingDimensions.values()
      );
      if (kind == CastingFeatureKind.InsertRecess) feature.geometry.solid = materia.casting.CastingFeature
        .CastingGeometrySolid.Insert(materia.casting.InsertSpec.InsertBody.Primitive(materia.casting.CastingSolid.Box(
          10,
          10,
          10
        )));
      var round = materia.casting.CastingFeatureCodec.decode(materia.casting.CastingFeatureCodec.encode([feature]))[0];
      check(
        round.kind == kind
        && round.adjustable.length == materia.casting.CastingFeature.CastingDimensions.values().length,
        "Feature kinds and adjustable dimension tokens survive the source codec"
      );
    }
    for (opening in materia.tooling
      .casting.CastingStrategies.StrategyOpenings.values()) for (axis in materia.tooling.casting.CastingStrategies
      .WithdrawalAxes.values()) for (strategy in materia.tooling.casting.CastingSetup.CoreStrategies.values()) {
      var round = materia.tooling.casting.CastingStrategies.snapshotRules([{
        kind: "lightening",
        opening: (opening: String),
        strategy: (strategy: String),
        priority: 0,
        preferredAxis: (axis: String),
        useEPSAboveBoardThickness: false
      }])[0];
      check(
        round.opening == opening && round.preferredAxis == axis && round.strategy == strategy,
        "Every strategy, opening and withdrawal token survives its codec"
      );
    }
  }

  static function fixtureEnds():Void {
    var d = fixture(), insert = d.input.inserts[0], hole = insert.holes[0];
    hole.frame = F.identity();
    hole.extent = Blind;
    hole.depth = 12;
    hole.threadDepth = 10;
    hole.diameter = 5;
    hole.tapDrill = 5;
    hole.clearanceDiameter = 6.6;
    hole.thread = {
      designation: "M6",
      majorDiameter: 6,
      pitch: 1,
      starts: 1,
      hand: Right,
      internal: true,
      toleranceClass: "6H",
      profile: Metric
    };
    hole.limits.policyId = "fixture-end-test";
    hole.limits.threaded = {minEngagement: 7.8, minTipGap: 1};
    TestInsertHoles.beforeCasting(insert);
    var face = materia.tooling.casting.InsertFixturePlanner.mountingFace(insert, F.identity(), d.recipe.hardware);
    var normal = F.transformVector(face, 0, 0, 1);
    check(
      Math.abs(face.z - insert.pose.z) < 1e-8 && normal.z < -1 + 1e-8,
      "A source underside thread selects the actual negative end, without moving the steel or hole"
    );
    check(
      materia.tooling.casting.InsertFixturePlanner.physicalEnd(insert, F.identity(), face),
      "The selected fixture plane is a physical source end"
    );
    var moved = F.compose(F.translation(0, 0, 1), face);
    check(
      !materia.tooling.casting.InsertFixturePlanner.physicalEnd(insert, F.identity(), moved),
      "An interior plane cannot be credited as a source contact face"
    );
    var socket = materia.casting.InsertSpec.InsertBody.CrimpedSocket(
      materia.casting.CastingSolid.Cylinder(8, 20),
      {length: 25, width: 11, thickness: 2.2, wave: 1.4, blend: 0.5}
    );
    var ends = materia.casting.InsertSpec.InsertBodyCodec.planarEndFrames(socket);
    check(ends.length == 1 && ends[0].z == 45, "A crimped retention tail does not invent a planar back receiver");
  }

  static function releaseContract():Void {
    var source = fixture(), restored = ToolingCodec.decode(ToolingCodec.encode(source));
    check(
      restored.recipe.releaseSystem.sealer.coats == 2 && restored.recipe.releaseSystem.agent.coats == 5,
      "The complete source codec stores the declared release coat counts"
    );
    check(switch restored.recipe.releaseSystem.agent.drySeconds {
      case Unresolved(reason):
        reason.indexOf("product timing") >= 0;
      default:
        false;
    }, "Manufacturer timing remains an explicit unresolved source input after JSON roundtrip");
    var before = ToolingCodec.recipeFingerprint(source.recipe);
    restored.recipe.releaseSystem.agent.coats++;
    check(
      ToolingCodec.recipeFingerprint(restored.recipe) != before,
      "Release process changes enter the recipe fingerprint"
    );
    var raw:Dynamic = ToolingCodec.documentRecord(source);
    Reflect.deleteField(raw.recipe, "releaseSystem");
    Reflect.setField(raw.recipe, "releaseSpec", "legacy placeholder");
    rejects(() -> ToolingCodec.snapshot(raw), "Unknown recipe field: releaseSpec");
  }

  static function fixtureReceiverContract():Void {
    var source = fixture(), before = ToolingCodec.recipeFingerprint(source.recipe);
    for (choice in [
      materia.tooling.ToolingRecipe.FixtureReceiver.IndependentDatums,
      materia.tooling.ToolingRecipe.FixtureReceiver.PanelWhenCoplanar
    ]) {
      source.recipe.hardware.fixtureReceiver = choice;
      var restored = ToolingCodec.decode(ToolingCodec.encode(source));
      check(restored.recipe.hardware.fixtureReceiver == choice, "Fixture receiver choices survive the source boundary");
    }
    check(
      ToolingCodec.recipeFingerprint(source.recipe) != before,
      "Fixture receiver selection enters the recipe fingerprint"
    );
    var raw:Dynamic = ToolingCodec.documentRecord(source);
    raw.recipe.hardware.fixtureReceiver = "unknown";
    rejects(() -> ToolingCodec.snapshot(raw), "Unknown fixture receiver");
    Reflect.deleteField(raw.recipe.hardware, "fixtureReceiver");
    rejects(() -> ToolingCodec.snapshot(raw), "fixtureReceiver");
  }

  static function insertEnvelopeRecords():Void {
    var document = fixture(), source = document.input.inserts[0];
    source.body = materia.casting.InsertSpec.InsertBody.Primitive(materia.casting.CastingSolid.Cylinder(40, 20));
    var before = ToolingCodec.canonical(source);
    var region = materia.tooling.casting.MoldRegions.insert(document, source.id);
    check(
      materia.tooling.casting.MoldRegions.boxes(region) == null,
      "Round steel envelopes never become rectangular material"
    );
    switch region {
      case Placed(geometry):
        check(
          ToolingCodec.canonical(geometry.pose) == ToolingCodec.canonical(F.compose(
            document.setup.sourceToMold,
            source.pose
          )),
          "Insert envelope placement composes the source and mold frames"
        );
        check(
          ToolingCodec.canonical(geometry.solid)
          == ToolingCodec.canonical(materia.casting.CastingFeature.CastingGeometrySolid.Insert(source.body)),
          "Insert envelope retains the exact body variant"
        );
      default:
        throw "Insert envelope needs its placed source body";
    }
    check(before == ToolingCodec.canonical(source), "Insert envelope queries do not mutate source records");
    rejects(() -> materia.tooling.casting.MoldRegions.insert(document, "absent"), "Expected one source insert");
  }

  public static function main():Void {
    fixtureReceiverContract();
    insertEnvelopeRecords();
    vocabularyRoundTrips();
    releaseContract();
    if (Sys.getEnv("TOOLING_ONLY") == "anchors") {
      AnchorCapacityTests.run(fixture());
      return;
    }
    if (Sys.getEnv("TOOLING_ONLY") == "fixture-faces") {
      fixtureFaces();
      fixturePanelFaces();
      Sys.println('FixtureFaceTests: $assertions assertions passed');
      return;
    }
    routedSocketFaces();
    boardLimitedStrategy();
    epsFillPortal();
    fixtureFaces();
    fixturePanelFaces();
    if (Sys.getEnv("TOOLING_ONLY") == "castability") {
      var document = fixture();
      document.recipe.minimumLigament = 2; // Same mold fixture as CastingMoldTests.run.
      var plan = materia.tooling.casting.CastingMoldPlanner.plan(document, CastingMoldTests.options(document));
      CastabilityTests.run(plan, ProcessTests.spec());
      return;
    }
    withdrawalTie();
    fixtureEnds();
    relievedFixing();
    sharedFixturePlate();
    unresolvedStockFixing();
    nominalProduct();
    var original = fixture(), encoded = ToolingCodec.encode(original), decoded = ToolingCodec.decode(encoded);
    check(ToolingCodec.encode(decoded) == encoded, "canonical roundtrip");
    var detachedSource = ToolingCodec.encodeInput(original.input);
    check(
      ToolingCodec.encodeInput(ToolingCodec.decodeInput(detachedSource)) == detachedSource,
      "detached casting source uses the same strict input records"
    );
    var sourceRecord:Dynamic = Json.parse(detachedSource);
    sourceRecord.version = ToolingCodec.VERSION - 1;
    rejects(
      () -> ToolingCodec.decodeInput(Json.stringify(sourceRecord)),
      "Unsupported tooling source format or version"
    );
    var stored:Dynamic = Json.parse(encoded), storedInsert = items(stored.document.input.inserts)[0];
    check(
      storedInsert.body.kind == "primitive"
      && storedInsert.body.solid.kind == "box" && storedInsert.body.solid.width == 40 && !Reflect.hasField(
        storedInsert,
        "width"
      ) && !Reflect.hasField(
        storedInsert,
        "depth"
      ) && !Reflect.hasField(
        storedInsert,
        "height"
      ),
      "one neutral body encoding without duplicate insert dimensions"
    );
    storedInsert.width = 40;
    rejects(() -> ToolingCodec.decode(Json.stringify(stored)), "Unknown insert field: width");
    var stock = fixture();
    stock.input.inserts[0].material = materia.casting.InsertMaterial.InsertMaterials.specification(C45Ground);
    rejects(() -> ToolingCodec.copyInput(stock.input), "Insert manufacturing grade differs from source stock");
    switch stock.input.inserts[0].origin {
      case Fabricated(manufacturing):
        manufacturing.material = stock.input.inserts[0].material;
      case Catalogue(_):
        throw "Fabricated fixture expected";
    }
    check(
      ToolingCodec.inputFingerprint(stock.input) != ToolingCodec.inputFingerprint(original.input),
      "insert stock condition enters the source fingerprint"
    );
    var boreRules = decoded.input.inserts[0].holes[0].limits;
    check(
      boreRules.threaded == null && Reflect.fields(boreRules).indexOf("minEngagement") < 0,
      "plain bores carry no placeholder engagement or tip-gap rules"
    );
    check(decoded.input.features[0].id == "insert-seat", "features canonicalized by ID");
    check(original.input.features[0].id == "pocket", "canonicalization does not reorder caller arrays");
    check(
      decoded.input.inserts[0].holes[0].diameter == 8 && decoded.input.inserts[0].positionTolerance == 0.01,
      "insert manufacturing interfaces survive roundtrip"
    );
    var top = F.transformPoint(decoded.setup.sourceToMold, 0, 0, 100);
    check(Math.abs(top.z) < 1e-9, "setup maps rail-side source top to bed");
    check(
      decoded.input.features[1].geometry.pose.z == -1
      && materia.casting.CastingFeature.CastingGeometrySolids.height(decoded.input.features[1].geometry.solid) == 61,
      "design-cut extensions preserved, not misrepresented as clipped cavity sizes"
    );
    decoded.input.envelope[0].profile[0].x = -999;
    decoded.input.inserts[0].pose.x = -999;
    decoded.recipe.core.evidence.push("changed");
    check(
      original.input.envelope[0].profile[0].x == -100
      && original.input.inserts[0].pose.x == 80 && original.recipe.core.evidence.length == 0,
      "nested snapshots are detached"
    );
    var ordered = fixture(), fingerprint = ToolingCodec.documentFingerprint(ordered);
    ordered.input.features.reverse();
    check(
      ToolingCodec.documentFingerprint(ordered) == fingerprint,
      "named collection order does not change fingerprint"
    );
    var before = ToolingCodec.recipeFingerprint(ordered.recipe);
    var defaults = fixture();

    var rules = materia.tooling.casting.CastingStrategies.resolve(ToolingCodec.copy(defaults));
    check(
      rules.length == 1 && rules[0].strategy == SacrificialEPS,
      "recipe rules select lightening cores without feature-ID tables"
    );
    defaults.input.features.filter(feature -> feature.id == "pocket")[0].kind = ServiceOpening;
    defaults.setup.sourceFingerprint = ToolingCodec.inputFingerprint(defaults.input);
    rules = materia.tooling.casting.CastingStrategies.resolve(ToolingCodec.copy(defaults));
    check(
      rules[0].strategy == ReusableFormer && rules[0].withdrawal != null && rules[0].withdrawal.z == 1,
      "body opening and mold orientation derive a default former withdrawal"
    );
    ordered.recipe.panel.measuredThickness = 17.6;
    check(ToolingCodec.recipeFingerprint(ordered.recipe) != before, "measured stock changes recipe fingerprint");
    var size = materia.casting.CastingFeature.CastingGeometrySolids.box(
      ordered.input.features[1].geometry.solid,
      "pocket"
    );
    ordered.input.features[1].geometry.solid =
      materia.casting.CastingFeature.CastingGeometrySolid.Cut(materia.casting.CastingSolid.Box(
        size.width,
        size.depth,
        size.height + 1
      ));
    rejects(() -> ToolingCodec.encode(ordered), "Stale tooling setup");
    ordered.setup.sourceFingerprint = ToolingCodec.inputFingerprint(ordered.input);
    check(ToolingCodec.encode(ordered).length > 0, "explicitly rebind setup to updated source");
    var antipodal = fixture();
    antipodal.input.features[1].geometry.pose.qw = -1;
    antipodal.setup.sourceFingerprint = ToolingCodec.inputFingerprint(antipodal.input);
    check(ToolingCodec.encode(antipodal).length > 0, "antipodal quaternions describe the same insert pose");

    function corrupt(edit:Dynamic -> Void, expected:String):Void {
      var raw:Dynamic = Json.parse(encoded);
      edit(raw);
      rejects(() -> ToolingCodec.decode(Json.stringify(raw)), expected);
    }
    corrupt(r -> {r.version = 7;
    }, "Unsupported tooling format");
    corrupt(r -> {r.version = 1.5;
    }, "Expected tooling integer");
    corrupt(r -> {r.document.input.lengthUnit = "inch";
    }, "explicit mm");
    corrupt(r -> {r.document.input.revision = 0;
    }, "revision must be positive");
    corrupt(r -> {items(r.document.input.envelope)[0].height = "100";
    }, "finite tooling number");
    corrupt(r -> {items(r.document.input.envelope)[0].height = true;
    }, "finite tooling number");
    corrupt(r -> {items(r.document.input.envelope)[0].height = 0;
    }, "positive tooling dimension");
    corrupt(r -> {items(r.document.input.envelope)[0].pose.qw = 2;
    }, "frame");
    corrupt(r -> {items(items(r.document.input.envelope)[0].profile).reverse();
    }, "convex and CCW");
    corrupt(r -> {items(items(r.document.input.envelope)[0].profile)[1] = {x: 0, y: 0};
    }, "convex and CCW");
    corrupt(r -> {items(items(r.document.input.envelope)[0].profile)[1] =
      items(items(r.document.input.envelope)[0].profile)[0];
    }, "convex and CCW");
    corrupt(r -> {items(r.document.input.features).push(items(r.document.input.features)[0]);
    }, "Duplicate casting feature");
    corrupt(r -> {items(r.document.input.features)[0].owner = "missing";
    }, "Invalid insert reference");
    corrupt(r -> {items(r.document.input.features)[0].geometry.pose.x += 1;
    }, "does not match insert");
    corrupt(r -> {items(r.document.input.features)[0].geometry.pose.qx = Math.sin(0.00005);
    items(r.document.input.features)[0].geometry.pose.qw = Math.cos(0.00005);
    }, "does not match insert");
    corrupt(r -> {items(r.document.input.features)[1].kind = "hole";
    }, "Unknown casting feature kind");
    corrupt(r -> {items(r.document.input.features)[1].owner = 17;
    }, "Invalid casting text");
    corrupt(r -> {items(r.document.input.features).splice(0, 1);
    }, "Missing recess");
    corrupt(r -> {items(items(r.document.input.inserts)[0].holes)[0].diameter = 40;
    items(items(r.document.input.inserts)[0].holes)[0].clearanceDiameter = 40;
    }, "bore consumes");
    corrupt(r -> {items(r.document.input.inserts)[0].role = "unrecognized";
    }, "Invalid insert role");
    corrupt(r -> {items(r.document.input.inserts)[0].finish = "ground after casting";
    }, "Invalid insert finish");
    corrupt(r -> {r.document.input.envelope = new Array<Dynamic>();
    }, "needs an envelope");
    corrupt(r -> {r.document.input.envelope = [for (_ in 0...513) items(r.document.input.envelope)[0]];
    }, "Too many tooling records");
    corrupt(r -> {r.document.recipe.featureRules = new Array<Dynamic>();
    }, "Missing or duplicate recipe strategy rule");
    corrupt(r -> {Reflect.setField(r.document.setup, "features", new Array<Dynamic>());
    }, "Unknown setup field: features");
    corrupt(r -> {items(r.document.recipe.featureRules)[0].strategy = "guess";
    }, "Invalid casting rule strategy");
    corrupt(r -> {r.document.recipe.panel.thicknessVariation = 18;
    }, "variation consumes");
    corrupt(r -> {r.document.recipe.panel.coatedFace = "up";
    }, "Invalid coated face");
    corrupt(r -> {r.document.recipe.panel.allowedRotations = [0, 180];
    }, "Invalid or duplicate panel stock rotation");
    corrupt(r -> {r.document.recipe.panel.allowedRotations = [0, 0];
    }, "Invalid or duplicate panel stock rotation");
    corrupt(r -> {r.document.recipe.panel.directionConstraint = "width-aligned";
    }, "Invalid panel direction constraint");
    corrupt(r -> {r.document.recipe.core.compressionStrengthPa = 100000;
    }, "requires a strain fraction");
    corrupt(r -> {r.document.recipe.core.densityKgM3 = -1;
    }, "positive tooling dimension");
    corrupt(r -> {r.document.recipe.routerDiameter = 0;
    }, "positive tooling dimension");
    corrupt(r -> {r.document.recipe.processEvidence = ["trial", "trial"];
    }, "Duplicate evidence");
    corrupt(r -> {Reflect.deleteField(r.document.recipe, "linerThickness");
    }, "Missing recipe field");
    corrupt(r -> {r.document.recipe.linerThicknes = 1;
    }, "Unknown recipe field");
    rejects(() -> ToolingCodec.decode("[]"), "Expected tooling object");
    rejects(() -> ToolingCodec.decode("{"), "");
    rejects(() -> ToolingCodec.decode(StringTools.lpad("",
      "x", ToolingCodec.MAX_TEXT_BYTES + 1)), "exceeds size limit");
    rejects(
      () -> ToolingCodec.decode(StringTools.lpad("", "é", Std.int(ToolingCodec.MAX_TEXT_BYTES / 2) + 1)),
      "exceeds size limit"
    );
    var invalid = fixture();
    invalid.input.envelope[0].height = Math.NaN;
    rejects(() -> ToolingCodec.encode(invalid), "finite tooling number");
    invalid.input.envelope[0].height = Math.POSITIVE_INFINITY;
    rejects(() -> ToolingCodec.inputFingerprint(invalid.input), "finite tooling number");

    var review = ToolingReview.inspect(fixture());
    var pendingRules = [for (diagnostic in review) if (diagnostic.status == "pending") diagnostic.ruleId];
    for (rule in ["tooling.geometry", "tooling.process", "tooling.loads", "tooling.core-stock"]) check(
      pendingRules.indexOf(rule) >= 0,
      "unknown qualification remains pending: " + rule
    );
    for (insert in fixture().input.inserts) check(
      pendingRules.indexOf("tooling.insert.anchorage." + insert.id + ".source") >= 0,
      "undeclared source anchorage remains pending: " + insert.id
    );
    var unsupported = fixture();
    ToolingTestRules.first(unsupported, Unsupported, null);
    check(
      ToolingReview.inspect(unsupported).filter(d -> d.status == "unsupported").length == 1,
      "explicit unsupported strategy survives serialization and review"
    );
    var reusable = fixture();
    ToolingTestRules.first(reusable, ReusableFormer, {x: 0, y: 0, z: 1});
    var restoredDirection = materia
      .tooling.casting.CastingStrategies.resolve(ToolingCodec.decode(ToolingCodec.encode(reusable)))[0].withdrawal;
    check(restoredDirection != null && restoredDirection.z == 1, "explicit reusable former direction");
    reusable.recipe.core.compressionStrengthPa = 100000;
    reusable.recipe.core.compressionStrainFraction = 0.1;
    reusable.recipe.core.densityKgM3 = 25;
    reusable.recipe.core.maximumTemperatureC = 60;
    reusable.recipe.core.evidence = ["test-only supplier reference"];
    reusable.recipe.processEvidence = ["test-only trial reference"];
    check(
      ToolingReview.inspect(reusable).filter(d -> d.ruleId == "tooling.process" && d.status == "pending").length == 1,
      "evidence references alone never qualify process"
    );
    Sys.println('ToolingTests: $assertions assertions passed');
    ReleaseSystemTests.run();
    ReleasePlanTests.run();
    PanelTests.run();
    BoxPlannerTests.run(fixture());
    CastingMoldTests.run(fixture());
    MoldRegionMathTests.run();
    AnchorCapacityTests.run(fixture());
    FormerConstructionTests.run(fixture().recipe);
  }
}
