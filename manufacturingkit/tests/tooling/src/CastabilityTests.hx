import materia.casting.CastingProfiles as Profiles;
import materia.casting.CastingFeature.CastingGeometrySolid;
import materia.tooling.ToolingCodec;
import materia.tooling.casting.CastingMoldPlan;
import materia.tooling.casting.CastingMoldPlan.MoldBox;
import materia.tooling.process.CastingProcessSpec;
import materia.tooling.process.CastabilityAnalysis;
import materia.tooling.process.CastingProcessSpec.ProcessSpecs;
import materia.tooling.process.FillAnalysis;
import materia.tooling.process.OrthogonalSections;

class CastabilityTests {
  static var assertions = 0;

  static function check(ok:Bool, message:String):Void {
    assertions++;
    if (!ok) throw message;
  }

  static function rejects(fn:Void -> Void, expected:String):Void {
    var error = "";
    try fn() catch (e:Dynamic) error = Std.string(e);
    check(error.indexOf(expected) >= 0, 'Expected "$expected", got "$error"');
  }

  static function box(x:Float, z:Float, width:Float, depth:Float, height:Float):MoldBox return {
    x: x,
    y: 0,
    z: z,
    width: width,
    depth: depth,
    height: height
  };

  static function status(r:CastabilityReport, rule:String):String {
    var rows = r.validation.filter(v -> v.rule == rule);
    check(rows.length == 1, "one finding per rule: " + rule);
    return rows[0].status;
  }

  static function liftingLocations():Void {
    var d = ToolingTests.fixture();
    d.recipe.designReview.minimumLiftingPoints = 4;
    d.setup.sourceFingerprint = ToolingCodec.inputFingerprint(d.input);
    var report = materia.tooling.process.CastabilityAnalysis.sourceReview(d);
    var locations = report.proposals.filter(proposal -> proposal.action == DeclareLiftingPoint);
    check(locations.length == 4, "missing lifting connections get a complete CoM-based review pattern");
    var envelope = materia.tooling.casting.CastingMoldPlanner.envelope(d);
    var moments = materia.tooling.casting.CastHandling.moments(d), x = 0.0, y = 0.0;
    for (location in locations) {
      if (location.point == null) throw "Missing lifting candidate point";
      var point: {x: Float, y: Float, z: Float} = cast location.point;
      check(
        Math.isFinite(point.x + point.y + point.z) && location.instruction.indexOf("nominal casting mass") >= 0,
        "each candidate names its source position and nominal mass basis"
      );
      x += point.x;
      y += point.y;
    }
    check(
      Math.abs(x / locations.length - moments.centerOfMassMoldMm.x) < 1e-8
      && Math.abs(y / locations.length - moments.centerOfMassMoldMm.y) < 1e-8,
      "candidate pattern surrounds the density-weighted casting CoM"
    );
    var original = x / locations.length;
    d.recipe.steelDensityKgM3 *= 2;
    var shifted = materia.tooling
      .process.CastabilityAnalysis.sourceReview(d).proposals.filter(proposal -> proposal.action == DeclareLiftingPoint);
    var moved = 0.0;
    for (location in shifted) {
      if (location.point == null) throw "Missing shifted lifting candidate";
      var point: {x: Float, y: Float, z: Float} = cast location.point;
      moved += point.x;
    }
    check(
      Math.abs(moved / shifted.length - original) > 1e-3,
      "a changed steel mass shifts the lifting pattern, not just its label"
    );
    rejects(() -> materia.tooling.process.CastabilityAnalysis.apply(d,
      shifted[0]), "Unmapped or stale source proposal");
  }

  static function authoredProfileCorners():Void {
    var semicircle = [Profiles.arc(0, 0, 40, -Math.PI / 2, Math.PI), Profiles.line({x: 0, y: 40}, {x: 0, y: -40})];
    var cases = [
      {solid: Cut(materia.casting.CastingSolid.Box(100, 80, 61)), rounding: 6.0, measured: 6.0},
      {solid: Prism(Profiles.roundedRectangle(-50, -40, 100, 80, 6), 61), rounding: 0.0, measured: 6.0},
      {solid: Prism(Profiles.roundedRectangle(-50, -40, 100, 80, 2), 61), rounding: 0.0, measured: 2.0},
      {solid: Prism(Profiles.rectangle(-50, -40, 100, 80), 61), rounding: 0.0, measured: 0.0},
      {solid: Cut(materia.casting.CastingSolid.Cylinder(20, 61)), rounding: 0.0, measured: 10.0},
      {solid: Prism(semicircle, 61), rounding: 0.0, measured: 0.0}
    ];
    for (sample in cases) {
      var document = ToolingTests.fixture();
      document.recipe.designReview.minimumCornerRadiusMm = 4;
      var feature = document.input.features.filter(feature -> feature.id == "pocket")[0];
      feature.geometry.solid = sample.solid;
      feature.cornerRadius = sample.rounding;
      document.setup.sourceFingerprint = ToolingCodec.inputFingerprint(document.input);
      var report = materia.tooling.process.CastabilityAnalysis.sourceReview(document);
      var radius = report.metrics.minimumCornerRadiusMm;
      check(
        radius != null && Math.abs(radius - sample.measured) < 1e-7,
        "source screening measures authored line/arc corners rather than the optional implicit-filleting parameter"
      );
      var rows = report.validation.filter(row -> row.rule == "process.level1.corner-radius.pocket");
      check(
        rows.length == 1 && rows[0].status == (sample.measured >= 4 ? materia.tooling.ToolingDiagnostic.ToolingCheckStatus.Pass : materia.tooling.ToolingDiagnostic.ToolingCheckStatus.Fail),
        "the existing corner-radius rule tests actual curvature and detects sharp arc-to-line joints"
      );
    }
  }

  static function nonBoxReports():Void {
    var document = ToolingTests.fixture();
    document.recipe.designReview.minimumLiftingPoints = 4;
    document.recipe.designReview.minimumInsertEdgeMm = 0;
    document.recipe.designReview.minimumInsertCoverMm = 1;
    document.recipe.designReview.minimumFixingAccessMm = 1;
    var baseline = materia.tooling.process.CastabilityAnalysis.sourceReview(document);
    var feature = document.input.features.filter(feature -> feature.id == "pocket")[0];
    feature.cornerRadius = 4;
    document.setup.sourceFingerprint = ToolingCodec.inputFingerprint(document.input);
    var before = ToolingCodec.canonical(document);
    var spec = ProcessTests.spec();
    spec.recipeFingerprint = ToolingCodec.recipeFingerprint(document.recipe);
    spec.castability.maximumAggregateSizeMm = 4;
    spec.castability.minimumPassageAggregateRatio = 3;
    spec.compound.thickness = CompoundThicknessLimit.Section(30);
    var source = ToolingCodec.documentFingerprint(document), process = "partial-process";
    var fill = FillAnalysis.analyze(
      document.input.id,
      source,
      process,
      [box(0, 0, 2, 2, 10), box(2, 0, 2, 2, 2), box(4, 0, 4, 2, 6)],
      [{id: "pocket", box: box(4, 6, 4, 2, 4)}],
      [],
      10,
      2400,
      10,
      null,
      100
    );
    var report = CastabilityAnalysis.analyze(document, spec, fill, null, source, process);
    check(
      report.minimumOpposingFaceSpanMm == null && report.maximumInscribedThicknessMm == null
      && report.maximumDiameterLowerMm == null && report.maximumDiameterUpperMm == null,
      "missing section geometry supplies no invented scalar measurements"
    );
    for (rule in ["process.castability.minimum-passage"]) {
      check(status(report, rule) == "unsupported", "a declared limit does not qualify missing geometry: " + rule);
      var row = report.validation.filter(row -> row.rule == rule)[0];
      check(
        row.affectedIds.length == 1 && row.affectedIds[0] == "pocket",
        "unsupported measurements name the source feature rather than a body bound"
      );
    }
    check(
      status(report, "process.level1.lifting-points") == "fail",
      "a non-box cavity cannot hide missing typed lifting connections"
    );
    for (name in ["insert-edge", "insert-cover", "fixing-access"]) check(
      status(report, "process.level1." + name + ".seat") == "pass",
      "a curved pocket does not suppress independent exact insert checks: " + name
    );
    check(
      report.designMetrics.minimumInsertEdgeMm == baseline.metrics.minimumInsertEdgeMm
      && report.designMetrics.minimumEmbeddedDepthMm == baseline.metrics.minimumEmbeddedDepthMm
      && report.designMetrics.minimumFixingAccessLandMm == baseline.metrics.minimumFixingAccessLandMm,
      "independent box-stock measurements are unchanged by an unrelated curved cavity"
    );
    var vent = report.proposals.filter(proposal -> proposal.action == RouteVent);
    check(vent.length == 1 && vent[0].point != null, "partial fill keeps its vent position");
    var ventPoint: {x: Float, y: Float, z: Float} = cast vent[0].point;
    check(
      vent[0].sourceIds[0] == "pocket" && ventPoint.z == 6,
      "exact fill evidence retains its owned vent proposal without orthogonal sections"
    );
    check(
      report.validation.filter(row -> row.rule == vent[0].rule && row.status == "fail").length == 1,
      "retaining a vent suggestion never clears the actual trapped-air failure"
    );
    var sourceOnly = CastabilityAnalysis.analyze(document, spec, null, null, source, process);
    check(
      sourceOnly.sourceFingerprint == source && sourceOnly.processFingerprint == process
      && sourceOnly.proposals.filter(proposal -> proposal.action == RouteVent).length == 0,
      "an unavailable fill report keeps source checks without inventing pockets or vent locations"
    );
    check(
      ToolingCodec.canonical(document) == before,
      "partial geometry screening leaves authored source facts unchanged"
    );
    document.input.features.reverse();
    document.recipe.featureRules.reverse();
    check(
      ToolingCodec.canonical(CastabilityAnalysis.analyze(document,
      spec, fill, null, source, process)) == ToolingCodec.canonical(report),
      "partial reports and proposal IDs do not depend on feature order"
    );
    rejects(
      () -> CastabilityAnalysis.analyze(document, spec, fill, null, source, "stale"),
      "Mismatched castability fill process"
    );
    var curvedBody = ToolingCodec.copy(document);
    curvedBody.input.envelope = materia.casting.CastingEnvelope.build(200, 180, 100, 10);
    curvedBody.setup.sourceFingerprint = ToolingCodec.inputFingerprint(curvedBody.input);
    var sourceChecks = materia.tooling.process.CastabilityAnalysis.sourceReview(curvedBody);
    for (name in ["insert-edge", "insert-cover"]) check(
      sourceChecks.validation.filter(row -> row.rule == "process.level1." + name + ".seat"
      && row.status == materia.tooling.ToolingDiagnostic.ToolingCheckStatus.Unsupported).length == 1,
      "a chamfered body does not receive a bounding-box insert measurement: " + name
    );
    check(
      sourceChecks.metrics.minimumInsertEdgeMm == null && sourceChecks.metrics.minimumEmbeddedDepthMm == null
      && sourceChecks.metrics.minimumFixingAccessLandMm == baseline.metrics.minimumFixingAccessLandMm,
      "unsupported body measurements stay null while the actual insert's fixing land stays measured"
    );
    check(
      sourceChecks.validation.filter(row -> row.rule == "process.level1.lifting-points"
      && row.status == materia.tooling.ToolingDiagnostic.ToolingCheckStatus.Fail).length == 1,
      "non-rectangular bodies retain their typed lifting-point rule"
    );
    fill.sourceFingerprint = "stale";
    rejects(
      () -> CastabilityAnalysis.analyze(document, spec, fill, null, source, process),
      "Mismatched castability section/fill source"
    );
  }

  public static function run(p:CastingMoldPlan, raw:CastingProcessSpec):Void {
    liftingLocations();
    authoredProfileCorners();
    nonBoxReports();
    var d = ToolingCodec.copy(p.document), s = ProcessSpecs.copy(raw);
    s.recipeFingerprint = ToolingCodec.recipeFingerprint(d.recipe);
    s.castability = {
      maximumAggregateSizeMm: 4,
      minimumPassageAggregateRatio: 3,
      minimumWallAggregateRatio: 5,
      steelAggregateMarginMm: 5,
      steelSpacingAggregateRatio: 4.0 / 3.0,
      evidence: ["fixture-assumption-not-qualified"]
    };
    s.compound.thickness = CompoundThicknessLimit.Section(30);
    s.maximumAirEscapeWidthMm = 100;
    var source = ToolingCodec.documentFingerprint(d), owner = d.input.features.filter(feature -> feature.kind
      != materia.casting.CastingFeature.CastingFeatureKind.InsertRecess)[0].id;
    function analyze(region:Array<MoldBox>, ?boundaries:Array<FillBoundary>):CastabilityReport {
      s.recipeFingerprint = ToolingCodec.recipeFingerprint(d.recipe);
      if (boundaries == null) boundaries = [];
      var fillHeight = 0.0;
      for (b in region) fillHeight = Math.max(fillHeight, b.z + b.height);
      var fill = FillAnalysis.analyze(
        d.input.id,
        source,
        "process",
        region,
        boundaries,
        [],
        fillHeight,
        2400,
        10,
        null,
        s.maximumAirEscapeWidthMm
      );
      var sections = OrthogonalSections.analyze(d.input.id, source, region, boundaries);
      return CastabilityAnalysis.analyze(d, s, fill, sections, source, "process", p.validation);
    }
    var slender = analyze([box(0, 0, 100, 60, 20)], [{id: owner, box: box(0, 20, 100, 60, 10)}]);
    check(status(slender, "process.castability.minimum-passage") == "pass", "sufficient passage passes");
    check(slender.maximumInscribedThicknessMm == 20, "rectangular thickness uses shortest dimension");
    var thin = analyze([box(0, 0, 100, 60, 8)], [{id: owner, box: box(0, 8, 100, 60, 10)}]);
    check(status(thin, "process.castability.minimum-passage") == "fail", "aggregate-based narrow passage fails");
    var widen = thin.proposals.filter(v -> v.action == WidenPassage && v.rule == "process.castability.minimum-passage");
    check(widen.length == 1 && widen[0].sourceIds.indexOf(owner) >= 0, "narrow passage reports source feature");
    check(
      widen[0].measuredMm == 8 && widen[0].targetMm == 12 && widen[0].axis == Z,
      "proposal carries exact dimension and target"
    );
    var proposalRegion:MoldBox = cast widen[0].region;
    check(
      proposalRegion.height == 8 && widen[0].owner == DesignReview,
      "source proposal has mold-space location and owner"
    );
    var atLimit = analyze([box(0, 0, 100, 60, 12)], [{id: owner, box: box(0, 12, 100, 60, 10)}]);
    check(status(atLimit, "process.castability.minimum-passage") == "pass", "equality at aggregate limit passes");
    var thick = analyze([box(0, 0, 100, 60, 40)]);
    check(
      thick.maximumDiameterLowerMm != null && (cast thick.maximumDiameterLowerMm:Float) > 30,
      "thick block exposes the datasheet section exceedance"
    );
    check(
      thick.proposals.filter(v -> v.action == ReduceSection && v.measuredMm == 40 && v.targetMm == 30).length == 1,
      "thick section has concrete target without modifying source"
    );
    var region = [box(0, 0, 2, 2, 10), box(2, 0, 2, 2, 2), box(4, 0, 4, 2, 6)];
    var boundaries = [{id: owner, box: box(4, 6, 4, 2, 4)}];
    var trapped = analyze(region, boundaries), vents = trapped.proposals.filter(v -> v.action == RouteVent);
    check(vents.length == 1 && vents[0].sourceIds.indexOf(owner) >= 0, "trapped air preserves source owner");
    var point: {x: Float, y: Float, z: Float} = cast vents[0].point;
    check(point.z == 6 && vents[0].owner == ToolingReview, "vent is a tooling-routing candidate at exact ceiling");
    check(
      trapped.validation.filter(v -> v.rule == vents[0].rule && v.status == "fail").length == 1,
      "vent suggestion never clears trap"
    );
    check(
      trapped.maximumDiameterUpperMm != null && (cast trapped.maximumDiameterUpperMm:Float) <= 30,
      "certified junction upper bound stays below the datasheet limit"
    );
    check(
      trapped.proposals.filter(v -> v.action == AnalyzeJunction).length == 0,
      "resolved geometry does not request an unsupported thickness algorithm"
    );
    var before = ToolingCodec.canonical(trapped);
    region.reverse();
    check(
      ToolingCodec.canonical(analyze(region, boundaries)) == before,
      "geometry order does not change findings or proposal IDs"
    );
    var vented = analyze(region.concat([box(6, 6, 2, 2, 4)]), boundaries);
    check(vented.proposals.filter(v -> v.action == RouteVent).length == 0, "actual vent clears topology suggestion");
    s.maximumAirEscapeWidthMm = 1;
    var broad = analyze(region, boundaries);
    check(status(broad, "process.fill.air-escape-width") == "fail", "broad ceiling uses real fill rule");
    check(
      broad.proposals.filter(v -> v.action == ReviewCeiling && v.sourceIds.indexOf(owner) >= 0).length == 1,
      "broad ceiling suggestion retains feature IDs"
    );
    var cross:Array<MoldBox> = [
      {x: -5, y: -1, z: -4, width: 10, depth: 2, height: 8},
      {x: -1, y: -5, z: -4, width: 2, depth: 10, height: 8}
    ];
    var crossSections = OrthogonalSections.analyze(d.input.id, source, cross, []);
    crossSections.junction = materia.tooling.process.JunctionThickness.analyze(
      d.input.id,
      source,
      cross,
      [],
      {diameterToleranceMm: 1e-8, maximumCells: 20}
    );
    var crossFill = FillAnalysis.analyze(d.input.id, source, "process", cross, [], [], 4, 2400, 10, null, null);
    var interval = crossSections.junction;
    var fabrication = FabricationTests.spec();
    s.compound.reference = materia.casting.CastingInput.CastingTextInput.Known(fabrication.resinReference);
    s.compound.maximumMassPerLiftKg = materia.casting.CastingInput.Known(1);
    s.mixComposition = {
      reference: "interval fixture",
      compoundReference: fabrication.resinReference,
      ingredients: [{
        id: "resin",
        kind: materia.tooling.process.PourPlanning.MixIngredientKind.Resin,
        materialReference: "fixture",
        massFraction: 1
      }]
    };
    var timed = FillAnalysis.analyze(d.input.id, source, "process", cross, [], [], 4, 2400, 10, 1e9, null);
    var pour = materia.tooling.process.PourPlanning.analyze(
      d.input.id,
      timed,
      s.mixComposition,
      {
        reference: "fixture",
        capacityKg: 1,
        mixingSeconds: 1,
        transferSeconds: 1,
        workingTimeSeconds: 10000,
        workingTimeReference: "fixture"
      }
    );
    function compoundStatus():String return materia.tooling.process.CastingProcessAnalysis.compoundCheck(
      d.input.id,
      s,
      crossSections,
      pour,
      fabrication
    ).status;
    s.compound.thickness = CompoundThicknessLimit.Section((interval.lowerDiameterMm + interval.upperDiameterMm) / 2);
    check(compoundStatus() == "pending", "straddling section interval never accepts its midpoint");
    s.compound.thickness = CompoundThicknessLimit.Section(interval.upperDiameterMm + 1);
    check(compoundStatus() == "pass", "safe upper bound resolves the datasheet limit without convergence");
    s.compound.thickness = CompoundThicknessLimit.Section(interval.lowerDiameterMm - 1);
    check(compoundStatus() == "fail", "unsafe lower bound rejects without convergence");
    s.castability.maximumAggregateSizeMm = null;
    s.compound.thickness = CompoundThicknessLimit.Unresolved("Compound section limit");
    s.maximumAirEscapeWidthMm = null;
    var unknown = analyze([box(0, 0, 100, 60, 20)]);
    check(status(unknown, "process.castability.minimum-passage") == "pending", "unknown aggregate never passes");
    check(compoundStatus() == "pending", "unknown datasheet section limit never passes");
    check(status(unknown, "process.fill.air-escape-width") == "pending", "unknown ceiling rule remains pending");
    for (strategy in materia.tooling.casting.CastingStrategies.resolve(d)) {
      if (strategy.strategy == materia.tooling.casting.CastingSetup.CoreStrategy.SacrificialEPS) check(
        unknown.proposals.filter(v -> v.action == QualifyBreakout
        && v.sourceIds.indexOf(strategy.featureId) >= 0).length == 1,
        "each EPS source has a physical breakout action"
      );
      else if (strategy.strategy == materia.tooling.casting.CastingSetup.CoreStrategy.ReusableFormer) {
        var actual = p.validation.filter(v -> v.rule == "tooling.former.draft." + strategy.featureId);
        for (v in actual) check(
          unknown.validation.filter(row -> ToolingCodec.canonical(v) == ToolingCodec.canonical(row)).length == 1,
          "rigid release reuses exact planner validation"
        );
      }
    }
    var unchanged = ToolingCodec.documentFingerprint(p.document);
    d.recipe.featureRules.reverse();
    d.input.features.reverse();
    check(
      ToolingCodec.canonical(analyze([box(0, 0, 100, 60, 20)])) == ToolingCodec.canonical(unknown),
      "source/setup member order does not change screening or proposal IDs"
    );
    var unsupportedFeature = materia.tooling.casting.CastingStrategies.resolve(d)[0].featureId;
    ToolingTestRules.first(d, materia.tooling.casting.CastingSetup.CoreStrategy.Unsupported, null);
    var unsupported = analyze([box(0, 0, 100, 60, 20)]);
    check(
      status(unsupported, "process.castability.release." + unsupportedFeature) == "unsupported",
      "unsupported strategy remains an explicit blocker"
    );
    check(
      unsupported.proposals.filter(v -> v.action == SplitOrSacrificial
      && v.sourceIds.indexOf(unsupportedFeature) >= 0).length == 1,
      "unsupported strategy gets a tooling action with its source ID"
    );
    d = ToolingCodec.copy(p.document);
    s.recipeFingerprint = ToolingCodec.recipeFingerprint(d.recipe);
    check(
      ToolingCodec.documentFingerprint(p.document) == unchanged,
      "screening and suggestions preserve the original source/setup"
    );
    for (proposal in unknown.proposals) check(
      unknown.validation.filter(v -> v.rule == proposal.rule).length == 1,
      "proposal references real finding"
    );
    var fill = FillAnalysis.analyze(
      d.input.id,
      source,
      "process",
      [box(0, 0, 100, 60, 20)],
      [],
      [],
      20,
      2400,
      10,
      null,
      null
    );
    var sections = OrthogonalSections.analyze(d.input.id, "stale", [box(0, 0, 100, 60, 20)], []);
    rejects(() -> CastabilityAnalysis.analyze(d, s, fill, sections, source, "process"), "Mismatched castability");
    sections.sourceFingerprint = source;
    rejects(
      () -> CastabilityAnalysis.analyze(d, s, fill, sections, source, "stale-process"),
      "Mismatched castability fill process"
    );
    var rawSpec:Dynamic = haxe.Json.parse(ToolingCodec.canonical(s));
    rawSpec.castability.maximumAggregateSizeMm = -1;
    rejects(() -> ProcessSpecs.fromRecord(rawSpec), "Invalid casting process number");
    rawSpec = haxe.Json.parse(ToolingCodec.canonical(s));
    Reflect.deleteField(rawSpec.castability, "minimumPassageAggregateRatio");
    rejects(() -> ProcessSpecs.fromRecord(rawSpec), "Unexpected casting record fields");
    s.recipeFingerprint = haxe.crypto.Sha256.encode("stale");
    rejects(
      () -> CastabilityAnalysis.analyze(d, s, fill, sections, source, "process", p.validation),
      "Stale castability recipe"
    );
    Sys.println('CastabilityTests: $assertions assertions passed');
  }
}
