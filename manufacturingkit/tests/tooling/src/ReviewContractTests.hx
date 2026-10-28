import materia.casting.InsertSpec.InsertBodyCodec as InsertBodies;
import materia.tooling.ToolingCodec;
import materia.tooling.ToolingValidation;
import materia.tooling.casting.CastingMoldPlan;
import materia.tooling.casting.CastingMoldPlanner;
import materia.tooling.casting.CastingMoldPlan.MoldOptions;
import materia.tooling.export.ToolingFabrication;
import materia.tooling.export.ToolingLoads;
import materia.tooling.process.CastabilityAnalysis;
import materia.tooling.process.CastingProcessAnalysis;
import materia.tooling.process.CastingProcessSpec;
import materia.tooling.process.FillAnalysis;
import materia.tooling.process.JunctionThickness;
import materia.tooling.process.OrthogonalSections;

class ReviewContractTests {
  static var assertions = 0;

  static function check(ok:Bool, message:String):Void {
    assertions++;
    if (!ok) throw message;
  }

  static function rejects(fn:Void -> Void, expected:String):Void {
    var message = "";
    try fn() catch (error:Dynamic) message = Std.string(error);
    check(message.indexOf(expected) >= 0, 'Expected "$expected", got "$message"');
  }

  public static function run(p:CastingMoldPlan):Void {
    var o = MoldOptions.copy(p.options);
    var source = ToolingCodec.copy(p.document);
    source.input.inserts[0].finish = materia.casting.InsertSpec.InsertFinish.ReplicatedAfterCasting;
    source.setup.sourceFingerprint = ToolingCodec.inputFingerprint(source.input);
    o.precision[0].masterReference = null;
    check(
      CastingMoldPlanner.plan(source, o).validation.filter(v -> v.rule
      == "tooling.precision.replication." + o.precision[0].insertId && v.status == "pending").length == 1,
      "missing replication master has its explicit validation rule"
    );
    var missing = CastingMoldPlanner.plan(source, o);
    missing.validation = missing.validation.filter(v -> v.rule
      != "tooling.precision.replication." + o.precision[0].insertId);
    missing.fabrication = ToolingReadiness.evaluate(missing.validation);
    rejects(
      () -> CastingMoldPlanner.validate(missing),
      "Replication requirement differs from source interfaces and process choices"
    );
    var oldOptions:Dynamic = haxe.Json.parse(ToolingCodec.canonical(o));
    var oldPrecision:Array<Dynamic> = cast Reflect.field(oldOptions, "precision");
    Reflect.setField(oldPrecision[0], "allowance", null);
    rejects(() -> MoldOptions.read(oldOptions), "Invalid mold process record");
    Reflect.deleteField(oldPrecision[0], "allowance");
    Reflect.setField(oldPrecision[0], "route", "replicated");
    rejects(() -> MoldOptions.read(oldOptions), "Invalid mold process record");
    Reflect.deleteField(oldPrecision[0], "route");
    Reflect.deleteField(oldPrecision[0], "jigReference");
    rejects(() -> MoldOptions.read(oldOptions), "Invalid mold process record");
    o.precision[0].jigReference = "synthetic-jig";
    check(
      MoldOptions.copy(o).precision[0].jigReference == "synthetic-jig",
      "jig reference is an explicit process choice"
    );
    o.precision[0].masterReference = "synthetic-master";
    o.precision[0].releaseSpec = "synthetic-release";
    check(
      CastingMoldPlanner.plan(source,
      o).validation.filter(v -> v.rule == "tooling.precision.replication." + o.precision[0].insertId).length == 0,
      "identified master and release clear only the replication prerequisites"
    );
    source.input.inserts[0].finish = materia.casting.InsertSpec.InsertFinish.AsSupplied;
    source.setup.sourceFingerprint = ToolingCodec.inputFingerprint(source.input);
    o.precision[0].masterReference = null;
    o.precision[0].releaseSpec = null;
    check(
      CastingMoldPlanner.plan(source,
      o).validation.filter(v -> v.rule == "tooling.precision.replication." + o.precision[0].insertId).length == 0,
      "process options cannot introduce replication into an as-supplied source"
    );
    var sourceInsert = source.input.inserts[0];
    sourceInsert.interfaces = [{
      id: "replicated-plane",
      geometry: materia
      .casting.InterfaceRequirement.PrecisionGeometry.Plane(materia.assembly.AssemblyFrames.translation(
        0,
        0,
        21
      )),
      route: materia.casting.InterfaceRequirement.InterfaceRoute.Replicated,
      allowanceMm: 1,
      finishingProcessErrorMm: materia.casting.CastingInput.Unresolved("Finishing process error allocation"),
      permittedFinishes: [materia.casting.InterfaceRequirement.InterfaceFinish.JigMaster],
      positionToleranceMm: 0.01,
      datumReference: "primary",
      relationshipIds: []
    }];
    check(
      materia.tooling.casting.InterfaceQualification.replicationRequirement(sourceInsert, o.precision[0]) != null,
      "explicit source interfaces require replication independently of the aggregate finish label"
    );
    var raw = ToolingCodec.encode(p.document);
    check(
      raw.indexOf('"label"') >= 0 && raw.indexOf('"sourceRole"') < 0,
      "source labels are separate from required typed roles"
    );
    var changed = ToolingCodec.copy(p.document);
    changed.recipe.designReview.minimumInsertEdgeMm = 5;
    var constrained = CastabilityAnalysis.sourceReview(changed).proposals.filter(v -> v.rule
      == "process.level1.insert-edge.seat");
    check(
      constrained.length == 1 && constrained[0].proposedValue == null,
      "width proposals preserve explicitly declared body openings"
    );
    // An independently authored inset seat permits width changes without changing openings.
    changed.input.inserts[0].pose.x = 78;
    var recess = changed.input.features.filter(feature -> feature.owner == "seat")[0];
    recess.geometry.pose.x = 78;
    recess.openings = recess.openings.filter(opening -> opening.target != "body.x-max");
    changed.setup.sourceFingerprint = ToolingCodec.inputFingerprint(changed.input);
    var review = CastabilityAnalysis.sourceReview(changed);
    var proposals = review.proposals.filter(v -> v.rule == "process.level1.insert-edge.seat");
    check(
      proposals.length == 1
      && proposals[0].sourceParameterId == "input.inserts.seat.body.solid.width" && proposals[0].proposedValue == 34,
      "source-only edge review supplies a mapped width/value proposal"
    );
    check(
      review.metrics.minimumInsertEdgeMm == 2 && review.metrics.minimumCornerRadiusMm == 0,
      "level-1 metrics retain actual source geometry"
    );
    var accepted = CastabilityAnalysis.apply(changed, proposals[0]);
    var replanned = CastingMoldPlanner.plan(accepted, p.options);
    check(
      InsertBodies.box(replanned.document.input.inserts[0].body,
      "seat").width == 34 && replanned.sourceFingerprint != p.sourceFingerprint,
      "accepted source proposal updates linked recess and successfully replans"
    );
    check(
      CastabilityAnalysis.sourceReview(accepted).validation.filter(v -> v.rule == "process.level1.insert-edge.seat"
      && v.status == "pass").length == 1,
      "accepted width change clears the actual source edge rule"
    );
    rejects(() -> CastabilityAnalysis.apply(accepted, proposals[0]), "stale source proposal");
    var limits = ToolingCodec.copy(changed);
    limits.recipe.designReview.minimumCornerRadiusMm = 2;
    limits.recipe.designReview.minimumInsertCoverMm = 20;
    limits.recipe.designReview.minimumFixingAccessMm = 20;
    limits.recipe.designReview.minimumLiftingPoints = 1;
    limits.recipe.designReview.removalClearanceMm = 0;
    ToolingTestRules.first(
      limits,
      materia.tooling.casting.CastingSetup.CoreStrategy.ReusableFormer,
      {x: 0, y: 0, z: -1}
    );
    var levelOne = CastabilityAnalysis.sourceReview(limits);
    function sourceFinding(
      report:materia.tooling.process.CastabilityAnalysis.CastingDesignReport,
      rule:String,
      id:String,
      status:String
    ):Void {
      var rows = report.validation.filter(row -> row.rule == "process.level1." + rule);
      check(
        rows.length == 1 && rows[0].status == status && rows[0].affectedIds.length == 1 && rows[0].affectedIds[0] == id,
        "source-only " + rule + " reports " + status + " with its actual source ID"
      );
    }
    for (row in [
      {rule: "corner-radius.pocket", id: "pocket"},
      {rule: "insert-cover.seat", id: "seat"},
      {rule: "insert-edge.seat", id: "seat"},
      {rule: "fixing-access.seat", id: "seat"},
      {rule: "lifting-points", id: limits.input.id}
    ]) sourceFinding(
      levelOne,
      row.rule,
      row.id,
      "fail"
    );
    sourceFinding(levelOne, "removal-direction.pocket", "pocket", "pass");
    var passing = ToolingCodec.copy(accepted);
    passing.recipe.designReview.minimumInsertCoverMm = 9;
    passing.recipe.designReview.minimumFixingAccessMm = 10;
    passing.recipe.designReview.removalClearanceMm = 0;
    ToolingTestRules.first(
      passing,
      materia.tooling.casting.CastingSetup.CoreStrategy.ReusableFormer,
      {x: 0, y: 0, z: 1}
    );
    var positive = CastabilityAnalysis.sourceReview(passing);
    for (row in [
      {rule: "insert-cover.seat", id: "seat"},
      {rule: "insert-edge.seat", id: "seat"},
      {rule: "fixing-access.seat", id: "seat"},
      {rule: "removal-direction.pocket", id: "pocket"}
    ]) sourceFinding(
      positive,
      row.rule,
      row.id,
      "pass"
    );
    var rounded = ToolingCodec.copy(passing);
    rounded.input.features.filter(feature -> feature.id == "pocket")[0].cornerRadius = 3;
    rounded.recipe.designReview.minimumCornerRadiusMm = 2;
    rounded.setup.sourceFingerprint = ToolingCodec.inputFingerprint(rounded.input);
    sourceFinding(CastabilityAnalysis.sourceReview(rounded), "corner-radius.pocket", "pocket", "pass");
    var s = FabricationTests.spec();
    s.plates = [{
      panelId: p.panels.members[0].panel.id,
      spanXmm: 100,
      spanYmm: 100,
      youngPa: 3e9,
      poisson: 0.3,
      allowableStressPa: 1e7,
      maximumDeflectionMm: 1,
      evidence: ["test"]
    }];
    rejects(() -> ToolingLoads.report(p, s), "without structural model");
    var invalid = FillAnalysis.check(
      "test.rule",
      [p.document.input.id],
      materia.tooling.ToolingDiagnostic.ToolingCheckStatus.Fail,
      "test",
      []
    );
    invalid.severity = cast "unknown";
    rejects(() -> ToolingReadiness.evaluate([invalid]), "severity/status consistency");
    invalid.severity = materia.tooling.ToolingDiagnostic.ToolingSeverity.Info;
    rejects(() -> ToolingReadiness.evaluate([invalid]), "severity/status consistency");
    var lower = ToolingCodec.copy(p.document);
    lower.setup.fillHeightMm = p.envelope.z + p.envelope.height / 2;
    check(
      CastingMoldPlanner.resinAtFill(lower).filter(b -> b.z + b.height > lower.setup.fillHeightMm + 1e-7).length == 0,
      "neutral datum/resin mask excludes material above fill height"
    );
    var bridges = CastingMoldPlanner.plan(p.document, p.options);
    var wider = CastingMoldPlanner.plan(p.document, p.options).cores[0];
    wider.id = "wide-core";
    var cavity = materia.tooling.casting.MoldCores.singleBox(wider.cavity, wider.id);
    var cy = cavity.y + cavity.depth / 2;
    cavity.depth = 200;
    cavity.y = cy - 100;
    // Rebuild physical foam too: a larger cavity alone supplies no wider bond pad.
    wider = materia.tooling.casting.MoldCoreLayers.box(
      cavity,
      "wide-core",
      p.document.recipe,
      materia.tooling.casting.CoreSupport.CoreSupportKind.Bridge
    );
    bridges.cores.push(wider);
    materia.tooling.casting.CoreSupports.sharedBridges(bridges.cavity, bridges.document.recipe, bridges.cores, [], []);
    var foam = materia.tooling.casting.MoldCores.singleBox(
      materia.tooling.casting.MoldCores.layerRegion(bridges.cores[0].layers[0]),
      bridges.cores[0].id
    );
    check(
      Math.abs(bridges.cores[0].support.bridgeBondAreaMm2 - foam.width * foam.depth) < 1e-7,
      "a wider row beam never credits bond area beyond the smaller foam pad"
    );
    var rigid = ToolingCodec.copy(p.document);
    for (strategy in materia.tooling.casting.CastingStrategies.resolve(rigid)) if (strategy.strategy
      == materia.tooling.casting.CastingSetup.CoreStrategy.SacrificialEPS) {
      ToolingTestRules.set(
        rigid,
        strategy.featureId,
        materia.tooling.casting.CastingSetup.CoreStrategy.ReusableFormer,
        0,
        {x: 0, y: 0, z: 1}
      );
    }
    var noCore = CastingMoldPlanner.plan(rigid, p.options);
    s = FabricationTests.spec();
    s.solvent = StyreneBearing;
    s.compatibilityEvidence = ["test resin"];
    check(
      noCore.cores.length == 0 && ToolingFabrication.plan(noCore,
      s).validation.filter(v -> v.rule == "fabrication.materials.compatibility" && v.status == "fail").length == 1,
      "unknown adhesive plus styrene fails without cores"
    );
    Sys.println('ReviewContractTests: $assertions assertions passed');
  }
}
