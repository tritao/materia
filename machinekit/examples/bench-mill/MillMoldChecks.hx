import materia.tooling.casting.CastingMoldPlan.MoldFixtures;
import materia.tooling.casting.MoldCores;
import materia.casting.BoxAlgebra;
import machinekit.tooling.CastingMoldCad;
import machinekit.tooling.ToolingVolume;
import machinekit.tooling.SweptRemoval;
import machinekit.assembly.PosedParts;
import cadkit.modeling.Vector;
import materia.tooling.casting.CastingMoldPlan;
import materia.tooling.casting.CastingMoldPlanner;
import materia.tooling.process.OrthogonalSections.SectionAxis;

/** Real mill acceptance cases; source CAD is an independent reference to the planner. */
class MillMoldChecks {
  static var assertions = 0;

  static function check(ok:Bool, label:String):Void {
    assertions++;
    if (!ok) throw label;
  }

  static function rejects(fn:Void -> Void, label:String):Void {
    var failed = false;
    try fn() catch (e:Dynamic) {
      failed = true;
      Sys.println("Expected mold rejection: " + Std.string(e));
    }
    check(failed, label);
  }

  static function equivalent(frame:BenchMillFrame, p:CastingMoldPlan):Void {
    var local = frame.base.body.geometry(), original = PosedParts.moved(local, p.document.setup.sourceToMold);
    local.close();
    var rebuilt = CastingMoldCad.reconstructed(p),
      a = original.subtractAll([rebuilt]), b = rebuilt.subtractAll([original]);
    check(a.volume() + b.volume() < 1e-3, "actual foundation CAD symmetric difference");
    a.close();
    b.close();
    rebuilt.close();
    original.close();
  }

  static function junctionGeometry(frame:BenchMillFrame, p:CastingMoldPlan):Void {
    var support = materia.tooling.casting.CastingBoxSupport.check(p.document.input, "process.junction.box-geometry");
    if (support.filter(item -> item.status
      == materia.tooling.ToolingDiagnostic.ToolingCheckStatus.Unsupported).length > 0) {
      for (insert in p.document.input.inserts) switch insert.anchorage {
        case Bolted(_, _):
          check(
            support.filter(item -> item.affectedIds.indexOf(insert.id) >= 0
            && item.status == materia.tooling.ToolingDiagnostic.ToolingCheckStatus.Unsupported).length == 1,
            "box-only junction check reports the exact anchor owner: " + insert.id
          );
        default:
      }
      return;
    }

    var boundaries:Array<materia.tooling.process.FillAnalysis.FillBoundary
      > = [for (feature in p.document.input.features) {
      id:feature.id, box:CastingMoldPlanner.featureBox(p.document, feature)
    }
    ];
    var resin = materia.casting.BoxAlgebra.subtract([p.envelope], [for (boundary in boundaries) boundary.box]);
    var report = materia.tooling.process.JunctionThickness.analyze(
      p.document.input.id,
      CastingMoldPlanner.fingerprint(p),
      resin,
      boundaries
    );
    check(report.converged, "actual mill junction diameter reaches its declared numerical tolerance");
    var owned:Array<cadkit.modeling.Part> = [];
    try {
      var local = frame.base.body.geometry();
      owned.push(local);
      var source = PosedParts.moved(local, p.document.setup.sourceToMold);
      owned.push(source);
      var area = 0.0;
      for (face in report.faces) {
        var b = face.region;
        area += face.axis
          == SectionAxis.X ? b.depth * b.height : face.axis == SectionAxis.Y ? b.width * b.height : b.width * b.depth;
      }
      check(
        Math.abs(area - source.shape.area()) < 1e-3,
        "mill exposed rectangles agree with actual foundation BREP area"
      );
      for (extra in [-1e-5, 0.05]) {
        var ball = new cadkit.modeling.Part(cadkit.Shape.sphere((extra
          < 0 ? report.lowerDiameterMm : report.upperDiameterMm) / 2 + extra));
        owned.push(ball);
        var placed = ball.translated(new Vector(report.witness.x, report.witness.y, report.witness.z));
        owned.push(placed);
        var outside = placed.subtract(source);
        owned.push(outside);
        check(
          extra < 0 ? outside.volume() < 1e-3 : outside.volume() > 1e-3,
          extra < 0 ? "mill witness sphere lies inside actual foundation CAD"
          : "enlarged mill witness crosses actual foundation CAD"
        );
      }
    } catch (error:Dynamic) {
      for (part in owned) if (!part.shape.isClosed()) part.close();
      throw error;
    }
    for (part in owned) if (!part.shape.isClosed()) part.close();
  }

  /** Real column source on the shared planner, without a kit export. */
  public static function columnPlan(frame:BenchMillFrame):Void {
    var count = 0;
    function verify(ok:Bool, message:String):Void {
      count++;
      if (!ok) throw message;
    }
    var plan = CastingMoldCad.plan(
      MillMoldSetup.document(frame, CastingShopRecipe.BOARD_THICKNESS_MM, Column),
      MillMoldSetup.options(frame, Column)
    );
    for (source in plan.document.input.features) if (source.kind
      != materia.casting.CastingFeature.CastingFeatureKind.InsertRecess) {
      var id = source.id;
      var records = plan.features.filter(feature -> feature.featureId == id);
      verify(
        records.length == 1 && records[0].kind == FeatureToolingKind.EPSFeature,
        "Column source cavity uses declared sacrificial EPS: " + id
      );
      verify(
        plan.formers.filter(former -> former.featureIds.indexOf(id) >= 0).length == 0,
        "Deep cavity needs no unbuildable rigid former: " + id
      );
      verify(
        plan.cores.filter(core -> records[0].memberIds.indexOf(core.id) >= 0).length > 0,
        "Deep cavity has physical EPS stock: " + id
      );
    }
    verify(plan.formers.length == 0, "Every deep column cavity has the declared sacrificial construction route");
    verify(
      plan.validation.filter(record -> record.rule == "tooling.feature.box-geometry"
      && record.status == materia.tooling.ToolingDiagnostic.ToolingCheckStatus.Unsupported).length == 0,
      "Column curved step cavities have exact supported tooling"
    );
    var source = CastingMoldCad.source(plan), reconstructed = CastingMoldCad.reconstructed(plan);
    var missing:cadkit.modeling.Part = null, extra:cadkit.modeling.Part = null;
    try {
      missing = source.subtract(reconstructed);
      extra = reconstructed.subtract(source);
      verify(missing.volume() + extra.volume() < 1e-5, "Stepped column mold reconstructs the exact source resin");
    } catch (error:Dynamic) {
      if (extra != null) extra.close();
      if (missing != null) missing.close();
      reconstructed.close();
      source.close();
      throw error;
    }
    extra.close();
    missing.close();
    reconstructed.close();
    source.close();
    for (record in CastingMoldCad.corePanelFit(plan)) verify(
      record.status == materia.tooling.ToolingDiagnostic.ToolingCheckStatus.Pass,
      record.message
    );
    Sys.println('Column mold plan: $count assertions passed');
  }

  public static function run(
    frame:BenchMillFrame,
    p:CastingMoldPlan,
    report:machinekit.tooling.CastingMoldCad.CastingMoldReview
  ):Void {
    check(p.cores.length == 10 && p.fixtures.length == 12, "actual mill pocket and insert inventory");
    check(
      p.features.length == p.document.input.features.length && p.features.length == 31,
      "every full-frame feature classified"
    );
    for (name in ["reconstruction", "coverage", "assembly-graph", "demolding-graph"]) check(
      report.validation.filter(v -> v.rule == "tooling.cad." + name && v.status == "pass").length == 1,
      "CAD review owns its " + name + " evidence"
    );
    var missingContext = "";
    try machinekit.structural.FoundationIntegrity.castingReview(frame.base,
      p.document) catch (error:Dynamic) missingContext = Std.string(error);
    check(
      missingContext.indexOf("Foundation casting review source differs from document") >= 0,
      "design review cannot silently trust the mold document's assembly context"
    );
    var design = machinekit.structural.FoundationIntegrity.castingReview(
      frame.base,
      p.document,
      MillMoldSetup.input(frame).interfaceContext
    );
    check(
      design.validation.length > 0 && design.metrics.liftingPointCount
      == p.document.input.inserts.filter(i -> i.role == materia.casting.InsertSpec.InsertRole.LiftingMount).length,
      "source-level design metrics map into foundation integrity"
    );
    check(
      report.symmetricDifferenceMm3 < 1e-3 && !report.fabrication.ready,
      "review geometry passes without fabricating readiness"
    );
    var rowGroups = report.assemblyGroups.filter(group -> group.members.filter(id -> p.cores.filter(core -> core.id
      == id).length == 1).length > 0);
    check(
      rowGroups.length == 2 && report.permanentGroups.length == 3,
      "two shared row fixtures and the bed-attached closures"
    );
    var anchored = p.document.input.inserts.filter(insert -> switch insert.anchorage {
      case Bolted(_, _):
        true;
      default:
        false;
    }
    );
    check(
      report.assemblyGroups.length == rowGroups.length + anchored.length,
      "each anchored insert has its own rigid insertion group alongside the core rows"
    );
    var first = report.assembly.assemblyOrder[0];
    check(
      first == "bed"
      || report.permanentGroups.filter(group -> group.id == first && group.members.indexOf("bed") >= 0).length == 1,
      "the bed, with its cured closures, installs first"
    );
    check(
      p.changeRequests.filter(c -> c.kind == "lifting-points").length == 0,
      "source lifting connections clear their design request"
    );
    check(
      p.validation.filter(check -> check.rule == "tooling.handling.flip" && check.status == "pending").length == 1,
      "rigging and flip evidence remain pending independently"
    );
    check(
      p.options.precision.length == p.document.input.inserts.length && p.options.foundationJointProcesses.length == 0,
      "process choices cover the source while the joint compound and cure evidence stay unresolved"
    );
    check(
      p.changeRequests.filter(request -> request.kind
      == materia.tooling.casting.CastingMoldPlan.DesignChangeKind.FoundationJointDefinition).length == 0,
      "the declared source joint clears its definition request"
    );
    var withoutJoint = materia.tooling.ToolingCodec.copyInput(p.document.input);
    if (withoutJoint.interfaceContext == null) throw "Mill source joint requires assembly context";
    withoutJoint.interfaceContext.foundationJoints = [];
    withoutJoint.interfaceContext.jointSources = {bodies: [], inserts: [], screws: []};
    var missingJoint = materia.tooling.casting.FoundationJointQualification.missingSource(withoutJoint);
    check(
      missingJoint != null && missingJoint.affectedIds.length == 4,
      "removing the source joint restores the four mounting-interface requests"
    );
    check(
      p.formers.filter(former -> StringTools.startsWith(former.volume.id, "bed-backing-")).length == 0,
      "rebated steel receivers remove the backing sliver formers"
    );
    check(
      p.changeRequests.filter(request -> request.kind
      == materia.tooling.casting.CastingMoldPlan.DesignChangeKind.InterfaceAllowance).length == 0,
      "explicit finishing stock clears the source allowance requests"
    );

    equivalent(frame, p);
    junctionGeometry(frame, p);
    for (c in p.cores.filter(c -> c.support.kind == materia.tooling.casting.CoreSupport.CoreSupportKind.Bridge)) {
      check(c.layers.length > 0, "every bridge core has wire-cut stock");
      for (layer in c.layers) check(
        layer.block.width <= p.document.recipe.core.width
        && layer.block.depth <= p.document.recipe.core.depth && layer.block.height <= p.document.recipe.core.height,
        "EPS layers fit the declared stock dimensions"
      );
      var first = MoldCores.singleBox(MoldCores.layerRegion(c.layers[0]), c.id);
      var last = MoldCores.singleBox(MoldCores.layerRegion(c.layers[c.layers.length - 1]), c.id);
      var cavity = MoldCores.singleBox(c.cavity, c.id);
      check(
        Math.abs(last.z + last.height + c.support.adhesiveThickness - cavity.z - cavity.height) < 1e-7,
        "glue line included in foam height"
      );
      for (index in 1...c.layers.length) {
        var before = MoldCores.singleBox(MoldCores.layerRegion(c.layers[index - 1]), c.id);
        var after = MoldCores.singleBox(MoldCores.layerRegion(c.layers[index]), c.id);
        check(
          Math.abs(after.z - before.z - before.height - c.support.adhesiveThickness) < 1e-7,
          "interlayer glue line included"
        );
      }
    }
    for (f in p.formers) {
      var solid = ToolingVolume.geometryOf(f.volume);
      check(
        solid.valid() && solid.shape.subshapeCount(CadKit.ShapeKind.Solid) == 1,
        "each former is one physical connected solid"
      );
      solid.close();
    }
    check(
      p.cores.filter(c -> c.support.registrationProfiles.length == 1).length == 4,
      "all access EPS cores retain exact router-relief sealing profiles"
    );
    var mismatched = CastingMoldPlanner.plan(p.document, p.options);
    var profiled = mismatched.cores.filter(c -> c.support.registrationProfiles.length > 0)[0];
    profiled.support.registrationProfiles[0].depth += 1;
    rejects(() -> CastingMoldPlanner.validate(mismatched), "former sealing profile must match panel operation");
    check(
      p.changeRequests.filter(c -> c.kind == "release-alternative").length
      == materia.tooling.casting.CastingStrategies.resolve(p.document).filter(f -> f.strategy
      == "reusable-former").length + p.formers.filter(f -> f.featureIds.length == 0).length,
      "every zero-draft rigid feature has a release decision"
    );
    check(
      p.validation.filter(v -> StringTools.startsWith(v.rule, "tooling.fixture.attachment.")).length
      == p.fixtures.filter(fixture -> MoldFixtures.volumes(fixture).length > 0).length,
      "fixture attachments are tooling pending records"
    );
    check(
      p.changeRequests.filter(c -> c.kind == "fixture-attachment").length == 0,
      "tool-only fixture attachments are not casting changes"
    );
    check(
      report.handling.castingMassKg > 0
      && report.demoldingSteps[report.demoldingSteps.length - 1].kind == "flip-to-service",
      "mass/CoM and flip appear in demolding sequence"
    );
    for (corner in ["LeftFront", "LeftRear", "RightFront", "RightRear"]) {
      var stems = p.document.input.features.filter(feature -> feature.id == "foot-stem-" + corner);
      check(
        stems.length == 1 && stems[0].geometry.pose.z + materia
        .casting.CastingFeature.CastingGeometrySolids.height(stems[0].geometry.solid) == frame.casting.footAccessFloor,
        "foot stem ends at its authored access-floor contact"
      );
    }

    var sourceFixings = [for (fixture in p.fixtures) for (fixing in fixture.fixings) fixing];
    check(sourceFixings.length == 8, "column anchors and lifting connections get source-thread fixture screws");
    for (fixture in p.fixtures) for (fixing in fixture.fixings) {
      var insert = p.document.input.inserts.filter(i -> i.id == fixture.insertId)[0];
      var hole = insert.holes.filter(h -> h.id == fixing.sourceHoleId)[0];
      var penetration = materia.tooling.casting.CastingMoldPlan.MoldOpenings.records(p,
        fixture.insertId).filter(pen -> pen.sourceHoleId == hole.id)[0];
      check(
        hole.thread != null && hole.extent == materia.casting.InsertHole.HoleExtent.Blind && penetration.plug == null
        && fixing.engagement + materia.casting.InsertHole.InsertHoleCodec.threadLimits(hole).minTipGap <= hole.depth
        + 1e-7 && materia.casting.InsertSpec.InsertBodyCodec.height(insert.body) - hole.depth >= hole.limits.minBack,
        "source fixing preserves the blind C45 back wall and has no buried plug"
      );
    }
    for (f in p.fixtures.filter(fixture -> MoldFixtures.volumes(fixture).length > 0)) {
      var volume = MoldFixtures.datum(f);
      check(
        p.seals.filter(seal -> seal.volume.id == "receiver-seal-" + volume.id
        && seal.insertId == f.insertId).length == 1,
        "datum receiver relief has its actual cured closure"
      );
      var datum = ToolingVolume.geometryOf(volume);
      check(datum.valid() && datum.solidCount() == 1, "bored datum support remains one valid connected solid");
      datum.close();
      check(
        MoldFixtures.lateral(f).baselineMm >= p.document.recipe.hardware.fixturePinSpacing
        && MoldFixtures.datum(f).cuts.length >= 2,
        "fixture has pin/slot and clearance-only fixing cuts"
      );
      check(
        materia.tooling.casting.CastingMoldPlan.MoldOpenings.records(p, f.insertId).length == 2 + f.fixings.length,
        "fixture bed cuts have explicit sealing face records"
      );
    }
    for (f in p.fixtures.filter(fixture -> MoldFixtures.volumes(fixture).length > 0)) {
      for (id in [MoldFixtures.lateral(f).primaryId, MoldFixtures.lateral(f).secondaryId]) {
        check(
          report.demolding.removalOrder.indexOf(id) < report.demolding.removalOrder.indexOf(MoldFixtures.datum(f).id),
          "through-bed pins withdraw before datum supports"
        );
        check(
          Math.abs(MoldFixtures.withdrawal(f).x) < 0.9
          || report.assembly.assemblyOrder.indexOf(MoldFixtures.datum(f).id)
          < report.assembly.assemblyOrder.indexOf(id),
          "datum supports seat before pins enter from below"
        );
      }
    }
    var savedAssembly = CastingMoldCad.assembly(p).encode();
    var restoredAssembly = machinekit.assembly.MachineAssembly.decode(savedAssembly);
    restoredAssembly.validate();
    check(restoredAssembly.encode() == savedAssembly, "full hardware assembly round-trips within connector limits");
    var unknown = "";
    try {
      var invalid = materia.tooling.ToolingCodec.copyInput(p.document.input);
      invalid.features[0].kind = cast "future-feature";
      materia.tooling.ToolingCodec.copyInput(invalid);
    } catch (e:Dynamic) {
      unknown = Std.string(e);
    }
    check(unknown.indexOf("Unknown casting feature kind") >= 0, "unknown feature kinds cannot inherit a strategy");
    var permuted = p.document.input.features.copy();
    permuted.reverse();
    var strategies = materia.tooling.casting.CastingStrategies.resolve(p.document);
    check(
      materia.tooling.ToolingCodec.canonical(CastingMoldPlanner.ownershipOrder(permuted,
      strategies)) == materia.tooling.ToolingCodec.canonical(CastingMoldPlanner.ownershipOrder(
        p.document.input.features,
        strategies
      )),
      "raw feature permutation preserves the actual ownership ordering before codec sorting"
    );
    var unsupported = materia.tooling.ToolingCodec.copy(p.document);
    var unsupportedChoice = materia.tooling.casting.CastingStrategies.resolve(unsupported)[0];
    unsupportedChoice.strategy = materia.tooling.casting.CastingSetup.CoreStrategy.Unsupported;
    var unsupportedKind = unsupported.input.features.filter(feature -> feature.id
      == unsupportedChoice.featureId)[0].kind;
    for (rule in unsupported.recipe.featureRules) if (rule.kind
      == unsupportedKind) rule.strategy = unsupportedChoice.strategy;
    var partial = CastingMoldPlanner.plan(unsupported, p.options), assessment = CastingMoldCad.assess(partial);
    check(
      assessment.geometry == null && assessment.diagnostics.hasErrors() && !partial.fabrication.ready,
      "unsupported feature reports through shared Diagnostics without CAD fabrication claims"
    );
    for (i in p.document.input.inserts) {
      var steel = CastingMoldCad.steel(p, i.id);
      for (protection in p.fixtures.filter(f -> f.insertId == i.id)[0].boreMasks) {
        var hole = i.holes.filter(h -> h.id == protection.sourceHoleId)[0];
        var hardware = p.hardware.filter(h -> h.id == protection.hardwareId)[0];
        var local = machinekit.tooling.ToolingPin.boreMask(hole).geometry();
        var world = PosedParts.moved(local, hardware.pose);
        local.close();
        var overlap = steel.intersect(world);
        check(overlap.volume() < 1e-5, "each bore mask follows its actual source hole and frame");
        overlap.close();
        world.close();
      }
      steel.close();
    }
    var changed = CastingMoldPlanner.plan(MillMoldSetup.document(frame, 17.8), MillMoldSetup.options(frame));
    check(
      CastingMoldPlanner.fingerprint(changed) != CastingMoldPlanner.fingerprint(p),
      "measured stock invalidates package"
    );
    equivalent(frame, changed);
    var bare = BenchMillFrame.create(frame.layout, null, null, frame.casting), simple = CastingMoldPlanner.plan(
      MillMoldSetup.document(bare),
      MillMoldSetup.options(bare)
    );
    check(
      simple.features.length < p.features.length
      && p.features.filter(f -> f.featureId == "rear-y-drive-seat").length == 1,
      "optional XY interfaces come from full source instance"
    );
    equivalent(bare, simple);
    var restored = CastingMoldPlanner.plan(p.document, p.options);
    restored.openings.pop();
    rejects(() -> CastingMoldPlanner.validate(restored), "unclassified contact opening");
    restored = CastingMoldPlanner.plan(p.document, p.options);
    var smaller = MoldCores.singleBox(restored.cores[0].cavity, restored.cores[0].id);
    smaller.height -= 2;
    restored.cores[0].cavity = materia.tooling.casting.MoldRegions.box(smaller);
    rejects(() -> CastingMoldCad.review(restored), "missing pocket geometry");
    restored = CastingMoldPlanner.plan(p.document, p.options);
    MoldFixtures.datum(restored.fixtures[0]).pieces[0].z += 30;
    rejects(() -> CastingMoldCad.review(restored), "displaced insert datum fixture");
    var core = p.cores.filter(c -> StringTools.startsWith(c.featureId, "foot-access-"))[0];
    var direction = MoldCores.withdrawal(core, p.panels.members.concat(p.supportPanels));
    var access = machinekit.structural.CastingRegionCad.part(core.cavity), castingPart = CastingMoldCad.source(p);
    rejects(
      () -> SweptRemoval.analyze([
        {id: core.id, part: access, direction: new Vector(-direction.x, -direction.y, -direction.z)},
        {id: "casting", part: castingPart, direction: new Vector(0, 0, 1)}
      ]),
      "inward foot access tooling trapped against actual casting"
    );
    access.close();
    castingPart.close();
    var review = report;
    var memberIds = [for (m in CastingMoldCad.assembly(p).components()) m.id];
    var playback = machinekit.tooling.MoldPlayback.frames(p, review, memberIds);
    var install = playback.filter(f -> f.phase == machinekit.tooling.MoldPlayback.MoldPlaybackPhase.Assembly);
    var demold = playback.filter(f -> f.phase == machinekit.tooling.MoldPlayback.MoldPlaybackPhase.Demolding);
    check(install.length == review.assemblyPlan.steps.length, "one playback snapshot per planned assembly operation");
    for (i in 0...review.assemblyPlan.steps.length) check(
      install[i].stepId == review.assemblyPlan.steps[i].id,
      "playback preserves the manufacturing assembly plan"
    );
    check(
      demold.length == review.demoldingSteps.length,
      "playback uses the actual demolding steps including release/flip"
    );
    check(
      install[install.length - 1].visibleMemberIds.length == memberIds.length,
      "final installation snapshot contains every hardware component"
    );
    check(
      demold[demold.length - 1].visibleMemberIds.join(",") == "cured-casting",
      "final scene retains the released casting"
    );
    check(
      demold[demold.length - 1].status == review.demoldingSteps[review.demoldingSteps.length - 1].status,
      "pending lift qualification stays visible in playback"
    );
    memberIds.reverse();
    check(
      materia.tooling.ToolingCodec.canonical(playback)
      == materia.tooling.ToolingCodec.canonical(machinekit.tooling.MoldPlayback.frames(
        p,
        review,
        memberIds
      )),
      "playback deterministic under member permutation"
    );
    var savedFingerprint = review.fingerprint, staleError = "";
    review.fingerprint = "stale";
    try machinekit.tooling.MoldPlayback.frames(p,
      review, memberIds) catch (error:Dynamic) staleError = Std.string(error);
    review.fingerprint = savedFingerprint;
    check(staleError.indexOf("Stale playback review") >= 0, "playback rejects stale CAD evidence");
    var missingError = "";
    try machinekit.tooling.MoldPlayback.frames(p,
      review, memberIds.filter(id -> id != "bed")) catch (error:Dynamic) missingError = Std.string(error);
    check(missingError.indexOf("Unknown playback member: bed") >= 0, "playback cannot silently omit a checked member");
    var group = review.assemblyGroups[0], cycleError = "";
    group.members.push(group.id);
    try machinekit.tooling.MoldPlayback.frames(p,
      review, memberIds) catch (error:Dynamic) cycleError = Std.string(error);
    group.members.pop();
    check(cycleError.indexOf("Cyclic playback group") >= 0, "playback reports cyclic group expansion");
    Sys.println('MillMoldChecks: $assertions assertions passed');
  }
}
