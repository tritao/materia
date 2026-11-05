import materia.tooling.casting.CastingMoldPlanner;
import machinekit.tooling.CastingMoldCad;

/** Phase 4 acceptance is a test suite, independent of full artifact export. */
class MillMoldSuite {
  public static function main():Void {
    var small = Sys.args().indexOf("--small") >= 0;
    var frame = MillProject.smallFrame();
    if (Sys.args().indexOf("--column-plan") >= 0) {
      MillMoldChecks.columnPlan(frame);
      return;
    }
    var p = small ? CastingMoldPlanner.planSinglePocket(SinglePocketSetup.document()) : CastingMoldCad.plan(
      MillMoldSetup.document(frame),
      MillMoldSetup.options(frame)
    );
    var review = CastingMoldCad.review(p);
    if (!small && Sys.args().indexOf("--assembly-only") < 0) MillMoldChecks.run(frame, p, review);
    var scene = machinekit.assembly.AssemblyPreview.scene(CastingMoldCad.assembly(p), "casting-presentation");
    scene.presentation = machinekit.tooling.ToolingPresentation.casting(p, scene, review);
    var steps = scene.presentation.procedures[0].steps;
    if (steps.length != review.demoldingSteps.length) throw "Casting presentation omitted planned steps";
    for (index in 0...steps.length) if (steps[index].id
      != review.demoldingSteps[index].id) throw "Casting presentation reordered planned steps";
    var assembly = scene.presentation.procedures[1];
    var planned = review.assemblyPlan.steps, byStep = new Map<String, Int>();
    if (assembly.steps.length != planned.length) throw "Presentation changed the assembly operation count";
    for (index in 0...planned.length) {
      var step = planned[index];
      if (assembly.steps[index].id != step.id) throw "Presentation reordered the manufacturing plan";
      for (dependency in step.dependsOn) if (!byStep.exists(dependency)) throw "Unmet planned assembly dependency";
      byStep.set(step.id, index);
      if (step.evidence.kind == materia.tooling.casting.MoldAssemblyPlanner.MoldAssemblyEvidenceKind.ManualOperation) {
        if (step.clearance.x != 0
          || step.clearance.y != 0 || step.clearance.z != 0) throw "Manual operation invented a motion path";
        for (member in materia.presentation
          .StepMotions.members(assembly.steps[index].action)) if (member.motion.evidence != materia.presentation
          .AssemblyPresentation.MotionEvidence.Illustrative) throw "Manual operation claims checked motion evidence";
      }
    }
    if (assembly.kind
      != materia.presentation.AssemblyPresentation.ProcedureKind.Assembly) throw "Missing assembly procedure";
    function steelId(id:String):String return switch p.construction {
      case SinglePocket:
        id;
      case SharedBridges:
        materia.tooling.ToolingIds.derive("insert-", id);
    };
    var installed = new Map<String, Int>();
    for (index in 0
      ...assembly.steps.length) for (member in materia.presentation.StepMotions.members(assembly.steps[index].action)) {
      if (installed.exists(member.select.occurrence)) throw "Duplicate installation: " + member.select.occurrence;
      installed.set(member.select.occurrence, index);
    }
    function indexOf(id:String):Int {
      var index = installed.get(id);
      if (index == null) throw "Missing installation: " + id;
      return index;
    }
    var base = p.panels.members[0].panel.id;
    if (indexOf(base) != 0) throw "Assembly must begin with the base";
    for (fixture in p.fixtures) {
      var insert = steelId(fixture.insertId);
      var support = switch fixture.support {
        case OnPanel(id):
          id;
        case OnDatum(volume, _, _):
          volume.id;
      };
      if (!installed.exists(insert) || !installed.exists(support)
        || indexOf(support) >= indexOf(insert)) throw "Steel insert must follow its locating support: " + insert;
      for (fixing in fixture.fixings) if (indexOf(fixing.hardwareId)
        <= indexOf(insert)) throw "Fixings must follow steel placement";
    }
    var definition = scene.assemblyDefinition;
    var flat = materia.assembly.AssemblyDefinitionFlattener.flattenView(definition);
    var poses = new Map<String, materia.assembly.AssemblyRecord.AssemblyFrame>();
    for (part in flat.occurrences) {
      if (!installed.exists(part.id)) throw "Assembly omitted occurrence: " + part.id;
      poses.set(part.id, part.initialPose);
    }
    var cursor = materia.presentation.PresentationEvaluator.reset(scene.presentation);
    cursor = new materia.presentation.PresentationEvaluator.PresentationCursor(
      cursor.view, cursor.explode, assembly.id, cursor.step, cursor.progress
    );
    var initial = materia.presentation.PresentationEvaluator.evaluate(
      definition,
      poses,
      scene.presentation,
      cursor,
      review.fingerprint
    );
    for (insert in p.document.input.inserts) if (initial.visibility.get(steelId(insert.id))
      != materia.presentation.AssemblyPresentation.VisibilityMode.Hidden) throw "Unplaced steel must be hidden";
    cursor = new materia.presentation.PresentationEvaluator.PresentationCursor(
      cursor.view, cursor.explode, cursor.procedure, assembly.steps.length, cursor.progress
    );
    var complete = materia.presentation.PresentationEvaluator.evaluate(
      definition,
      poses,
      scene.presentation,
      cursor,
      review.fingerprint
    );
    for (part in flat.occurrences) if (complete.visibility.get(part.id) != materia
      .presentation.AssemblyPresentation.VisibilityMode.Shown) throw "Completed assembly must show every part";
    materia.project.SceneArtifact.encode(scene);
    var playback = machinekit.tooling.MoldPlayback.frames(p, review, [for (part in flat.occurrences) part.id]);
    var assemblyFrames = playback.filter(frame -> frame.phase
      == machinekit.tooling.MoldPlayback.MoldPlaybackPhase.Assembly);
    if (assemblyFrames.length != planned.length) throw "Playback changed the manufacturing operation count";
    for (index in 0...planned.length) if (assemblyFrames[index].stepId
      != planned[index].id) throw "Playback reordered the manufacturing assembly plan";
    var orphan:materia.tooling.casting.CoreSupport.SupportHardware = {
      id: "orphan-fastener",
      kind: LocatingPin,
      pose: materia.assembly.AssemblyFrames.identity(),
      diameter: 3,
      length: 10
    };
    p.hardware.push(orphan);
    var rejected = false;
    try materia.tooling.casting.MoldAssemblyPlanner.build(
      p,
      review.assembly,
      review.assemblyMotions,
      review.assemblyGroups.concat(review.permanentGroups)
    ) catch (error:Dynamic) rejected = Std.string(error).indexOf("Missing hardware attachment relationship") >= 0;
    p.hardware.pop();
    if (!rejected) throw "Unclassified hardware received a fallback assembly step";
    Sys.println("Casting presentation: planner step order and artifact validation passed");
  }
}
