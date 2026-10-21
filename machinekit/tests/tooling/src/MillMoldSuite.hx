import materia.tooling.casting.CastingMoldPlanner;
import machinekit.tooling.CastingMoldCad;

/** Phase 4 acceptance is a test suite, independent of full artifact export. */
class MillMoldSuite {
	public static function main():Void {
		var frame = MillMoldSetup.frame();
		var p = CastingMoldPlanner.plan(MillMoldSetup.document(frame), MillMoldSetup.options(frame));
		var review=CastingMoldCad.review(p);
        MillMoldChecks.run(frame,p,review);
        var scene=machinekit.assembly.AssemblyPreview.scene(CastingMoldCad.assembly(p),"casting-presentation");
        scene.presentation=machinekit.tooling.ToolingPresentation.casting(p,scene,review);
        var steps=scene.presentation.procedures[0].steps;
        if(steps.length!=review.demoldingSteps.length)throw "Casting presentation omitted planned steps";
        for(index in 0...steps.length)if(steps[index].id!=review.demoldingSteps[index].id)throw "Casting presentation reordered planned steps";
        materia.project.SceneArtifact.encode(scene);
        Sys.println("Casting presentation: planner step order and artifact validation passed");
	}
}
