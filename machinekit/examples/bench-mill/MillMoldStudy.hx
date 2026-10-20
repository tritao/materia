import materia.tooling.ToolingCodec;
import materia.tooling.process.MoldOrientationStudy;
import materia.tooling.process.OrthogonalSections;
import materia.tooling.process.FillAnalysis;
import materia.tooling.ToolingDiagnostic.ToolingCheckStatus;
import materia.assembly.AssemblyFrames as F;
import machinekit.tooling.CastingMoldCad;

/** Three declared setups. No source changes, strategy search or automatic approval. */
class MillMoldStudy {
	public static function candidates(frame:machinekit.milling.MillFrame, ?process:materia.tooling.process.CastingProcessSpec):Array<MoldOrientationCandidate> {
		var current = MillMoldSetup.document(frame),
			options = MillMoldSetup.options(frame);
		var upright = MoldOrientationStudy.reorient(current, "base-upright", F.identity());
		var side = MoldOrientationStudy.reorient(current, "base-on-side", F.fromRotationMatrix(0, 0, 0, [0, 0, -1, 0, 1, 0, 1, 0, 0]));
		return [
			for (d in [current, upright, side])
				{
					id: d.setup.id,
					document: d,
					options: options,
					process: candidateProcess(d, current.setup.id, process)
				}
		];
	}

	static function candidateProcess(d:materia.tooling.ToolingDocument, currentId:String,
			process:Null<materia.tooling.process.CastingProcessSpec>):materia.tooling.process.CastingProcessSpec {
		var selected = process == null ? MillFabricationSetup.process(d.recipe) : materia.tooling.process.ProcessSpecCodec.snapshot(process);
		// Thermal contact conditions are specific to an orientation. Material data can travel;
		// authored slab/boundary assignments require an explicit reassessment for alternatives.
		if (d.setup.id != currentId)
			selected.sections = [];
		return selected;
	}

	public static function study(frame:machinekit.milling.MillFrame, ?existing:machinekit.tooling.CastingMoldCad.CastingMoldReview, geometry:Bool = true,
			?process:materia.tooling.process.CastingProcessSpec):MoldOrientationReport {
		if (!geometry)
			return MoldOrientationStudy.run(candidates(frame, process), MillFabricationSetup.spec());
		return MoldOrientationStudy.run(candidates(frame, process), MillFabricationSetup.spec(), p -> {
			var hash = materia.tooling.casting.CastingMoldCodec.fingerprint(p);
			var review = existing != null && existing.fingerprint == hash ? existing : CastingMoldCad.review(p);
			return {
				planFingerprint: review.fingerprint,
				assemblySteps: review.assembly.assemblyOrder.length,
				demoldingSteps: review.demoldingSteps.length,
				assemblyBlockingEdges: review.assembly.blocking.length,
				demoldingBlockingEdges: review.demolding.blocking.length,
				validation: review.validation
			};
		});
	}

	public static function write(report:MoldOrientationReport, dir:String):Void {
		machinekit.tooling.ToolingFiles.directory(dir);
		sys.io.File.saveContent(dir + "/orientation-comparison.json", ToolingCodec.canonical(report) + "\n");
		var text = [
			"# Mill-base mold orientation comparison",
			"",
			"REVIEW ONLY. No source/setup is changed and no fabrication approval is granted.",
			"",
			"Ranking: " + report.rankingRule +
			". Candidates retain the same feature strategy and core-support policy; planning failure requests a tooling redesign, not a claim that an orientation is inherently impossible.",
			"",
			"| Setup | Planned | CAD checked | Air pockets | Flip | Source requests | Fail / unsupported / pending |",
			"| --- | --- | --- | ---: | --- | ---: | --- |"
		];
		for (r in report.rankedCandidates)
			text.push("| "
				+ r.id
				+ " | "
				+ r.planningSucceeded
				+ " | "
				+ r.geometryChecked
				+ " | "
				+ (r.fill == null ? "unsupported" : Std.string(r.fill.trappedPockets.length))
				+ " | "
				+ (r.handling == null ? "unsupported" : Std.string(r.handling.requiresFlip))
				+ " | "
				+ (r.planningSucceeded ? Std.string(r.changeRequests.length) : "not evaluated")
				+ " | "
				+ MoldOrientationStudy.count(r, ToolingCheckStatus.Fail)
				+ " / "
				+ MoldOrientationStudy.count(r, ToolingCheckStatus.Unsupported)
				+ " / "
				+ MoldOrientationStudy.count(r, ToolingCheckStatus.Pending)
				+ " |");
		for (r in report.rankedCandidates) {
			text.push("\n## " + r.id + "\n");
			text.push(r.decidingReason + ": " + ToolingCodec.canonical(r.rankMetrics));
			if (r.planningFailure != null)
				text.push("Planning failed: " + r.planningFailure);
			if (r.fill == null || r.cure == null || r.sections == null || r.castability == null || r.thermalCoverage == null || r.handling == null) {
				text.push("Process analysis unsupported or failed; see validation records.");
				continue;
			}

			text.push("Casting mass: "
				+ r.handling.castingMassKg
				+ " kg. Lumped peak estimate: "
				+ r.cure.lumpedPeakEstimateC
				+ " C (null means unknown).");
			if (r.effort != null) {
				var e = r.effort;
				text.push("Panels: " + e.panels + " (" + e.uniquePanels + " unique); rigid formers: " + e.formers + "; EPS cores: " + e.epsCores
					+ "; production blank sheets: " + e.sheetCount + "; logical hardware stacks: " + e.logicalHardwareStacks + ".");
			}
			text.push("Castability: minimum opposing-face span " + r.castability.minimumOpposingFaceSpanMm + " mm; " + r.castability.proposals.length
				+ " review suggestions (not applied).");
			text.push("Inscribed diameter: "
				+ r.sections.junction.lowerDiameterMm
				+ ".."
				+ r.sections.junction.upperDiameterMm
				+ " mm (geometry tolerance reached: "
				+ r.sections.junction.converged
				+ "). Unbound thermal volume: "
				+ r.thermalCoverage.uncoveredVolumeMm3
				+ " mm3; geometry bounds do not establish a junction thermal model.");
			for (v in r.validation)
				if (v.status != ToolingCheckStatus.Pass)
					text.push("- `" + v.rule + "` (" + v.status + "): " + v.message);
		}
		sys.io.File.saveContent(dir + "/orientation-comparison.md", text.join("\n") + "\n");
	}

	public static function main():Void {
		var cwd = Sys.getCwd(),
			root = sys.FileSystem.exists(cwd + "/machinekit") ? cwd + "/machinekit/examples/bench-mill" : cwd;
		var report = study(MillMoldSetup.frame()),
			dir = root + "/build/tooling/orientation-study";
		write(report, dir);
		Sys.println("Mold orientation review: " + dir + "; preference=" + report.preferredReviewCandidateId);
	}
}
