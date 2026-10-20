package materia.tooling;

import materia.tooling.ToolingDiagnostic.ToolingCheckStatus;
import materia.tooling.ToolingDiagnostic.ToolingSeverity;
import materia.tooling.casting.CastingSetup.CoreStrategy;

/** Input-contract review only. Passing the codec never establishes fabrication readiness. */
class ToolingReview {
	public static function inspect(document:ToolingDocument):Array<ToolingDiagnostic> {
		var d = ToolingCodec.snapshot(document);
		var result:Array<ToolingDiagnostic> = [
			{ruleId: "tooling.input-contract", subjectId: d.input.id, severity: Info, status: Pass,
				message: "Input, references and setup fingerprint are valid", evidence: []},
			{ruleId: "tooling.geometry", subjectId: d.setup.id, severity: Warning, status: Pending,
				message: "Panel generation, cavity equivalence and removal paths have not been checked", evidence: []},
			{ruleId: "tooling.process", subjectId: d.recipe.id, severity: Warning, status: Pending,
				message: "Resin, EPS, covering, adhesive and cure behavior require physical qualification",
				evidence: d.recipe.processEvidence.copy()},
			{ruleId: "tooling.loads", subjectId: d.recipe.id, severity: Warning, status: Pending,
				message: "Panel, fixture and core-restraint loads have not been checked", evidence: []}
		];
		if (d.recipe.core.compressionStrengthPa == null || d.recipe.core.densityKgM3 == null ||
			d.recipe.core.maximumTemperatureC == null || d.recipe.core.evidence.length == 0)
			result.push({ruleId: "tooling.core-stock", subjectId: d.recipe.core.materialSpec, severity: Warning, status: Pending,
				message: "EPS supplier properties or evidence are incomplete", evidence: d.recipe.core.evidence.copy()});
		for (f in materia.tooling.casting.CastingStrategies.resolve(d)) if (f.strategy == Unsupported)
			result.push({ruleId: "tooling.core-strategy", subjectId: f.featureId, severity: Error, status: Unsupported,
				message: "No supported tooling strategy has been selected for this feature", evidence: []});
		for (check in materia.tooling.casting.InsertQualifications.check(d.input.inserts))
			result.push({ruleId: check.rule, subjectId: check.affectedIds[0], status: check.status,
				severity: check.severity, message: check.message, evidence: check.evidence.copy()});
		return result;
	}
}
