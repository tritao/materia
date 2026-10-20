package materia.tooling.casting;

import materia.tooling.ToolingValidation;
import materia.tooling.casting.CastingToolingInput.CastingToolingInsert;
import materia.casting.InsertOrigin.InsertGeometryBasis;

/** Product provenance survives every planner; nominal geometry data never clears readiness. */
class InsertQualifications {
	public static function check(inserts:Array<CastingToolingInsert>):Array<ToolingValidation> {
		var result:Array<ToolingValidation> = [];
		for (insert in inserts) {
			var origin = switch insert.origin {case Fabricated: continue; case Catalogue(product): product;};
			var rule = "tooling.insert.geometry." + insert.id;
			if (origin.geometryBasis == NominalStudy)
				result.push(ToolingReadiness.pending(rule, [insert.id],
					"Supplier geometry dimensions are a nominal study; verify the actual product before fabrication"));
			else
				result.push({rule: rule, affectedIds: [insert.id], status: materia.tooling.ToolingDiagnostic.ToolingCheckStatus.Pass,
					severity: materia.tooling.ToolingDiagnostic.ToolingSeverity.Info,
					message: "Geometry evidence is bound to the declared insert geometry; anchorage capacity is a separate check",
					evidence: [for (evidence in origin.evidence) evidence.reference + " sha256:" + evidence.sha256]});
		}
		return result;
	}
}
