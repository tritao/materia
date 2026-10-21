package materia.tooling.casting;

import materia.tooling.ToolingValidation;
import materia.tooling.casting.CastingToolingInput.CastingToolingInsert;
import materia.casting.InsertOrigin.InsertGeometryBasis;

/** Product provenance survives every planner; nominal geometry data never clears readiness. */
class InsertQualifications {
	public static function check(inserts:Array<CastingToolingInsert>):Array<ToolingValidation> {
		var result:Array<ToolingValidation> = [];
		for (insert in inserts) {
			result=result.concat(InsertAnchorageChecks.check(insert));
			var origin = switch insert.origin {case Fabricated(_):continue;case Catalogue(product):product;};
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

	public static function validateAnchorage(inserts:Array<CastingToolingInsert>,validation:Array<ToolingValidation>):Void {
		var expected:Array<ToolingValidation> = [];
		for (insert in inserts) expected=expected.concat(InsertAnchorageChecks.check(insert));
		var actual=validation.filter(check -> StringTools.startsWith(check.rule,"tooling.insert.anchorage."));
		expected.sort((a,b) -> Reflect.compare(a.rule,b.rule));actual.sort((a,b) -> Reflect.compare(a.rule,b.rule));
		if (materia.tooling.ToolingCodec.canonical(expected)!=materia.tooling.ToolingCodec.canonical(actual))
			throw "Insert anchorage validation differs from source";
	}

	public static function manufacturing(inserts:Array<CastingToolingInsert>):Array<ToolingValidation> {
		var result:Array<ToolingValidation> = [];
		for (insert in inserts) switch insert.origin {
			case Fabricated(_):result=result.concat(InsertManufacturingPlan.build(insert).validation);
			case Catalogue(_):
		}
		return result;
	}

	public static function validateManufacturing(inserts:Array<CastingToolingInsert>,validation:Array<ToolingValidation>):Void {
		for (expected in manufacturing(inserts)) {
			var found=validation.filter(check -> check.rule==expected.rule);
			if (found.length!=1 || materia.tooling.ToolingCodec.canonical(found[0])!=materia.tooling.ToolingCodec.canonical(expected))
				throw "Insert manufacturing validation differs from source: "+expected.rule;
		}
	}
}
