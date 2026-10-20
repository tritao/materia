package materia.tooling.casting;

import materia.assembly.AssemblyFrames as F;
import materia.casting.BoxAlgebra;
import materia.casting.CastingCavities;
import materia.tooling.ToolingDocument;
import materia.tooling.ToolingValidation;
import materia.tooling.process.FillAnalysis;
import materia.tooling.casting.MoldPlan.MoldBox;

typedef CastingBoxRegions = {
	var sourceId:String;
	var documentFingerprint:String;
	var regions:Null<{resin:Array<MoldBox>, boundaries:Array<FillBoundary>}>;
	var validation:Array<ToolingValidation>;
}

/** One source-to-analysis projection. Unsupported geometry has no box substitute. */
class CastingSourceRegions {
	static inline var RULE = "casting.source.box-geometry";

	public static function derive(document:ToolingDocument):CastingBoxRegions {
		var d = materia.tooling.ToolingCodec.snapshot(document);
		var checks = CastingBoxSupport.check(d.input, RULE);
		var result:CastingBoxRegions = {sourceId: d.input.id,
			documentFingerprint: materia.tooling.ToolingCodec.documentFingerprint(d), regions: null, validation: checks};
		if (checks.filter(check -> check.status == materia.tooling.ToolingDiagnostic.ToolingCheckStatus.Unsupported).length > 0)
			return result;
		for (axis in [F.transformVector(d.setup.sourceToMold, 1, 0, 0), F.transformVector(d.setup.sourceToMold, 0, 1, 0),
			F.transformVector(d.setup.sourceToMold, 0, 0, 1)])
			if (materia.casting.CastingBounds.axis(axis) == null) {
				result.validation = [FillAnalysis.check(RULE + "." + d.input.id, [d.input.id], Unsupported,
					"Unsupported non-axial setup for box analysis: " + d.input.id, [])];
				return result;
			}

		function placed(box:MoldBox):MoldBox {
			return BoxMoldPlanner.box(box.width, box.depth, box.height,
				F.compose(d.setup.sourceToMold, F.translation(box.x + box.width / 2, box.y + box.depth / 2, box.z)));
		}
		var envelope:CastingCavities.FinishedCastingCavity = {
			featureId: d.input.id, parentId: d.input.id,
			geometry: CastingCavities.FinishedCavityGeometry.Prisms(d.input.envelope)
		};
		var body:Array<MoldBox> = [];
		var boundaries:Array<FillBoundary> = [];
		for (box in CastingCavities.boxes(envelope)) body.push(placed(box));
		for (cavity in CastingCavities.derive(d.input.envelope, d.input.features))
			for (box in CastingCavities.boxes(cavity)) boundaries.push({id: cavity.featureId, box: placed(box)});
		result.regions = {resin: BoxAlgebra.subtract(body, [for (boundary in boundaries) boundary.box]), boundaries: boundaries};
		return result;
	}

	public static function checks(geometry:CastingBoxRegions, prefix:String):Array<ToolingValidation>
		return [for (check in geometry.validation) {
			rule: prefix + check.rule.substr(RULE.length), affectedIds: check.affectedIds.copy(),
			status: check.status, severity: check.severity, message: check.message, evidence: check.evidence.copy()
		}];
}
