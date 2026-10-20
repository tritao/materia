package materia.tooling.process;

import materia.tooling.ToolingCodec;
import materia.tooling.casting.CastingMoldPlan;
import materia.tooling.casting.CastingMoldPlanner;
import materia.tooling.casting.CastingMoldCodec;
import materia.casting.BoxAlgebra;
import materia.tooling.process.CastingProcessSpec;
import materia.tooling.process.FillAnalysis;
import materia.tooling.process.CureAnalysis;

typedef CastingProcessReport = {
	var version:Int;
	var sourceFingerprint:String;
	var processFingerprint:String;
	var spec:CastingProcessSpec;
	var fill:FillReport;
	var cure:CureReport;
	var sections:OrthogonalSections.OrthogonalSectionReport;
	var castability:CastabilityAnalysis.CastabilityReport;
	var thermalCoverage:ThermalCoverage.ThermalCoverageReport;
	var validation:Array<materia.tooling.ToolingValidation>;
	var fabrication:materia.tooling.ToolingValidation.FabricationReadiness;
}

/** Process checks extend the real mold planner's validations. They never clear
 * its requests or substitute a separate geometry/readiness decision.
 */
class CastingProcessAnalysis {
	public static function validateSectionSources(document:materia.tooling.ToolingDocument, spec:CastingProcessSpec):Void {
		var knownIds = [document.input.id];
		for (feature in document.input.features)
			knownIds.push(feature.id);
		for (insert in document.input.inserts)
			knownIds.push(insert.id);
		for (section in spec.sections)
			for (id in section.sourceIds)
				if (knownIds.indexOf(id) < 0)
					throw "Unknown cure section source: " + id;
	}

	public static function analyze(p:CastingMoldPlan, raw:CastingProcessSpec, gravityMps2:Float,
			?fabrication:materia.tooling.export.FabricationSpec):CastingProcessReport {
		CastingMoldPlanner.validate(p);
		if (fabrication != null)
			fabrication = materia.tooling.export.FabricationSpecCodec.snapshot(fabrication);
		var spec = ProcessSpecCodec.snapshot(raw);
		if (spec.recipeFingerprint != ToolingCodec.recipeFingerprint(p.document.recipe))
			throw "Stale process recipe fingerprint";
		var sourceFingerprint = CastingMoldCodec.fingerprint(p);
		var fingerprint = haxe.crypto.Sha256.encode(ToolingCodec.canonical({
			source: sourceFingerprint,
			spec: spec,
			gravityMps2: gravityMps2,
			fabrication: fabrication == null ? null : {resinReference: fabrication.resinReference, adhesiveContainsEPS: fabrication.adhesiveContainsEPS}
		}));
		var features = p.document.input.features.copy();
		features.sort((a, b) -> Reflect.compare(a.id, b.id));
		var boundaries:Array<FillBoundary> = [
			for (f in features)
				{id: f.id, box: CastingMoldPlanner.featureBox(p.document, f)}
		];
		var resin = BoxAlgebra.subtract([p.envelope], [for (b in boundaries) b.box]);
		var projection = FillTooling.project(p);
		var panels:Array<FillPanel> = [
			for (member in p.panels.members.concat(p.supportPanels)) {
				var bounds = materia.tooling.panel.PanelGeometry.bounds(materia.tooling.panel.PanelContours.outside(member.panel));
				var bottom = Math.POSITIVE_INFINITY;
				for (x in [bounds.x, bounds.x + bounds.width])
					for (y in [bounds.y, bounds.y + bounds.height])
						for (z in [0.0, member.panel.stock.measuredThickness])
							bottom = Math.min(bottom, materia.assembly.AssemblyFrames.transformPoint(member.pose, x, y, z).z);
				{id: member.panel.id, bottomMm: Math.max(p.envelope.z, bottom)};
			}
		];
		var fill = FillAnalysis.analyze(p.document.input.id, sourceFingerprint, fingerprint, resin, boundaries,
			[for (c in p.cores) {id: c.id, boxes: [c.cavity]}], p.document.setup.fillHeightMm, p.document.recipe.mixDensityKgM3, gravityMps2,
			spec.pourRateMm3PerSecond, spec.maximumAirEscapeWidthMm, null, projection.blockers, panels);
		if (projection.unsupportedIds.length > 0)
			fill.validation.push(FillAnalysis.check("process.fill.tooling-projection", projection.unsupportedIds,
				materia.tooling.ToolingDiagnostic.ToolingCheckStatus.Unsupported,
				"Curved tooling openings intersect the fill footprint. They are conservatively blocked; exact curved projection is not implemented.", []));
		validateSectionSources(p.document, spec);
		var cure = CureAnalysis.analyze(p.document.input.id, sourceFingerprint, fingerprint, spec, p.document.recipe.mixDensityKgM3,
			p.document.recipe.core.maximumTemperatureC, p.cores.length > 0
			|| fabrication == null
			|| fabrication.adhesiveContainsEPS != false);
		cure.validation.push(FillAnalysis.check("process.cure.resin-identity", [p.document.recipe.id],
			fabrication == null ? materia.tooling.ToolingDiagnostic.ToolingCheckStatus.Pending : fabrication.resinReference != spec.cure.reference ? materia.tooling.ToolingDiagnostic.ToolingCheckStatus.Fail : materia.tooling.ToolingDiagnostic.ToolingCheckStatus.Pass,
			"Cure reference must name the same resin as the fabrication solvent record.",
			fabrication == null ? [] : [fabrication.resinReference, spec.cure.reference]));
		var sections = OrthogonalSections.analyze(p.document.input.id, sourceFingerprint, resin, boundaries);
		for (contact in spec.faceContacts)
			if (sections.junction.faces.filter(face -> ThermalFaces.id(face) == contact.faceId).length != 1)
				throw "Unknown thermal face contact: " + contact.faceId;
		var sectionChecks = sections.validation;
		var castability = CastabilityAnalysis.analyze(p.document, spec, fill, sections, sourceFingerprint, fingerprint, p.validation);
		var thermalCoverage = ThermalCoverage.analyze(p.document.input.id, sections, spec, fingerprint, p.document);
		var coverageChecks = thermalCoverage.validation;
		var geometric = thermalCoverage.validation.filter(v -> v.rule == "process.cure.geometric-coverage")[0];
		cure.validation = cure.validation.filter(v -> v.rule != "process.cure.section-coverage");
		cure.validation.push(FillAnalysis.check("process.cure.section-coverage", geometric.affectedIds, geometric.status, geometric.message,
			geometric.evidence));
		cure.validation = cure.validation.concat(coverageChecks);
		var validation = p.validation.concat(cure.validation).concat(sectionChecks);
		for (check in castability.validation)
			if (validation.filter(v -> v.rule == check.rule).length == 0)
				validation.push(check);
		validation.sort((a, b) -> Reflect.compare(a.rule, b.rule));
		return {
			version: 5,
			sourceFingerprint: sourceFingerprint,
			processFingerprint: fingerprint,
			spec: spec,
			fill: fill,
			cure: cure,
			sections: sections,
			castability: castability,
			thermalCoverage: thermalCoverage,
			validation: validation,
			fabrication: materia.tooling.ToolingValidation.ToolingReadiness.evaluate(validation)
		};
	}
}
