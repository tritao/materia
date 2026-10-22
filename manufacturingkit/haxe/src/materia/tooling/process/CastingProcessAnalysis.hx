package materia.tooling.process;

import materia.tooling.ToolingCodec;
import materia.tooling.casting.CastingMoldPlan;
import materia.tooling.casting.CastingMoldPlanner;
import materia.tooling.casting.CastingMoldCodec;
import materia.tooling.process.CastingProcessSpec;
import materia.tooling.process.FillAnalysis;
import materia.tooling.process.CureAnalysis;

/** Optional exact geometry implementation supplied by the native application.
 * All process, cure, pour and readiness decisions remain in this pipeline. */
typedef ExactFillBackend = (CastingMoldPlan,FillSourceSettings)->FillAssessment;

typedef CastingProcessReport = {
	var version:Int;
	var sourceFingerprint:String;
	var processFingerprint:String;
	var spec:CastingProcessSpec;
	var fill:Null<FillReport>;
	var pour:Null<PourPlanning.PourAssessment>;
	var cure:CureReport;
	var sections:Null<OrthogonalSections.OrthogonalSectionReport>;
	var castability:CastabilityAnalysis.CastabilityReport;
	var thermalCoverage:Null<ThermalCoverage.ThermalCoverageReport>;
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
			?fabrication:materia.tooling.export.FabricationSpec,?exactFill:ExactFillBackend):CastingProcessReport {
		CastingMoldPlanner.validate(p);
		if (fabrication != null)
			fabrication = materia.tooling.export.FabricationSpecCodec.snapshot(fabrication);
		var spec = ProcessSpecCodec.snapshot(raw);
		if (spec.recipeFingerprint != ToolingCodec.recipeFingerprint(p.document.recipe))
			throw "Stale process recipe fingerprint";
		var sourceFingerprint = CastingMoldCodec.fingerprint(p);
		var fingerprint=CastingProcessAnalysis.fingerprint(p,spec,gravityMps2,fabrication);
		var projection = FillTooling.project(p);
		var panels:Array<FillPanel> = [
			for (member in p.panels.members.concat(p.supportPanels)) {
				var bounds = materia.casting.CastingProfileGeometry.bounds(materia.tooling.panel.PanelContours.outside(member.panel));
				var bottom = Math.POSITIVE_INFINITY;
				for (x in [bounds.x, bounds.x + bounds.width])
					for (y in [bounds.y, bounds.y + bounds.height])
						for (z in [0.0, member.panel.stock.measuredThickness])
							bottom = Math.min(bottom, materia.assembly.AssemblyFrames.transformPoint(member.pose, x, y, z).z);
				{id: member.panel.id, bottomMm: Math.max(p.envelope.z, bottom)};
			}
		];
		var fillSettings:FillSourceSettings = {
			sourceFingerprint: sourceFingerprint, processFingerprint: fingerprint,
			cores: [for (c in p.cores) {id: c.id, geometry: c.cavity}], densityKgM3: p.document.recipe.mixDensityKgM3,
			gravityMps2: gravityMps2, pourRateMm3PerSecond: spec.pourRateMm3PerSecond,
			maximumAirEscapeWidthMm: spec.maximumAirEscapeWidthMm, extraLevels: [], fillObstructions: projection.blockers, panels: panels
		};
		var geometry = materia.tooling.casting.CastingSourceRegions.derive(p.document);
		if (geometry.regions == null) {
			var assessment=exactFill==null ? null : exactFill(p,fillSettings);
			var fill=assessment==null ? null : assessment.report;
			var checks=(assessment==null ? materia.tooling.casting.CastingSourceRegions.checks(geometry,"process.fill.geometry")
				: assessment.validation.copy()).concat(materia.tooling.casting.CastingSourceRegions.checks(geometry,"process.sections.geometry"));
			var castability=CastabilityAnalysis.analyze(p.document,spec,fill,null,sourceFingerprint,fingerprint,p.validation);
			var pour=fill==null ? null : planPour(p,spec,fill);
			validateSectionSources(p.document,spec);
			var cure=CureAnalysis.analyze(p.document.input.id,sourceFingerprint,fingerprint,spec,p.document.recipe.mixDensityKgM3,
				p.document.recipe.core.maximumTemperatureC,p.cores.length>0 || fabrication==null || fabrication.adhesiveContainsEPS!=false);
			cure.validation=cure.validation.filter(check -> check.rule!="process.cure.section-coverage"
				&& check.rule!="process.cure.geometric-coverage");
			for (rule in ["process.cure.section-coverage","process.cure.geometric-coverage"])
				cure.validation.push(FillAnalysis.check(rule,[p.document.input.id],materia.tooling.ToolingDiagnostic.ToolingCheckStatus.Unsupported,
					"Exact non-box thermal geometry is unsupported; no box coverage is credited.",[]));
			cure.validation.push(resinIdentity(p.document.recipe.id,spec,fabrication));
			if (pour!=null) {
				checks=checks.concat(pour.validation);
				for (check in pour.validation)
					if (check.rule=="process.pour.cure-mass-basis" || check.rule=="process.pour.cure-sequence") cure.validation.push(check);
			}
			checks=checks.concat(cure.validation).concat(p.validation).concat(castability.validation);
			var unique=new Map<String,materia.tooling.ToolingValidation>();
			for (check in checks) {
				if (unique.exists(check.rule) && ToolingCodec.canonical(unique.get(check.rule))!=ToolingCodec.canonical(check))
					throw "Conflicting process validation: "+check.rule;
				unique.set(check.rule,check);
			}
			checks=[for (check in unique) check];
			checks.sort((a,b) -> Reflect.compare(a.rule,b.rule));
			return {version:ProcessSpecCodec.VERSION,sourceFingerprint:sourceFingerprint,processFingerprint:fingerprint,spec:spec,
				fill:fill,pour:pour,cure:cure,sections:null,castability:castability,thermalCoverage:null,validation:checks,
				fabrication:materia.tooling.ToolingValidation.ToolingReadiness.evaluate(checks)};
		}
		var fillAssessment=FillAnalysis.assessRegions(geometry,p.document.setup.fillHeightMm,fillSettings);
		if (fillAssessment.report == null) throw "Unsupported source fill geometry";
		var fill:FillAnalysis.FillReport = cast fillAssessment.report;
		if (projection.unsupportedIds.length > 0)
			fill.validation.push(FillAnalysis.check("process.fill.tooling-projection", projection.unsupportedIds,
				materia.tooling.ToolingDiagnostic.ToolingCheckStatus.Unsupported,
				"Curved tooling openings intersect the fill footprint. They are conservatively blocked; exact curved projection is not implemented.", []));
		var pour=planPour(p,spec,fill);
		validateSectionSources(p.document, spec);
		var cure = CureAnalysis.analyze(p.document.input.id, sourceFingerprint, fingerprint, spec, p.document.recipe.mixDensityKgM3,
			p.document.recipe.core.maximumTemperatureC, p.cores.length > 0
			|| fabrication == null
			|| fabrication.adhesiveContainsEPS != false);
		for (check in pour.validation)
			if (check.rule == "process.pour.cure-mass-basis" || check.rule == "process.pour.cure-sequence")
				cure.validation.push(check);
		cure.validation.push(resinIdentity(p.document.recipe.id,spec,fabrication));
		var sectionAssessment = OrthogonalSections.assessRegions(geometry, sourceFingerprint);
		if (sectionAssessment.report == null) throw "Unsupported source section geometry";
		var sections:OrthogonalSections.OrthogonalSectionReport = cast sectionAssessment.report;
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
		for (check in pour.validation) {
			var existing = validation.filter(v -> v.rule == check.rule);
			if (existing.length == 0) validation.push(check);
			else if (existing.length != 1 || ToolingCodec.canonical(existing[0]) != ToolingCodec.canonical(check))
				throw "Inconsistent cure/pour validation: " + check.rule;
		}
		for (check in castability.validation)
			if (validation.filter(v -> v.rule == check.rule).length == 0)
				validation.push(check);
		validation.sort((a, b) -> Reflect.compare(a.rule, b.rule));
		return {
			version: ProcessSpecCodec.VERSION,
			sourceFingerprint: sourceFingerprint,
			processFingerprint: fingerprint,
			spec: spec,
			fill: fill,
			pour: pour,
			cure: cure,
			sections: sections,
			castability: castability,
			thermalCoverage: thermalCoverage,
			validation: validation,
			fabrication: materia.tooling.ToolingValidation.ToolingReadiness.evaluate(validation)
		};
	}
	public static function fingerprint(p:CastingMoldPlan,raw:CastingProcessSpec,gravityMps2:Float,
		?fabrication:materia.tooling.export.FabricationSpec):String {
		if (!Math.isFinite(gravityMps2) || gravityMps2<=0) throw "Invalid process gravity";
		var spec=ProcessSpecCodec.snapshot(raw);
		if (fabrication!=null) fabrication=materia.tooling.export.FabricationSpecCodec.snapshot(fabrication);
		var sourceFingerprint=CastingMoldCodec.fingerprint(p);
		return haxe.crypto.Sha256.encode(ToolingCodec.canonical({
			source: sourceFingerprint,
			spec: spec,
			gravityMps2: gravityMps2,
			fabrication: fabrication == null ? null : {resinReference: fabrication.resinReference, adhesiveContainsEPS: fabrication.adhesiveContainsEPS}
		}));
	}

	static function planPour(p:CastingMoldPlan,spec:CastingProcessSpec,fill:FillReport):PourPlanning.PourAssessment {
		var pour = PourPlanning.analyze(p.document.input.id, fill, spec.mixComposition, spec.mixer);
		pour.validation.push(PourPlanning.cureConsistency(p.document.input.id, spec.mixComposition,
			spec.cure.reference, spec.cure.resinMassFraction));
		var schedule = pour.schedule;
		if (schedule != null && schedule.batches.length > 1)
			pour.validation.push(FillAnalysis.check("process.pour.cure-sequence", [p.document.input.id],
				materia.tooling.ToolingDiagnostic.ToolingCheckStatus.Pending,
				"The uniform full-casting cure model does not cover this batch arrival history; thermal/gel behaviour requires sequence-bound qualification.",
				[pour.mixingFingerprint]));
		return pour;
	}

	static function resinIdentity(recipeId:String,spec:CastingProcessSpec,
		fabrication:Null<materia.tooling.export.FabricationSpec>):materia.tooling.ToolingValidation {
		var status=fabrication==null ? materia.tooling.ToolingDiagnostic.ToolingCheckStatus.Pending
			: fabrication.resinReference!=spec.cure.reference ? materia.tooling.ToolingDiagnostic.ToolingCheckStatus.Fail
			: materia.tooling.ToolingDiagnostic.ToolingCheckStatus.Pass;
		return FillAnalysis.check("process.cure.resin-identity", [recipeId],status,
			"Cure reference must name the same resin as the fabrication solvent record.",
			fabrication == null ? [] : [fabrication.resinReference, spec.cure.reference]);
	}

}
