package materia.tooling.process;

import materia.tooling.ToolingCodec;
import materia.tooling.casting.CastingMoldPlan;
import materia.tooling.casting.CastingMoldPlanner;
import materia.tooling.casting.CastingMoldCodec;
import materia.tooling.casting.BoxVolumes;
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
	var validation:Array<materia.tooling.ToolingValidation>;
	var fabrication:materia.tooling.ToolingValidation.FabricationReadiness;
}

/** Process checks extend the real mold planner's validations. They never clear
 * its requests or substitute a separate geometry/readiness decision.
 */
class CastingProcessAnalysis {
	public static function analyze(p:CastingMoldPlan, raw:CastingProcessSpec, gravityMps2:Float):CastingProcessReport {
		CastingMoldPlanner.validate(p);
		var spec = ProcessSpecCodec.snapshot(raw);
		if (spec.recipeFingerprint != ToolingCodec.recipeFingerprint(p.document.recipe))
			throw "Stale process recipe fingerprint";
		var sourceFingerprint = CastingMoldCodec.fingerprint(p);
		var fingerprint = haxe.crypto.Sha256.encode(ToolingCodec.canonical({source: sourceFingerprint, spec: spec, gravityMps2: gravityMps2}));
		var features = p.document.input.features.copy();
		features.sort((a, b) -> Reflect.compare(a.id, b.id));
		var boundaries:Array<FillBoundary> = [
			for (f in features)
				{id: f.id, box: CastingMoldPlanner.featureBox(p.document, f)}
		];
		var resin = BoxVolumes.subtract([p.envelope], [for (b in boundaries) b.box]);
		var fill = FillAnalysis.analyze(p.document.input.id, sourceFingerprint, fingerprint, resin, boundaries,
			[for (c in p.cores) {id: c.id, boxes: [c.cavity]}], p.document.setup.fillHeightMm, p.document.recipe.mixDensityKgM3, gravityMps2,
			spec.pourRateMm3PerSecond, spec.maximumAirEscapeWidthMm);
		var knownIds = [p.document.input.id];
		for (f in p.document.input.features)
			knownIds.push(f.id);
		for (i in p.document.input.inserts)
			knownIds.push(i.id);
		for (s in spec.sections)
			for (id in s.sourceIds)
				if (knownIds.indexOf(id) < 0)
					throw "Unknown cure section source: " + id;
		var cure = CureAnalysis.analyze(p.document.input.id, sourceFingerprint, fingerprint, spec, p.document.recipe.mixDensityKgM3,
			p.document.recipe.core.maximumTemperatureC, p.cores.length > 0);
		var validation = p.validation.concat(fill.validation).concat(cure.validation);
		validation.sort((a, b) -> Reflect.compare(a.rule, b.rule));
		return {
			version: 1,
			sourceFingerprint: sourceFingerprint,
			processFingerprint: fingerprint,
			spec: spec,
			fill: fill,
			cure: cure,
			validation: validation,
			fabrication: materia.tooling.ToolingValidation.ToolingReadiness.evaluate(validation)
		};
	}
}
