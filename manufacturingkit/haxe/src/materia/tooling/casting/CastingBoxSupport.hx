package materia.tooling.casting;

import materia.casting.CastingCavities;
import materia.tooling.ToolingValidation;
import materia.tooling.process.FillAnalysis;

/** Box-only consumers qualify the exact derived cavities before projecting any region. */
class CastingBoxSupport {
	public static function check(input:CastingToolingInput, rule:String, includeEmbeddedHardware:Bool = true):Array<ToolingValidation> {
		var checks:Array<ToolingValidation> = [];
		if (includeEmbeddedHardware) for (insert in input.inserts) switch insert.anchorage {
			case Bolted(_,_):checks.push(FillAnalysis.check(rule+".anchors."+insert.id,[insert.id],Unsupported,
				"Unsupported non-box anchor occupancy: "+insert.id+"; use exact native regions, never box substitutes",[]));
			default:
		}
		for (cavity in CastingCavities.derive(input.envelope, input.features))
			if (!CastingCavities.supportsBoxes(cavity))
				checks.push(FillAnalysis.check(rule + "." + cavity.featureId, [cavity.featureId], Unsupported,
					"Unsupported non-box finished cavity: " + cavity.featureId, []));
		if (checks.length == 0 && !CastingCavities.supportsBoxes({featureId: input.id, parentId: input.id, geometry: materia.casting.CastingCavities.FinishedCavityGeometry.Prisms(input.envelope)}))
			checks.push(FillAnalysis.check(rule + "." + input.id, [input.id], Unsupported,
				"Unsupported non-box casting envelope: " + input.id, []));
		if (checks.length == 0)
			checks.push(FillAnalysis.check(rule, [input.id], Pass, "Exact axis-aligned box regions are supported", []));
		return checks;
	}
}
