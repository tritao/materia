package machinekit.tooling;

import machinekit.structural.CastInsert;
import machinekit.structural.InsertHoles;
import machinekit.component.ComponentValues;
import materia.casting.InsertSpec.InsertBodyCodec;
import materia.casting.InsertOrigin.InsertOriginCodec;
import materia.tooling.casting.CastingToolingInput.CastingToolingInsert;

/** A copy back into the common source component: no inferred holes, roles or dimensions. */
class ToolingInsertCad {
	public static function component(insert:CastingToolingInsert):CastInsert
		return staged(insert,materia.casting.InsertStages.InsertStage.BeforeCasting);

	public static function serviceComponent(insert:CastingToolingInsert):CastInsert
			return staged(insert,materia.casting.InsertStages.InsertStage.Service);

	static function staged(insert:CastingToolingInsert,stage:materia.casting.InsertStages.InsertStage):CastInsert
		return new CastInsert(new ComponentValues().setToken("buildStage",stage).setToken("body",InsertBodyCodec.encode(insert.body))
			.setToken("interfaces",haxe.Json.stringify(materia.casting.InterfaceRequirement.InterfaceRequirements.records(insert.interfaces)))
			.setToken("origin",haxe.Json.stringify(InsertOriginCodec.record(insert.origin)))
			.setToken("anchorage",haxe.Json.stringify(materia.casting.InsertAnchorage.InsertAnchorageCodec.record(insert.anchorage)))
			.setToken("holes",InsertHoles.fromNeutral(insert.holes)).setToken("stockMaterial",insert.material.grade)
			.setToken("role",insert.role).setToken("label",insert.label).setToken("finish",insert.finish)
			.setNumber("positionTolerance",insert.positionTolerance));
}
