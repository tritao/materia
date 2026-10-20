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
		return new CastInsert(new ComponentValues().setToken("body",InsertBodyCodec.encode(insert.body))
			.setToken("origin",haxe.Json.stringify(InsertOriginCodec.record(insert.origin)))
			.setToken("holes",InsertHoles.fromNeutral(insert.holes)).setToken("stockMaterial",insert.material.grade)
			.setToken("role",insert.role).setToken("label",insert.label).setToken("finish",insert.finish)
			.setNumber("positionTolerance",insert.positionTolerance));
}
