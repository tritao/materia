package machinekit.tooling;

import materia.tooling.ToolingCodec;
import materia.tooling.ToolingEnums.ToolingLengthUnit;
import materia.tooling.casting.CastingToolingInput;
import materia.tooling.casting.CastingToolingInput.CastingToolingInsert;
import machinekit.structural.MineralFoundation;
import machinekit.structural.MineralFoundation.CastInsertPlacement;

/** Detached copy of source design records and the authoritative hole facet. */
class FoundationToolingAdapter {
	public static function snapshot(foundation:MineralFoundation, id:String, revision:Int):CastingToolingInput {
		foundation.body.finishedCavities();
		return ToolingCodec.inputSnapshot({
			id: id,
			revision: revision,
			lengthUnit: ToolingLengthUnit.Millimetres,
			envelope: foundation.body.castingEnvelope(),
			features: foundation.body.castingFeatures(),
			inserts: [for (row in foundation.insertLocations()) insert(row)]
		});
	}

	static function insert(row:CastInsertPlacement):CastingToolingInsert {
		var source = row.part.insertSpec();
		return {
			id: row.id,
			body: source.body,
			material: row.part.insertMaterial(),
			holes: source.holes,
			origin: source.origin,
			pose: row.pose,
			role: cast row.part.parameterToken("role"),
			label: row.part.parameterToken("label"),
			positionTolerance: row.part.parameterNumber("positionTolerance"),
			finish: cast row.part.parameterToken("finish")
		};
	}
}
