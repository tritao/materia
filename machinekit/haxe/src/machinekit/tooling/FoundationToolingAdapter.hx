package machinekit.tooling;

import materia.tooling.ToolingCodec;
import materia.tooling.ToolingEnums.ToolingLengthUnit;
import materia.tooling.casting.CastingToolingInput;
import materia.tooling.casting.CastingToolingInput.CastingToolingInsert;
import machinekit.structural.MineralFoundation;
import machinekit.structural.MineralFoundation.CastInsertPlacement;

/** Detached copy of source design records and the authoritative hole facet. */
class FoundationToolingAdapter {
	public static function snapshot(foundation:MineralFoundation, id:String, revision:Int,
			?context:materia.casting.CastingInterfaceContext):CastingToolingInput {
		foundation.body.finishedCavities();
		var expected:Array<materia.casting.EmbeddedAnchors.EmbeddedInsertAnchors> = [];
		for (row in foundation.insertLocations()) {
			var spec=row.part.insertSpec();
			switch spec.anchorage {case Bolted(_,_):expected.push({owner:row.id,pose:row.pose,spec:spec});default:}
		}
		if (ToolingCodec.canonical(materia.casting.EmbeddedAnchors.records(expected))
			!=ToolingCodec.canonical(materia.casting.EmbeddedAnchors.records(foundation.body.embeddedAnchors())))
			throw "Foundation embedded anchor occupancy differs from source inserts";
		return ToolingCodec.inputSnapshot({
			id: id,
			revision: revision,
			lengthUnit: ToolingLengthUnit.Millimetres,
			datumScheme: foundation.body.foundationDatumScheme(),
			interfaceContext: context,
			interfaces: foundation.body.precisionRequirements(),
			envelope: foundation.body.castingEnvelope(),
			features: foundation.body.castingFeatures(),
			inserts: [for (row in foundation.insertLocations()) insert(row)]
		});
	}

	static function insert(row:CastInsertPlacement):CastingToolingInsert {
		if (row.part.buildStage()!=materia.casting.InsertStages.InsertStage.Service)
			throw "Foundation design requires a service-stage insert: "+row.id;
		var source = row.part.insertSpec();
		return {
			id: row.id,
			body: source.body,
			material: row.part.insertMaterial(),
			holes: source.holes,
			origin: source.origin,
			anchorage:source.anchorage,
			interfaces: source.interfaces,
			pose: row.pose,
			role: cast row.part.parameterToken("role"),
			label: row.part.parameterToken("label"),
			positionTolerance: row.part.parameterNumber("positionTolerance"),
			finish: cast row.part.parameterToken("finish")
		};
	}
}
