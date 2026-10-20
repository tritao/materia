package machinekit.tooling;

import materia.tooling.ToolingCodec;
import materia.tooling.ToolingEnums.ToolingLengthUnit;
import materia.tooling.casting.CastingToolingInput;
import machinekit.structural.MineralFoundation;
import machinekit.structural.InsertHoles;
import machinekit.design.DesignInterfaces;

/** Detached copy of source design records and the authoritative hole facet. */
class FoundationToolingAdapter {
    public static function snapshot(foundation:MineralFoundation,id:String,revision:Int):CastingToolingInput {
        foundation.body.finishedCavities();
        return ToolingCodec.inputSnapshot({id:id,revision:revision,lengthUnit:ToolingLengthUnit.Millimetres,
            envelope:foundation.body.castingEnvelope(),features:foundation.body.castingFeatures(),
            inserts:[for(row in foundation.insertLocations()) {
                id:row.id,width:row.part.parameterNumber("width"),depth:row.part.parameterNumber("depth"),height:row.part.parameterNumber("height"),
                material:row.part.insertMaterial(),holes:InsertHoles.copy(DesignInterfaces.of(row.part)),pose:row.pose,role:cast row.part.parameterToken("role"),
                label:row.part.parameterToken("label"),positionTolerance:row.part.parameterNumber("positionTolerance"),finish:cast row.part.parameterToken("finish")
            }]});
    }
}
