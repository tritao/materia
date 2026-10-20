package materia.tooling.casting;

import materia.assembly.AssemblyFrames as F;
import materia.assembly.AssemblyRecord.AssemblyFrame;
import materia.casting.InsertHole;
import materia.casting.InsertHole.InsertHoleCodec;
import materia.tooling.ToolingRecipe.ToolingHardware;

/** Both mold planners use the same source-thread access and axial depth budget. */
class InsertFixings {
	public static function protectionValidation(insert:materia.tooling.casting.CastingToolingInput.CastingToolingInsert):Array<materia.tooling.ToolingValidation> {
		var result:Array<materia.tooling.ToolingValidation> = [];
		for (hole in insert.holes) {
			var thread = hole.thread;
			if (thread == null)
				continue;
			var reason = hole.counterbore != null ? "A separate counterbore sealing cap must be specified."
				: thread.profile != Metric || thread.hand != Right || thread.starts != 1
					? "Select protection hardware for this non-standard set-screw thread." : null;
			if (reason != null)
				result.push({rule: "tooling.fixture.hole-protection." + insert.id + "." + hole.id,
					affectedIds: [insert.id, hole.id], status: materia.tooling.ToolingDiagnostic.ToolingCheckStatus.Unsupported,
					severity: materia.tooling.ToolingDiagnostic.ToolingSeverity.Error, message: reason, evidence: []});
		}
		return result;
	}

	public static function mask(hole:InsertHole, id:String, plugLength:Float,
			withdrawal:{x:Float, y:Float, z:Float}):materia.tooling.casting.CastingMoldPlan.BoreMaskProtection {
		var threaded = hole.thread != null;
		if (threaded && plugLength > (cast hole.threadDepth:Float))
			throw "Threaded plug exceeds usable source thread: " + hole.id;
		if (threaded && plugLength * (hole.extent == Through ? 2 : 1) + InsertHoleCodec.threadLimits(hole).minTipGap > hole.depth + 1e-7)
			throw "Thread protection exceeds source hole depth budget: " + hole.id;
		return {sourceHoleId: hole.id, hardwareId: id, plugLength: threaded ? plugLength : null,
			farEndId: threaded && hole.extent == Through ? materia.tooling.ToolingIds.derive("far-end-", id) : null,
			withdrawal: withdrawal};
	}

	public static function select(hole:InsertHole, insertToMold:AssemblyFrame, supportFace:AssemblyFrame,
			hardware:ToolingHardware):Null<{
		entry:AssemblyFrame,
		engagement:Float,
		clearanceDiameter:Float,
		plug:Null<{diameter:Float, length:Float, materialSpec:String}>
	}> {
		var entry = F.compose(insertToMold, hole.frame), axis = F.transformVector(entry, 0, 0, 1);
		if (hole.thread == null || axis.z < 1 - 1e-8 || Math.abs(entry.z - supportFace.z) > 1e-7)
			return null;
		var thread:InsertThread = cast hole.thread;
		if (hole.extent == Through && hardware.plugMaterial != "steel")
			throw "Unsupported threaded fixing plug material: " + hardware.plugMaterial;
		var plug = hole.extent == Through ? {
			diameter: thread.majorDiameter, length: hardware.plugDepth, materialSpec: hardware.plugMaterial
		} : null;
		var limits = InsertHoleCodec.threadLimits(hole);
		var occupied = limits.minEngagement + limits.minTipGap + (plug == null ? 0 : plug.length);
		if (occupied > hole.depth + 1e-7
			|| limits.minEngagement + limits.minTipGap > (cast hole.threadDepth:Float) + 1e-7)
			return null;
		return {entry: entry, engagement: limits.minEngagement,
			clearanceDiameter: hole.clearanceDiameter, plug: plug};
	}
}
