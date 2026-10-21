package machinekit.structural;

import machinekit.assembly.MachineAssembly;
import machinekit.assembly.Diagnostics;
import materia.assembly.AssemblyFrames as F;

/** Geometry/tolerance checks only. No structural, modal or process adequacy claim. */
class FoundationIntegrity {
	public static function check(foundation:MineralFoundation, ?measured:Map<String, materia.assembly.AssemblyRecord.AssemblyFrame>):Diagnostics {
		var out = new Diagnostics(),
			poses = measured == null ? foundation.solvedPoses() : measured,
			bodyPose = poses.get("casting");
		if (bodyPose == null)
			throw "missing casting pose";
		for (row in foundation.insertLocations()) {
			var actual = poses.get(row.id),
				expected = F.compose(bodyPose, row.pose);
			if (actual == null)
				throw "missing insert pose";
			var part = foundation.components().filter(m -> m.id == row.id)[0].component;
			var insert:CastInsert = cast part,
				tolerance = insert.parameterNumber("positionTolerance");
			var distance = Math.sqrt(Math.pow(actual.x - expected.x, 2) + Math.pow(actual.y - expected.y, 2) + Math.pow(actual.z - expected.z, 2));
			var normal = F.transformVector(actual, 0, 0, 1),
				nominal = F.transformVector(expected, 0, 0, 1);
			var tilt = Math.sqrt(Math.pow(normal.x - nominal.x, 2) + Math.pow(normal.y - nominal.y, 2) + Math.pow(normal.z - nominal.z, 2));
			var faceSpan = switch materia.casting.InsertSpec.InsertBodyCodec.mouth(row.part.bodySpec()) {
				case Box(width, depth, _): Math.max(width, depth);
				case Cylinder(diameter, _): diameter;
			};
			if (distance + tilt * faceSpan > tolerance)
				out.error("foundation.insert-plane", row.id, 'Insert "${row.id}" is outside its position/face-parallelism tolerance');
		}
		return out;
	}

	/** Source-bound level-1 casting checks enter the same design Diagnostics.
	 * Capture context independently from the owning assembly, not the document.
	 * Standalone foundations can omit it.
	 */
	public static function castingReview(foundation:MineralFoundation, document:materia.tooling.ToolingDocument,
			?context:materia.casting.CastingInterfaceContext):{
		diagnostics:Diagnostics,
		metrics:materia.tooling.process.CastingLevelOne.CastingDesignMetrics,
		validation:Array<materia.tooling.ToolingValidation>,
		proposals:Array<materia.tooling.process.CastabilityAnalysis.CastabilityProposal>,
		failures:Int,
		pending:Int,
		unsupported:Int
	} {
		var input = machinekit.tooling.FoundationToolingAdapter.snapshot(foundation, document.input.id, document.input.revision, context);
		if (materia.tooling.ToolingCodec.inputFingerprint(input) != materia.tooling.ToolingCodec.inputFingerprint(document.input))
			throw "Foundation casting review source differs from document";
		var review = materia.tooling.process.CastingLevelOne.analyze(document);
		review.validation=review.validation.concat(machinekit.tooling.InsertAnchorGeometry.review(input));
		if (foundation.body.collisionHullQualification.status == Unsupported)
			review.validation.push(materia.tooling.process.FillAnalysis.check("casting.collision-hull.unsupported." + input.id,
				[input.id], materia.tooling.ToolingDiagnostic.ToolingCheckStatus.Unsupported,
				foundation.body.collisionHullQualification.describe(), []));
		return {
			metrics: review.metrics,
			diagnostics: machinekit.tooling.ToolingDiagnostics.fromValidation(review.validation),
			validation: review.validation,
			proposals: review.proposals,
			failures: review.validation.filter(v -> v.status == "fail").length,
			pending: review.validation.filter(v -> v.status == "pending").length,
			unsupported: review.validation.filter(v -> v.status == "unsupported").length
		};
	}

	/** Named steel supports; a supplied connection map also checks the actual assembly mates. */
	public static function checkMounts(assembly:MachineAssembly, mounts:Array<{name:String, parent:String}>,
			?jointIds:Map<String, String>):Diagnostics {
		var out = new Diagnostics(), members = assembly.components(), joints = assembly.definition().joints;
		for (m in mounts) {
			var found = members.filter(p -> p.id == m.parent), connections = jointIds == null ? [] : joints.filter(j -> j.id == jointIds.get(m.name));
			if (found.length != 1 || !Std.isOfType(found[0].component, CastInsert)
				|| (jointIds != null && (connections.length != 1 || connections[0].parent != m.parent)))
				out.error("foundation.precision-mount", m.name, 'Precision mount "${m.name}" must land on a cast-in steel insert, got "${m.parent}"');
		}
		for (joint in joints) {
			var parents = members.filter(part -> part.id == joint.parent), children = members.filter(part -> part.id == joint.child);
			if (parents.length == 1 && children.length == 1 && Std.isOfType(parents[0].component, MineralCasting)
				&& (Std.isOfType(children[0].component, machinekit.motion.LinearRail)
					|| Std.isOfType(children[0].component, machinekit.hobbing.TableHousing)))
				out.error("foundation.precision-mount", joint.id, "Precision mount attaches directly to the casting body");
		}
		return out;
	}
}
