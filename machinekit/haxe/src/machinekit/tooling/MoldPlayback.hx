package machinekit.tooling;

import materia.assembly.AssemblyFrames as F;
import materia.assembly.AssemblyRecord.AssemblyFrame;
import materia.tooling.ToolingCodec;
import materia.tooling.ToolingDiagnostic.ToolingCheckStatus;
import materia.tooling.casting.CastingMoldPlan;
import materia.tooling.casting.CastingMoldCodec;
import materia.tooling.casting.CoreSupport.SupportHardwareKind;
import machinekit.tooling.CastingMoldCad.CastingMoldReview;
import machinekit.tooling.CastingMoldCad.DemoldingStepKind;
import machinekit.assembly.AssemblyPreview;
import machinekit.assembly.PosedParts;
import machinekit.component.Solids;
import materia.project.SceneArtifact;
import materia.project.SceneArtifact.SceneArtifactPart;
import cadkit.modeling.Part;

enum abstract MoldPlaybackPhase(String) to String {
	var Assembly = "assembly";
	var Demolding = "demolding";
}

enum abstract MoldPlaybackAction(String) to String {
	var Install = "install";
	var FitLooseHardware = "fit-loose-hardware";
	var ReleaseLooseHardware = "release-hardware";
	var WithdrawMember = "withdraw";
	var ReorientCasting = "flip-to-service";
}

typedef MoldPlaybackFrame = {
	var phase:MoldPlaybackPhase;
	var stepId:String;
	var action:MoldPlaybackAction;
	var activeMemberIds:Array<String>;
	var visibleMemberIds:Array<String>;
	var castingPose:AssemblyFrame;
	var status:ToolingCheckStatus;
}

/** Static snapshots of checked steps, never a new motion/adhesion/flip simulation. */
class MoldPlayback {
	public static function frames(p:CastingMoldPlan, review:CastingMoldReview, memberIds:Array<String>):Array<MoldPlaybackFrame> {
		if (review.fingerprint != CastingMoldCodec.fingerprint(p))
			throw "Stale playback review";
		var ids = memberIds.copy();
		ids.sort(Reflect.compare);
		var groups = new Map<String, Array<String>>();
		for (group in review.permanentGroups.concat(review.assemblyGroups)) {
			if (groups.exists(group.id))
				throw "Duplicate playback group";
			groups.set(group.id, group.members.copy());
		}
		var hardware = [for (h in p.hardware) h.id];
		function expand(id:String, path:Array<String>):Array<String> {
			if (path.indexOf(id) >= 0)
				throw "Cyclic playback group: " + path.concat([id]).join(" -> ");
			if (id == "cured-casting")
				return [id];
			var group = groups.get(id), out:Array<String> = [];
			if (group != null) {
				for (child in group)
					for (part in expand(child, path.concat([id])))
						if (out.indexOf(part) < 0)
							out.push(part);
			} else if (hardware.indexOf(id) >= 0) {
				for (part in ids)
					if (part == id || StringTools.startsWith(part, id + "-"))
						out.push(part);
			} else if (ids.indexOf(id) >= 0)
				out.push(id);
			if (out.length == 0)
				throw "Unknown playback member: " + id;
			out.sort(Reflect.compare);
			return out;
		}
		var visible = new Map<String, Bool>(),
			result:Array<MoldPlaybackFrame> = [];
		var pose = F.identity();
		function snapshot(phase:MoldPlaybackPhase, step:String, action:MoldPlaybackAction, active:Array<String>, status:ToolingCheckStatus):Void {
			var present = [for (id in visible.keys()) id];
			present.sort(Reflect.compare);
			active = active.copy();
			active.sort(Reflect.compare);
			result.push({
				phase: phase,
				stepId: step,
				action: action,
				activeMemberIds: active,
				visibleMemberIds: present,
				castingPose: F.compose(F.identity(), pose),
				status: status
			});
		}
		for (id in review.assembly.assemblyOrder) {
			var active = expand(id, []);
			for (part in active) {
				if (visible.exists(part))
					throw "Assembly playback installs a member twice: " + part;
				visible.set(part, true);
			}
			snapshot(MoldPlaybackPhase.Assembly, id, MoldPlaybackAction.Install, active, ToolingCheckStatus.Pass);
		}
		var loose:Array<String> = [];
		for (h in p.hardware)
			if (CastingMoldCad.looseHardware(p, h))
				for (part in expand(h.id, []))
					if (loose.indexOf(part) < 0)
						loose.push(part);
		for (id in loose)
			visible.set(id, true);
		for (id in ids)
			if (!visible.exists(id))
				throw "Assembly playback has no installation step for " + id;
		snapshot(MoldPlaybackPhase.Assembly, "fit-loose-tooling", MoldPlaybackAction.FitLooseHardware, loose, ToolingCheckStatus.Pending);
		// Steel is now represented by the single cured resin/steel solid.
		for (i in p.document.input.inserts)
			visible.remove(materia.tooling.ToolingIds.derive("insert-", i.id));
		for (hardware in CastingMoldCad.anchorComponents(p))
			visible.remove(hardware.id);
		visible.set("cured-casting", true);
		for (step in review.demoldingSteps) {
			var active:Array<String> = [];
			for (id in step.memberIds)
				for (part in expand(id, []))
					if (active.indexOf(part) < 0)
						active.push(part);
			if (step.kind == DemoldingStepKind.FlipToService)
				pose = F.compose(pose, p.handling.moldToService);
			else if (step.kind == DemoldingStepKind.Withdraw && active.indexOf("cured-casting") >= 0) {
				var motion = review.demoldingMotions.filter(m -> m.id == "cured-casting");
				if (motion.length != 1)
					throw "Missing cured-casting playback motion";
				var v = motion[0], travel = review.demolding.travelMm;
				pose = F.translation(v.x * travel, v.y * travel, v.z * travel);
			} else
				for (id in active) {
					if (!visible.exists(id))
						throw "Demolding playback withdraws a missing member: " + id;
					visible.remove(id);
				}
			var action:MoldPlaybackAction = switch (step.kind) {
				case DemoldingStepKind.ReleaseHardware: MoldPlaybackAction.ReleaseLooseHardware;
				case DemoldingStepKind.Withdraw: MoldPlaybackAction.WithdrawMember;
				case DemoldingStepKind.FlipToService: MoldPlaybackAction.ReorientCasting;
				default: throw "Unsupported demolding playback step";
			};
			snapshot(MoldPlaybackPhase.Demolding, step.id, action, active, step.status);
		}
		return result;
	}

	static function cured(p:CastingMoldPlan, pose:AssemblyFrame):Part {
		var owned:Array<Part> = [];
		return Solids.building(owned, parts -> {
			parts.push(CastingMoldCad.source(p));
			for (i in p.document.input.inserts)
				parts.push(CastingMoldCad.steel(p, i.id));
			var body = Solids.union(parts);
			parts.push(body);
			var world = PosedParts.moved(body, pose);
			body.close();
			return world;
		});
	}

	/** Static presentation scenes have no collision proxies or dynamics. Exact validation
	 * lives in the fingerprint-bound CAD review, not approximate viewer boxes.
	 */
	public static function scene(parts:Array<SceneArtifactPart>, id:String):materia.project.SceneArtifact.SceneArtifactData {
		return {
			lengthUnit: "mm",
			metresPerUnit: 0.001,
			parts: parts,
			assemblyDefinition: {
				schemaVersion: materia.assembly.AssemblyDefinitionCodec.VERSION,
				id: id,
				lengthUnit: "mm",
				definitions: [for (part in parts) {id: part.id, connectors: [], collisionHulls: []}],
				occurrences: [
					for (part in parts)
						{
							id: part.id,
							definition: part.id,
							initialPose: F.identity(),
							grounded: true
						}
				],
				joints: []
			}
		};
	}

	public static function write(p:CastingMoldPlan, review:CastingMoldReview, dir:String):Array<MoldPlaybackFrame> {
		if (review.fingerprint != CastingMoldCodec.fingerprint(p))
			throw "Stale playback review";
		var assembly = CastingMoldCad.assembly(p),
			members = assembly.components(),
			poses = assembly.solvedPoses();
		var sequence = frames(p, review, [for (m in members) m.id]),
			cache = new Map<String, SceneArtifactPart>();
		for (m in members) {
			var local = m.component.geometry(), world:Part;
			try
				world = PosedParts.moved(local, poses.get(m.id))
			catch (error:Dynamic) {
				local.close();
				throw error;
			}
			local.close();
			cache.set(m.id, AssemblyPreview.part(m.id, m.component.designation, world, m.component.materialId));
		}
		var castCache = new Map<String, SceneArtifactPart>(),
			folder = dir + "/sequence";
		ToolingFiles.directory(folder);
		var records:Array<Dynamic> = [], index = 0;
		var instructions = [
			"# Assembly and demolding playback",
			"",
			"Visual-only snapshots of checked orders. Collision proxies/dynamics are disabled; exact collision and removal evidence is in review.json. These scenes do not simulate glue release, EPS breakup, handling or flip trajectories.",
			"Loose fixing installation/release and lifting qualifications retain their pending status. Casting withdrawal is shown at the checked clearance distance.",
			""
		];
		for (frame in sequence) {
			var parts:Array<SceneArtifactPart> = [];
			for (id in frame.visibleMemberIds) {
				if (id != "cured-casting") {
					parts.push(cache.get(id));
					continue;
				}
				var key = ToolingCodec.canonical(frame.castingPose),
					part = castCache.get(key);
				if (part == null) {
					var center = p.handling.centerOfMassMoldMm;
					var worldCenter = F.transformPoint(frame.castingPose, center.x, center.y, center.z);
					var mass = new machinekit.component.MassProperties(p.handling.castingMassKg,
						new cadkit.modeling.Vector(worldCenter.x, worldCenter.y, worldCenter.z), machinekit.component.MassProperties.MassSource.Declared);
					part = AssemblyPreview.part(id, "Cured casting and embedded steel", cured(p, frame.castingPose), "mineral-cast", null, mass);
					// Composite inertia is not inferred from uniform-density display geometry.
					part.inertia = null;
					part.physicalInertiaKgM2 = null;
					castCache.set(key, part);
				}
				parts.push(part);
			}
			var filename = (frame.phase : String) + "-" + StringTools.lpad(Std.string(++index), "0", 3) + ".mtrg";
			sys.io.File.saveBytes(folder + "/" + filename, SceneArtifact.encode(scene(parts, "playback-" + index)));
			records.push({file: "sequence/" + filename, frame: frame});
			instructions.push("- [" + filename + "](" + filename + "): " + frame.stepId + " (" + frame.status + ")");
		}
		sys.io.File.saveContent(dir + "/sequence-playback.json", ToolingCodec.canonical({
			version: 1,
			sourceFingerprint: review.fingerprint,
			limitation: "Static checked-order snapshots. Loose hardware, bond release, EPS breakup and lifting/flip qualification remain manual/pending. No new trajectory simulation.",
			frames: records
		}) + "\n");
		sys.io.File.saveContent(folder + "/README.md", instructions.join("\n") + "\n");
		return sequence;
	}
}
