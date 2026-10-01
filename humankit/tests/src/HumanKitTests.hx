import animkit.AnimationAsset;
import humankit.CapsulePlacement;
import humankit.HumanBodyProxy;
import humankit.HumanBody;
import humankit.HumanJob;
import humankit.HumanJobSpec;
import humankit.HumanJobBuilder;
import humankit.HumanCarryPosture;
import humankit.ApproachFor;
import humankit.WalkTo;
import humankit.Reach;
import humankit.ReleaseLimb;
import humankit.Pick;
import humankit.Place;
import humankit.Carry;
import humankit.Press;
import humankit.Wait;
import humankit.PlayClip;
import humankit.HumanBodyView;
import humankit.HumanBone;
import humankit.HumanHand;
import humankit.HumanTargetBox;
import humankit.HumanCapsule;
import humankit.HumanDescription;
import humankit.HumanDisplay;
import humankit.HumanLimb;
import humankit.HumanReachTask;
import humankit.HumanWalker;
import humankit.HumanPose;
import humankit.Naturalness;
import humankit.HumanPosture;
import humankit.HumanCharacter;
import humankit.HumanoidRig;
import humankit.Mat4;
import humankit.RigMapping;
import humankit.facility.FacilityWalk;
import humankit.facility.FacilityTargets;
import humankit.facility.FacilityJobs;
import materia.automation.facility.Facility;
import materia.automation.facility.FacilityRouter;
import materia.automation.facility.Lane;
import materia.automation.facility.Rack;
import materia.automation.facility.RackSlot;
import materia.automation.facility.RackSlotPose;
import materia.automation.facility.Station;
import materia.automation.facility.Surface;
import materia.automation.facility.Zone;
import nativekit.scene.Scene;
import nativekit.scene.SceneRenderer;
import nativekit.scene.SceneView;
import robotkit.mobile.Footprint;
import robotkit.mobile.Pose2;
import robotkit.navigation.Path;
import sys.FileSystem;
import sys.io.File;

class HumanKitTests {
	static function main():Void {
		var assets = assetDir();
		var worker = AnimationAsset.load(assets + "/quaternius/worker.glb");
		var rig = HumanoidRig.detect(worker);
		if (rig.mapping.name != "quaternius")
			throw 'Expected the Quaternius preset, got ${rig.mapping.name}';
		if (!rig.has(HumanBone.MiddleR) || rig.joint(HumanBone.HandR) != worker.jointIndex("Wrist.R"))
			throw "Worker hand bones did not map";
		if (RigMapping.localName("mixamorig:LeftHand") != "LeftHand")
			throw "Namespace prefixes are not stripped";
		var soldier = AnimationAsset.load(assets + "/kenney/character-soldier.glb");
		var rejected = false;
		try HumanoidRig.detect(soldier) catch (error:String) rejected = true;
		if (!rejected)
			throw "A non-humanoid rig was accepted";

		var scene = Scene.create();
		var human = new HumanCharacter(scene, worker, rig, null, "Worker");
		var height = human.height();
		if (height < 1.7 || height > 1.95)
			throw 'Unexpected worker height $height';
		var head = human.pose.bonePosition(HumanBone.Head);
		var left = human.pose.bonePosition(HumanBone.HandL);
		var right = human.pose.bonePosition(HumanBone.HandR);
		var foot = human.pose.bonePosition(HumanBone.FootL);
		// Characters face +X with +Z up, so their left side is +Y.
		if (head[2] < 1.3 || foot[2] > 0.3 || left[1] <= right[1])
			throw 'Landmarks are not anatomical: head $head left $left right $right foot $foot';
		checkRigid(human.pose.boneFrame(HumanBone.HandR), "right hand frame");
		bodyProxy(human.pose, height);

		var before = drawCalls(scene);
		var wrench = AnimationAsset.load(assets + "/props/wrench.glb");
		var held = human.attach(wrench, HumanBone.HandR, null, "Wrench");
		human.player.playNamed("walk", 0.0);
		for (step in 0...5)
			human.advance(0.1);
		human.publish();
		var snapshot = scene.snapshot();
		var world = snapshot.findNode(held.node).worldTransform();
		var palm = Mat4.position(human.pose.boneFrame(HumanBone.HandR));
		for (axis in 0...3)
			if (Math.abs(world.element(12 + axis) - palm[axis]) > 1e-4)
				throw 'Attachment does not follow the hand: palm $palm';
		if (human.changedNodes().indexOf(held.node) < 0)
			throw "Attachment node is not reported as changed";
		snapshot.dispose();
		var after = drawCalls(scene);
		if (after != before + 1)
			throw 'Attaching a one-mesh prop changed draw calls from $before to $after';
		bodyView(scene, human);
		human.dispose();
		universalCharacter(scene, worker, rig);
		universalLibrary(scene);
		walking(scene, worker, rig);
		reaching(scene, worker, rig);
		elbowStaysPut(scene, worker, rig);
		elbowStaysPutEverywhere(scene, worker, rig);
		fingersCurl(scene, worker, rig);
		leaning(scene, worker, rig);
		grasping(scene, worker, rig);
		facilityRoute(scene, worker, rig);
		facilitySurfaces();
		postureStature();
		turning(scene, worker, rig);
		retreating(scene, worker, rig);
		holdingFeet(scene);
		kneeling(scene);
		footKeepsItsPitch(scene);
		naturalness(scene, worker, rig, "bundled");
		facilityTargets(scene, worker, rig);
		reachTask(scene, worker, rig);
		placeReferencePoint(scene, worker, rig);
		jobs(scene, worker, rig);
		jobSpecs(scene, worker, rig);
		scene.dispose();
		Sys.println("humankit tests: ok");
	}

	/**
	 * Quaternius's Universal Animation Library character: a second rig, with real leg chains and a crouch clip. The
	 * checks that name no joints run on it as they do on the bundled worker; the rest are about what it adds.
	 */
	static function universalCharacter(scene:Scene, bundled:AnimationAsset, bundledRig:HumanoidRig):Void {
		var asset = AnimationAsset.load(assetDir() + "/quaternius-ual/ual-standard.glb");
		var rig = HumanoidRig.detect(asset);
		if (rig.mapping.name != "universal") throw 'The library character matched the ${rig.mapping.name} preset';
		var human = new HumanCharacter(scene, asset, rig, null, "Universal");
		inRange(human.height(), 1.7, 1.95, "library character height");
		// Its fingers curl by the same joint turns, about the axis found on its own skeleton: in the rest pose a
		// light curl brings the middle fingertip toward the palm's face. (Its relaxed idle hand already starts
		// partly curled, so a full curl overshoots there; only the rest pose is held to this.)
		var depth = function():Float {
			var frame = human.pose.boneFrame(HumanBone.HandR), wrist = human.pose.bonePosition(HumanBone.HandR);
			var knuckle = human.pose.bonePosition(HumanBone.MiddleR), tip = human.fingertip(ArmR, HumanHand.MIDDLE);
			var total = 0.0;
			for (axis in 0...3) total -= (tip[axis] - (wrist[axis] + knuckle[axis]) * 0.5) * frame[8 + axis];
			return total;
		};
		human.advance(0.0);
		var open = depth();
		human.setHandCurl(ArmR, 0.3);
		human.advance(0.0);
		var closing = depth();
		if (closing - open < 0.02) throw 'Curling a library hand moved the middle fingertip from $open to $closing m off the palm';
		human.setHandCurl(ArmR, 0.0);
		human.player.play(asset.clipIndex("idle"), 0.0);
		human.advance(0.0);
		// Its legs are chains, so an ankle can be reached, which the bundled worker's cannot.
		var foot = human.pose.bonePosition(HumanBone.FootL);
		var ankle = [foot[0] + 0.15, foot[1], foot[2] + 0.15];
		human.reach(LegL, ankle, 1.0);
		human.advance(0.0);
		if (distance(human.pose.bonePosition(HumanBone.FootL), ankle) > 0.03)
			throw "A library character's ankle did not reach its target";
		human.release(LegL);
		// It crouches: the pelvis comes down steadily and the feet stay on the floor.
		if (!human.canCrouch()) throw "The library character has a crouch clip but cannot crouch";
		var last = 10.0;
		for (amount in [0.0, 0.25, 0.5, 0.75, 1.0]) {
			human.setCrouch(amount);
			human.advance(0.0);
			var pelvis = human.pose.bonePosition(HumanBone.Pelvis)[2];
			if (!(pelvis < last)) throw 'The pelvis did not come down at crouch $amount: $pelvis after $last';
			last = pelvis;
			for (bone in [HumanBone.FootL, HumanBone.FootR])
				if (human.pose.bonePosition(bone)[2] > 0.2) throw 'A foot left the floor at crouch $amount: ${human.pose.bonePosition(bone)[2]}';
		}
		human.setCrouch(0.0);
		human.advance(0.0);
		var standing = human.pose.bonePosition(HumanBone.Pelvis)[2];
		if (standing - last < 0.3) throw 'A full crouch only lowered the pelvis ${standing - last} m';
		// Posed from the going-down clip, a depth is that fraction of the way down in the pelvis too.
		human.setCrouch(0.5);
		human.advance(0.0);
		var halfway = (standing - human.pose.bonePosition(HumanBone.Pelvis)[2]) / (standing - last);
		if (Math.abs(halfway - 0.5) > 0.1) throw 'Half a crouch put the pelvis $halfway of the way down';
		human.setCrouch(0.0);
		human.advance(0.0);
		// The planner crouches for a low surface and not for a shelf; the bundled worker, which cannot, refuses the low one.
		var body = new HumanBody(human);
		var bench:HumanTargetBox = {center: [0.6, 0.0, 0.55], halfExtents: [0.2, 0.2, 0.05], yaw: 0.0};
		var shelf:HumanTargetBox = {center: [0.6, 0.0, 1.15], halfExtents: [0.2, 0.2, 0.05], yaw: 0.0};
		var low = new ApproachFor([0.6, 0.0, 0.62], ArmR, 1.0, false, null, bench);
		var lowJob = new HumanJob(body).add(low);
		var planStart = Sys.time();
		lowJob.advance(1.0 / 60.0);
		Sys.println('PLAN bench approach took ${Math.round((Sys.time() - planStart) * 1000)} ms');
		if (lowJob.failure() != null) throw 'A bench was not reachable crouched: ${lowJob.failure()}';
		var high = new ApproachFor([0.6, 0.0, 1.22], ArmR, 1.0, false, null, shelf);
		var highJob = new HumanJob(body).add(high);
		highJob.advance(1.0 / 60.0);
		if (highJob.failure() != null) throw 'A shelf was not reachable: ${highJob.failure()}';
		if (!(low.crouch >= 0.4) || !(high.crouch <= 0.1))
			throw 'The planner crouched ${low.crouch} for a bench and ${high.crouch} for a shelf';
		// A deep top at table height is no reason to crouch: it lowers the shoulder, not forward over the edge.
		var wide:HumanTargetBox = {center: [0.7, 0.0, 1.01], halfExtents: [0.4, 0.4, 0.05], yaw: 0.0};
		var deep = new ApproachFor([0.7, 0.0, 1.1], ArmR, 1.0, false, null, wide);
		var deepJob = new HumanJob(body).add(deep);
		deepJob.advance(1.0 / 60.0);
		if (!(deep.crouch <= 0.1)) throw 'The planner crouched ${deep.crouch} for a deep top at table height';
		if (deepJob.failure() != null) throw 'A deep top at table height was not reachable: ${deepJob.failure()}';
		if (!(deep.hinge > 0.05)) throw 'The planner did not bend at the hips to reach across a deep top: hinge ${deep.hinge}, lean ${deep.lean}';
		body.cancel();
		human.dispose();
		var plain = new HumanCharacter(scene, bundled, bundledRig, null, "Bundled");
		var plainBody = new HumanBody(plain);
		if (plain.canCrouch() || plainBody.canCrouch()) throw "The bundled worker has no crouch clip but claims to crouch";
		if (plain.canKneel() || plainBody.canKneel()) throw "The bundled worker has no kneeling clip but claims to kneel";
		plainBody.setCrouch(1.0);
		plainBody.advance(0.1);
		if (plainBody.crouchAmount() != 0.0) throw "A body that cannot crouch crouched";
		var refused = new HumanJob(plainBody).add(new ApproachFor([0.6, 0.0, 0.62], ArmR, 1.0, false, null, bench));
		refused.advance(1.0 / 60.0);
		if (refused.failure() == null || refused.failure().indexOf("crouching is unsupported") < 0)
			throw 'A bench was not refused by the worker that cannot crouch: ${refused.failure()}';
		plain.dispose();
		// The checks that name no joints hold on the library character too.
		var again = AnimationAsset.load(assetDir() + "/quaternius-ual/ual-standard.glb");
		var againRig = HumanoidRig.detect(again);
		naturalness(scene, again, againRig, "library", 0.1);
		walking(scene, again, againRig);
		elbowStaysPut(scene, again, againRig);
		leaning(scene, again, againRig);
		facilityRoute(scene, again, againRig);
	}

	/**
	 * The library the runtime loads: the free pack's clips and the extracted ones, each listed in clips.json with
	 * where it came from. Every listed clip must be there at its length and stand on the floor at the height the
	 * free clips do, which is how a mistake in the retargeting (a hip height out by centimetres, a flipped limb)
	 * shows.
	 */
	static function universalLibrary(scene:Scene):Void {
		var directory = assetDir() + "/quaternius-ual";
		var asset = AnimationAsset.load(directory + "/ual-work.glb");
		var rig = HumanoidRig.detect(asset);
		var manifest:Dynamic = haxe.Json.parse(File.getContent(directory + "/clips.json"));
		var clips:Array<Dynamic> = Reflect.field(manifest, "clips");
		var human = new HumanCharacter(scene, asset, rig, null, "Library");
		var extracted = 0;
		var standing = function(name:String):{pelvis:Float, foot:Float} {
			human.player.restart(asset.clipIndex(name), true);
			human.advance(0.0);
			var low = 9.0, pelvis = 0.0;
			for (step in 0...10) {
				human.advance(step == 0 ? 0.0 : asset.clipDurations[asset.clipIndex(name)] / 10.0);
				low = Math.min(low, Math.min(human.pose.bonePosition(HumanBone.FootL)[2], human.pose.bonePosition(HumanBone.FootR)[2]));
				pelvis += human.pose.bonePosition(HumanBone.Pelvis)[2] / 10.0;
			}
			return {pelvis: pelvis, foot: low};
		};
		var reference = standing("Idle_Loop");
		for (clip in clips) {
			var name:String = Reflect.field(clip, "name");
			var index = asset.clipIndex(name);
			if (index < 0) throw 'The library lacks the clip "$name" its manifest lists';
			if (Reflect.field(clip, "file") == null) continue;
			extracted++;
			var length:Float = Reflect.field(clip, "length_seconds");
			if (Math.abs(asset.clipDurations[index] - length) > 0.05)
				throw 'Clip "$name" is ${asset.clipDurations[index]} s, not the $length s it came with';
			if (Reflect.field(clip, "license") != "CC0" || Reflect.field(clip, "source") == null || Reflect.field(clip, "sha256") == null || Reflect.field(clip, "library") == null)
				throw 'Clip "$name" does not say where it came from';
			// The upright clips share a pelvis height with the free ones, and a tired slouch (knees bent) keeps its feet down.
			if (name == "Idle_Tired_Loop" && Math.abs(standing(name).foot - reference.foot) > 0.05) throw 'The tired idle has its feet off the floor';
			if (["Idle_FoldArms_Loop", "Idle_Lantern_Loop", "Idle_TalkingPhone_Loop", "Idle_LookAround_Loop", "Walk_Bwd_Loop", "Walk_L_Loop", "Walk_R_Loop"].indexOf(name) >= 0) {
				var measured = standing(name);
				if (Math.abs(measured.pelvis - reference.pelvis) > 0.06 || Math.abs(measured.foot - reference.foot) > 0.05)
					throw 'Standing clip "$name" has its pelvis at ${measured.pelvis} and a foot at ${measured.foot}, not near ${reference.pelvis} and ${reference.foot}';
			}
		}
		if (extracted < 25) throw 'Only $extracted extracted clips are in the library';
		var carrying = standing("Walk_Carry_Loop");
		if (Math.abs(carrying.foot - reference.foot) > 0.06 || carrying.pelvis < 0.6 || carrying.pelvis > reference.pelvis + 0.05)
			throw 'The carrying walk has its pelvis at ${carrying.pelvis} and a foot at ${carrying.foot}';
		human.dispose();
		// A body that carries walks with the carrying gait, and one that does not with the ordinary walk.
		var again = new HumanCharacter(scene, asset, rig, null, "Carrier");
		var body = new HumanBody(again);
		if (body.walker.carryGait == null || !(body.walker.carryGait.naturalSpeed > 0.5 && body.walker.carryGait.naturalSpeed < 2.5))
			throw "The library character has no usable carrying gait";
		body.setCarry([ArmR]);
		body.walker.follow([[0.0, 0.0], [2.0, 0.0]], 1.0);
		if (again.player.currentClip() != asset.clipIndex("walk_carry")) throw "A body carrying a part did not take the carrying walk";
		body.setCarry([]);
		body.walker.follow([[0.0, 0.0], [2.0, 0.0]], 1.0);
		if (again.player.currentClip() != body.walker.gait.clip) throw "A body carrying nothing did not take the ordinary walk";
		again.dispose();
	}

	static function jobSpecs(scene:Scene, asset:AnimationAsset, rig:HumanoidRig):Void {
		var human = new HumanCharacter(scene, asset, rig, null, "Spec worker");
		var body = new HumanBody(human);
		body.walker.place(0.0, 0.0, 0.0);
		body.advance(0.0);
		var targets = new JobTargets();
		targets.boxes.set("part", {center: [0.8, 0.0, 1.1], halfExtents: [0.1, 0.1, 0.05], yaw: 0.0});
		targets.boxes.set("table", {center: [2.0, 1.0, 0.65], halfExtents: [0.5, 0.4, 0.4], yaw: Math.PI / 2});
		var spec = HumanJobSpec.parse('{"version":1,"loop":false,"steps":[{"action":"pick","object":"part"},{"action":"place","onto":"table","offset":[0.2,0.1]}]}');
		if (HumanJobSpec.parse(spec.toJson()).steps.length != 2) throw "Job JSON round trip failed";
		var built = HumanJobBuilder.build(spec, targets, body);
		if (built.holds.length != 2) throw "Pick/place bindings missing";
		var pick:Pick = cast built.holds[0].action;
		var place:Place = cast built.holds[1].action;
		if (Math.abs(pick.target[2] - 1.16) > 1e-6) throw "Grasp height is wrong";
		if (Math.abs(place.target[0] - 1.9) > 1e-6 || Math.abs(place.target[1] - 1.2) > 1e-6
			|| Math.abs(place.target[2] - 1.1) > 1e-6) throw 'Rotated place point is wrong: ${place.target}';
		var pressSpec = HumanJobSpec.parse('{"version":1,"loop":false,"steps":[{"action":"press","target":{"point":[0.8,0.2,1.35]}}]}');
		var pressJob = HumanJobBuilder.build(pressSpec, targets, body).job;
		var press:humankit.Press = cast pressJob.orderedActions()[1];
		if (Math.abs(press.point[2] - 1.35) > 1e-6) throw "Explicit press height was lost";
		var pointError = "";
		try HumanJobSpec.parse('{"version":1,"loop":false,"steps":[{"action":"press","target":{"point":[0.8,0.2]}}]}')
		catch (error:Dynamic) pointError = Std.string(error);
		if (pointError.indexOf("[x, y, z]") < 0) throw 'Press point error is unclear: $pointError';
		for (bad in [
			'{"version":2,"loop":false,"steps":[]}',
			'{"version":1,"loop":false,"steps":[{"action":"dance"}]}',
			'{"version":1,"loop":false,"steps":[{"action":"pick"}]}',
			'{"version":1,"loop":false,"steps":[{"action":"wait","seconds":1e400}]}',
			'{"version":1,"loop":false,"steps":[{"action":"pick","object":"part","hand":"foot"}]}',
			'{"version":1,"loop":false,"steps":[{"action":"place","onto":"table"}]}',
			'{"version":1,"loop":false,"steps":[{"action":"pick","object":"part"},{"action":"pick","object":"part"}]}',
			'{"version":1,"loop":false,"steps":[{"action":"wait","seconds":1,"unexpected":true}]}',
			'{"version":1,"loop":false,"steps":[],"future":3}'
		]) {
			var rejected = false;
			try HumanJobSpec.parse(bad) catch (error:Dynamic) rejected = true;
			if (!rejected) throw 'Accepted invalid job: $bad';
		}
		var missingPickError = "";
		try HumanJobSpec.parse('{"version":1,"loop":false,"steps":[{"action":"pick"}]}')
		catch (error:Dynamic) missingPickError = Std.string(error);
		if (missingPickError.indexOf("step 0.object") < 0)
			throw 'Missing pick error lost its field path: $missingPickError';
		var unknownStepError = "";
		try HumanJobSpec.parse('{"version":1,"loop":false,"steps":[{"action":"wait","seconds":1,"unexpected":true}]}')
		catch (error:Dynamic) unknownStepError = Std.string(error);
		if (unknownStepError.indexOf('step 0 has unknown field "unexpected"') < 0)
			throw 'Unknown step field error lost its field path: $unknownStepError';
		var warningSpec = HumanJobSpec.parse('{"version":1,"loop":false,"steps":[{"action":"walkTo","target":{"object":"missing"}},{"action":"playClip","clip":"NoSuchClip","seconds":1}]}');
		var warnings = HumanJobSpec.check(warningSpec, targets, body);
		if (warnings.length != 2 || warnings[0].indexOf('step 0: unknown object "missing"') < 0 ||
			warnings[1].indexOf('step 1: character lacks clip "NoSuchClip"') < 0)
			throw 'Wrong edit-time warnings: $warnings';
		targets.boxes.set("high", {center: [0.0, 0.0, 3.0], halfExtents: [0.1, 0.1, 0.1], yaw: 0.0});
		targets.boxes.set("low", {center: [0.0, 0.0, 0.1], halfExtents: [0.1, 0.1, 0.1], yaw: 0.0});
		var heights = HumanJobSpec.parse('{"version":1,"loop":false,"steps":[{"action":"press","target":{"object":"high"}},{"action":"press","target":{"object":"low"}}]}');
		if (HumanJobSpec.check(heights, targets, body).length != 2) throw "Missing reach warnings";
		var walkTargets = new JobTargets();
		walkTargets.boxes.set("rack", {center: [2.0, 0.0, 0.5], halfExtents: [0.5, 0.5, 0.5], yaw: 0.0});
		var walkSpec = HumanJobSpec.parse('{"version":1,"loop":false,"steps":[{"action":"walkTo","target":{"object":"rack"}}]}');
		var walk = HumanJobBuilder.build(walkSpec, walkTargets, body).job;
		for (tick in 0...500) if (!walk.isDone()) walk.advance(0.02);
		if (!walk.isDone() || walk.failure() != null) throw 'Object walk failed: ${walk.failure()}';
		var root = body.rootTransform();
		if (Math.abs(root[12] - 1.0) > 0.02 || Math.abs(root[13]) > 0.02 || root[0] < 0.99)
			throw 'Object walk stop/facing wrong: $root';
		walkTargets.boxes.set("table", {center: [2.0, 2.0, 0.5], halfExtents: [0.5, 0.5, 0.5], yaw: 0.0});
		body.walker.place(0.0, 0.0, 0.0);
		body.advance(0.0);
		var twoStops = HumanJobSpec.parse('{"version":1,"loop":false,"steps":[{"action":"walkTo","target":{"object":"rack"}},{"action":"walkTo","target":{"object":"table"}}]}');
		var routed = HumanJobBuilder.build(twoStops, walkTargets, body).job;
		for (tick in 0...800) if (!routed.isDone()) routed.advance(0.02);
		root = body.rootTransform();
		if (!routed.isDone() || routed.failure() != null || Math.abs(root[12]-1.341886) > 0.03 ||
			Math.abs(root[13]-1.025658) > 0.03)
			throw 'Second object stop ignored the rack-side approach: $root';
		body.walker.place(0.0, 0.0, 0.0);
		body.advance(0.0);
		walkTargets.boxes.set("button", {center: [2.0, 0.0, 1.5], halfExtents: [0.2, 0.2, 0.2], yaw: 0.0});
		var frontSpec = HumanJobSpec.parse('{"version":1,"loop":false,"steps":[{"action":"walkTo","target":{"point":[3,0]}},{"action":"press","target":{"object":"button","anchor":"front"}}]}');
		var frontJob = HumanJobBuilder.build(frontSpec, walkTargets, body).job;
		var frontPress:humankit.Press = cast frontJob.orderedActions()[2];
		for (tick in 0...900) if (!frontJob.isDone()) frontJob.advance(0.02);
		if (!frontJob.isDone() || frontJob.failure() != null || Math.abs(frontPress.point[0]-2.2) > 0.01)
			throw 'Front press used the build-time side: ${frontPress.point} root=${body.rootTransform()} failure=${frontJob.failure()}';
		body.walker.place(0.0, 0.0, 0.0);
		body.advance(0.0);
		var animationBuilt = HumanJobBuilder.build(spec, targets, body);
		var animation = animationBuilt.job;
		var animationPick:Pick = cast animationBuilt.holds[0].action;
		var animationPlace:Place = cast animationBuilt.holds[1].action;
		for (tick in 0...3000) {
			if (animation.isDone()) break;
			animation.advance(0.02);
		}
		if (!animation.isDone() || animation.failure() != null || !animationPick.grip ||
			animationPick.pickError > 0.02 || animationPlace.grip || animationPlace.placementError > 0.02)
			throw 'Built animation failed: ${animation.failure()}, pick miss ${animationPick.pickError}, place miss ${animationPlace.placementError}';
		human.dispose();
	}

	static function placeReferencePoint(scene:Scene, asset:AnimationAsset, rig:HumanoidRig):Void {
		for (withObject in [false, true]) {
			var human = new HumanCharacter(scene, asset, rig, null, "Place reference test");
			var body = new HumanBody(human);
			body.advance(0.0);
			// Without a held object Place puts the palm on target.
			var hand = withObject ? body.toWorld(human.pose.bonePosition(HumanBone.HandR)) : body.gripPoint(ArmR);
			var target = [hand[0] + (withObject ? 0.08 : 0.02), hand[1], hand[2]];
			if (withObject) body.setHeldPoint(ArmR, function() {
				var current = body.toWorld(human.pose.bonePosition(HumanBone.HandR));
				return [current[0] + 0.07, current[1], current[2]];
			});
			var place = new Place(target, [ArmR], withObject ? 0.0 : 0.1);
			var job = new HumanJob(body).add(place);
			job.advance(1.0 / 60.0);
			if (withObject && distance(body.heldPoint(ArmR), target) >= 0.005)
				throw 'Place did not solve the held-point offset in one advance: ${body.heldPoint(ArmR)}';
			for (_ in 0...99) {
				job.advance(1.0 / 60.0);
				if (!place.grip) break;
			}
			var actual = withObject ? body.heldPoint(ArmR) : body.gripPoint(ArmR);
			var error = distance(actual, target);
			if (place.grip || error >= 0.005 || withObject && place.placementError >= 0.005)
				throw 'Place reference point error: held=$withObject actual=$actual target=$target error=$error residual=${place.placementError}';
			human.dispose();
		}
		var human = new HumanCharacter(scene, asset, rig, null, "Unreachable place test");
		var body = new HumanBody(human);
		body.advance(0.0);
		var hand = body.toWorld(human.pose.bonePosition(HumanBone.HandR));
		var unreachable = new Place([hand[0] + 2.0, hand[1], hand[2]], [ArmR], 0.1);
		var job = new HumanJob(body).add(unreachable);
		for (_ in 0...5) {
			var before = body.toWorld(human.pose.bonePosition(HumanBone.HandR));
			job.advance(1.0 / 60.0);
			var after = body.toWorld(human.pose.bonePosition(HumanBone.HandR));
			if (distance(before, after) > 0.05) throw 'Unreachable Place moved the hand: $before to $after';
		}
		if (job.failure() == null || job.failure().indexOf("Place target out of reach") < 0)
			throw 'Unreachable Place did not fail clearly: ${job.failure()}';
		human.dispose();
	}

	/** Measurements and capsule placement at the worker's rest pose. */
	static function bodyProxy(rest:HumanPose, height:Float):Void {
		var description = HumanDescription.measure(rest, height);
		if (description.stature != height || description.scale != 1.0)
			throw "A measured description keeps the rig's height and scale";
		inRange(description.shoulderWidth, 0.2, 0.5, "shoulder width");
		inRange(description.hipWidth, 0.1, 0.4, "hip width");
		inRange(description.upperArm, 0.15, 0.4, "upper arm");
		inRange(description.thigh, 0.3, 0.6, "thigh");
		inRange(description.torso, 0.3, 0.8, "torso");

		var proxy = HumanBodyProxy.standard(rest, description);
		if (proxy.capsules.length != 15)
			throw 'Expected 15 capsules, got ${proxy.capsules.length}';
		var placements = proxy.place(rest);
		var arm = capsuleIndex(proxy, "upper_arm.L");
		var shoulder = rest.bonePosition(HumanBone.UpperArmL), elbow = rest.bonePosition(HumanBone.ForearmL);
		for (axis in 0...3)
			if (Math.abs(placements[arm].center[axis] - (shoulder[axis] + elbow[axis]) * 0.5) > 1e-6)
				throw "The upper arm capsule is not centred between shoulder and elbow";
		var along = Mat4.normalize(Mat4.subtract(elbow, shoulder)), axisOfArm = capsuleAxis(placements[arm]);
		if (Mat4.dot(along, axisOfArm) < 1 - 1e-9)
			throw "The upper arm capsule does not lie along the arm";
		var head = capsuleIndex(proxy, "head");
		var crown = capsuleEnd(proxy.capsules[head], placements[head], 1.0)[2] + proxy.capsules[head].radius;
		if (Math.abs(crown - height) > 0.01)
			throw 'The head capsule reaches $crown, not the crown at $height';
		// Feet rest on the floor; nothing sinks more than a few centimetres below it.
		for (name in ["foot.L", "foot.R", "shin.L", "shin.R"]) {
			var index = capsuleIndex(proxy, name);
			var capsule = proxy.capsules[index];
			var lowest = Math.min(capsuleEnd(capsule, placements[index], -1.0)[2], capsuleEnd(capsule, placements[index], 1.0)[2])
				- capsule.radius;
			if (StringTools.startsWith(name, "foot") && lowest > 0.06 || lowest < -0.03)
				throw '$name sits at $lowest, not on the floor';
		}

		// Placing through a root: turned a quarter about +Z and moved 5 m along +X.
		var root = [0.0, 1.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 5.0, 0.0, 0.0, 1.0];
		var moved = proxy.place(rest, root)[arm];
		var expected = [5.0 - placements[arm].center[1], placements[arm].center[0], placements[arm].center[2]];
		for (axis in 0...3)
			if (Math.abs(moved.center[axis] - expected[axis]) > 1e-6)
				throw "The root transform does not carry the capsules";

		// The same body twice as tall: every capsule doubles, and so does the root's scale.
		var giant = HumanDescription.measure(rest, height).scaledTo(height * 2.0);
		if (Math.abs(giant.scale - 2.0) > 1e-12 || Math.abs(giant.upperArm - description.upperArm * 2.0) > 1e-9)
			throw "Scaling a description does not scale its lengths";
		var tall = HumanBodyProxy.standard(rest, giant);
		for (index in 0...proxy.capsules.length)
			if (Math.abs(tall.capsules[index].length - proxy.capsules[index].length * 2.0) > 1e-6
				|| Math.abs(tall.capsules[index].radius - proxy.capsules[index].radius * 2.0) > 1e-9)
				throw '${proxy.capsules[index].name} does not scale with the body';
		var scaled = tall.place(rest, [2.0, 0.0, 0.0, 0.0, 0.0, 2.0, 0.0, 0.0, 0.0, 0.0, 2.0, 0.0, 0.0, 0.0, 0.0, 1.0]);
		var tallCrown = capsuleEnd(tall.capsules[head], scaled[head], 1.0)[2] + tall.capsules[head].radius;
		if (Math.abs(tallCrown - height * 2.0) > 0.02)
			throw 'The scaled head reaches $tallCrown, not ${height * 2.0}';
	}

	/** The capsule and skeleton views draw only when shown and follow the pose. */
	static function bodyView(scene:Scene, human:HumanCharacter):Void {
		var proxy = HumanBodyProxy.standard(human.pose, HumanDescription.measure(human.pose, human.height()));
		var base = drawCalls(scene);
		var view = new HumanBodyView(scene, human.root, proxy, human.pose);
		if (drawCalls(scene) != base)
			throw "A new body view draws before it is shown";
		view.show(Capsules);
		if (drawCalls(scene) != base + proxy.capsules.length)
			throw 'The capsule view draws ${drawCalls(scene) - base} parts, not ${proxy.capsules.length}';
		view.show(Skeleton);
		var bones = drawCalls(scene) - base;
		if (bones < 15 || bones + proxy.capsules.length != view.nodes().length)
			throw 'The skeleton view draws $bones bones';
		view.show(Mesh);
		if (drawCalls(scene) != base)
			throw "Returning to the mesh still draws the body view";

		human.advance(0.1);
		view.update(human.pose);
		var placement = proxy.place(human.pose)[0];
		var snapshot = scene.snapshot();
		// The character root sits at the origin, so model space is world space.
		var world = snapshot.findNode(view.nodes()[0]).worldTransform();
		for (axis in 0...3)
			if (Math.abs(world.element(12 + axis) - placement.center[axis]) > 1e-4)
				throw "The first capsule node is not where the proxy places it";
		snapshot.dispose();
	}

	/** Walking a route with the clip matched to the speed keeps planted feet still. */
	static function walking(scene:Scene, asset:AnimationAsset, rig:HumanoidRig):Void {
		var human = new HumanCharacter(scene, asset, rig, null, "Walker");
		var walker = new HumanWalker(human);
		inRange(walker.gait.naturalSpeed, 0.5, 2.5, "natural walking speed");
		if (walker.isWalking())
			throw "A new walker is already walking";

		// Along +X, then a left turn towards +Y.
		var speed = 1.4;
		walker.follow([[0.0, 0.0], [3.0, 0.0], [3.0, 2.0]], speed);
		var step = 1.0 / 60.0;
		var drift = 0.0, travel = 0.0, previous:Null<Array<Float>> = null, lowest = Math.POSITIVE_INFINITY;
		var heights:Array<Float> = [], worldFeet:Array<Array<Float>> = [];
		for (_ in 0...90) {
			walker.advance(step);
			var root = walker.rootTransform();
			var foot = human.pose.bonePosition(HumanBone.FootL);
			var world = [
				root[0] * foot[0] + root[4] * foot[1] + root[12],
				root[1] * foot[0] + root[5] * foot[1] + root[13]
			];
			heights.push(foot[2]);
			worldFeet.push(world);
			lowest = Math.min(lowest, foot[2]);
		}
		// After the 0.3 s start: the body is at full speed and the walk fully faded in.
		for (index in 24...worldFeet.length)
			if (heights[index] <= lowest + 0.01 && heights[index - 1] <= lowest + 0.01) {
				drift += Math.abs(worldFeet[index][0] - worldFeet[index - 1][0]);
				travel += speed * step;
			}
		if (travel == 0.0)
			throw "The left foot never planted during the walk";
		if (drift > 0.25 * travel)
			throw 'A planted foot slid ${drift} m while the body walked ${travel} m';
		// Ramping up from rest over 0.3 s costs half of that at full speed; per-frame
		// integration may add up to one frame's travel.
		if (Math.abs(walker.distance() - speed * (90 * step - 0.15)) > speed * step)
			throw 'The walker covered ${walker.distance()} m, not a ramped ${speed * (90 * step - 0.15)} m';
		if (Math.abs(human.player.speed - speed / walker.gait.naturalSpeed) > 1e-9)
			throw "The walk clip does not play at the gait-matched rate";

		for (_ in 0...600)
			walker.advance(step);
		var root = walker.rootTransform();
		if (walker.isWalking() || Math.abs(root[12] - 3.0) > 1e-6 || Math.abs(root[13] - 2.0) > 1e-6)
			throw 'The walker did not stop at the end of its route: ${root[12]}, ${root[13]}';
		// Facing +Y after the turn.
		if (Math.abs(root[0]) > 1e-3 || Math.abs(root[1] - 1.0) > 1e-3)
			throw "The walker does not face along the last leg";
		if (human.player.currentClip() != asset.clipIndex("idle") || human.player.speed != 1.0)
			throw "The walker does not idle at normal speed after arriving";
		human.dispose();
	}

	/** Two-bone IK puts a wrist on a target, bends the elbow downwards, and lets go cleanly. */
	static function reaching(scene:Scene, asset:AnimationAsset, rig:HumanoidRig):Void {
		var human = new HumanCharacter(scene, asset, rig, null, "Reacher");
		human.advance(0.0);
		var shoulder = human.pose.bonePosition(HumanBone.UpperArmR);
		var restHand = human.pose.bonePosition(HumanBone.HandR);
		var description = HumanDescription.measure(human.pose, human.height());
		var reachLength = description.upperArm + description.forearm;
		// Ahead of the right shoulder and a little down: well within reach.
		var direction = Mat4.normalize([1.0, -0.2, -0.3]);
		var target = [for (axis in 0...3) shoulder[axis] + direction[axis] * reachLength * 0.8];
		human.reach(ArmR, target);
		human.advance(0.0);
		var hand = human.pose.bonePosition(HumanBone.HandR);
		if (distance(hand, target) > 0.01)
			throw 'The wrist reached ${hand}, not ${target}';
		var elbow = human.pose.bonePosition(HumanBone.ForearmR);
		if (elbow[2] >= (shoulder[2] + hand[2]) * 0.5)
			throw "The elbow does not bend downwards";

		// Out of reach: the arm straightens towards the target.
		var far = [for (axis in 0...3) shoulder[axis] + direction[axis] * reachLength * 3.0];
		human.reach(ArmR, far);
		human.advance(0.0);
		hand = human.pose.bonePosition(HumanBone.HandR);
		var towards = Mat4.normalize(Mat4.subtract(hand, shoulder));
		if (Mat4.dot(towards, direction) < 0.99 || Math.abs(distance(hand, shoulder) - reachLength) > 0.02)
			throw "An unreachable target does not straighten the arm towards it";

		// Half weight lands between the animation and the full reach.
		human.reach(ArmR, target, 0.5);
		human.advance(0.0);
		var half = human.pose.bonePosition(HumanBone.HandR);
		if (distance(half, target) < 0.01 || distance(half, restHand) < 0.01)
			throw "Half-weight IK does not blend";

		human.release(ArmR);
		human.advance(0.0);
		if (distance(human.pose.bonePosition(HumanBone.HandR), restHand) > 1e-4)
			throw "Releasing the arm does not restore its animated pose";
		var rejected = false;
		try human.reach(LegL, [0.3, 0.1, 0.1]) catch (_:Dynamic) rejected = true;
		if (!rejected)
			throw "A Quaternius leg, whose foot is not below its shin, was accepted as a chain";
		human.dispose();
	}

	/** Curling a hand brings its fingertips toward the wrist, on both hands, and opening returns them. */
	static function fingersCurl(scene:Scene, asset:AnimationAsset, rig:HumanoidRig):Void {
		var human = new HumanCharacter(scene, asset, rig, null, "Curler");
		human.instance.clearLayers();
		for (side in ["L", "R"]) {
			var hand = side == "L" ? HumanLimb.ArmL : HumanLimb.ArmR;
			var spread = function():Float {
				human.instance.evaluate();
				var matrices = human.instance.readJointMatrices();
				var tip = asset.jointIndex('Middle4.$side') * 64, wrist = asset.jointIndex('Wrist.$side') * 64;
				return distance([matrices.getFloat(tip + 48), matrices.getFloat(tip + 52), matrices.getFloat(tip + 56)],
					[matrices.getFloat(wrist + 48), matrices.getFloat(wrist + 52), matrices.getFloat(wrist + 56)]);
			};
			human.setHandCurl(hand, 0.0);
			var open = spread();
			human.setHandCurl(hand, 1.0);
			var fist = spread();
			if (open - fist < 0.05)
				throw 'Curling the $side hand moved the middle fingertip from ${open} m to ${fist} m of the wrist';
			human.setHandCurl(hand, 0.5);
			var half = spread();
			if (half > open - 0.01 || half < fist + 0.01)
				throw 'A half curl of the $side hand is not between open and closed: $open, $half, $fist';
			human.setHandCurl(hand, 0.0);
			if (Math.abs(spread() - open) > 1e-4)
				throw 'Opening the $side hand did not return the fingers';
			if (human.handCurl(hand) != 0.0) throw "handCurl does not report the curl";
		}
		var rejected = false;
		try human.setHandCurl(LegL, 1.0) catch (_:Dynamic) rejected = true;
		if (!rejected) throw "A leg was given fingers to curl";
		human.dispose();
	}

	/**
	 * A hand closes on an object to the object's size: each finger curls until its own tip is as deep as the
	 * object, so a thicker object closes the hand more, fingers of different lengths close differently, a
	 * thin one is pinched, and the shape goes once the object is let go.
	 */
	static function grasping(scene:Scene, asset:AnimationAsset, rig:HumanoidRig):Void {
		var human = new HumanCharacter(scene, asset, rig, null, "Grasper");
		human.player.play(asset.clipIndex("idle"), 0.0);
		human.advance(0.0);
		var body = new HumanBody(human);
		var step = 1.0 / 60.0;
		for (hand in [ArmR, ArmL]) {
			// Tip depth runs from about 2.3 cm (open) to 6 to 7 cm (curled half way), then falls as the fist closes.
			var shallow = grasped(body, hand, 0.035), medium = grasped(body, hand, 0.05), deep = grasped(body, hand, 0.06);
			if (!(shallow[HumanHand.INDEX] < medium[HumanHand.INDEX] && medium[HumanHand.INDEX] < deep[HumanHand.INDEX]))
				throw 'A thicker object did not close the $hand hand further: ${shallow[1]}, ${medium[1]}, ${deep[1]}';
			// Each finger's tip ends as deep as the object, to a few millimetres.
			body.setGrasp(hand, 0.05);
			human.setHandCurls(hand, body.grasp(hand));
			human.advance(0.0);
			for (finger in [HumanHand.INDEX, HumanHand.MIDDLE, HumanHand.RING, HumanHand.PINKY])
				if (Math.abs(body.fingerDepth(hand, finger) - 0.05) > 0.006)
					throw 'Finger $finger of the $hand hand is ${body.fingerDepth(hand, finger)} m deep, not 0.05';
			// Fingers of different lengths close by different amounts to the same depth.
			var spread = Math.max(Math.max(medium[1], medium[2]), Math.max(medium[3], medium[4])) -
				Math.min(Math.min(medium[1], medium[2]), Math.min(medium[3], medium[4]));
			if (spread < 0.02) throw 'Every finger of the $hand hand closed alike to one depth: $medium';
			// A thin object is pinched: only the thumb and index close on it.
			var thin = grasped(body, hand, 0.02);
			if (thin[HumanHand.MIDDLE] > body.posture.relaxedCurl + 1e-6 || thin[HumanHand.PINKY] > body.posture.relaxedCurl + 1e-6)
				throw 'The $hand hand did not pinch a thin object: $thin';
			if (Math.abs(thin[HumanHand.THUMB] - body.posture.thumbShare * thin[HumanHand.INDEX]) > 1e-6)
				throw "The thumb does not follow the index finger";
			if (medium[HumanHand.PINKY] <= thin[HumanHand.PINKY] && medium[HumanHand.MIDDLE] <= thin[HumanHand.MIDDLE])
				throw 'A thick object was no more closed on than a thin one by the $hand hand: $medium against $thin';
		}
		// Holding, the fingers settle into the grasp; letting go, they relax and the grasp is forgotten.
		body.setGrasp(ArmR, 0.05);
		var wanted = body.grasp(ArmR);
		body.setCarry([ArmR]);
		for (_ in 0...60) body.advance(step);
		var held = human.handCurls(ArmR);
		for (finger in 0...HumanHand.FINGERS)
			if (Math.abs(held[finger] - wanted[finger]) > 0.02)
				throw 'Finger $finger did not settle into the grasp: ${held[finger]} against ${wanted[finger]}';
		body.setCarry([]);
		for (_ in 0...60) body.advance(step);
		var relaxed = human.handCurls(ArmR);
		for (finger in 0...HumanHand.FINGERS)
			if (Math.abs(relaxed[finger] - body.posture.relaxedCurl) > 0.02)
				throw 'Finger $finger did not relax after letting go: ${relaxed[finger]}';
		if (body.grasp(ArmR) != null) throw "The grasp was kept after the object was let go";
		human.dispose();
	}

	static function grasped(body:HumanBody, hand:HumanLimb, depth:Float):Array<Float> {
		body.setGrasp(hand, depth);
		return body.grasp(hand);
	}

	/** Leaning carries the shoulders forward, an unused arm hangs instead of swinging back, and standing up undoes it. */
	static function leaning(scene:Scene, asset:AnimationAsset, rig:HumanoidRig):Void {
		var human = new HumanCharacter(scene, asset, rig, null, "Leaner");
		human.player.play(asset.clipIndex("idle"), 0.0);
		human.advance(0.0);
		var body = new HumanBody(human);
		var step = 1.0 / 60.0;
		var shoulderX = function(bone:HumanBone):Float return human.pose.bonePosition(bone)[0];
		var upright = shoulderX(HumanBone.UpperArmR);
		var wanted = 0.06;
		var made = body.leanFor(ArmR, wanted);
		if (Math.abs(made.shift - wanted) > 0.005)
			throw 'A lean asked to move the shoulder $wanted m is predicted to move it ${made.shift} m';
		if (Math.abs(shoulderX(HumanBone.UpperArmR) - upright) > 1e-4 || human.spineLean() != 0.0)
			throw "Working out a lean left the pose changed";
		var capped = body.leanFor(ArmR, 5.0);
		if (capped.angle > body.posture.maxLean + 1e-6 || capped.shift >= 5.0)
			throw 'A lean of ${capped.angle} rad was allowed past ${body.posture.maxLean}';

		body.setLean(made.angle);
		for (_ in 0...90) body.advance(step);
		var leaned = shoulderX(HumanBone.UpperArmR) - upright;
		if (Math.abs(leaned - made.shift) > 0.01)
			throw 'Leaning moved the shoulder $leaned m, not the ${made.shift} m predicted';
		body.setLean(0.5);
		for (_ in 0...90) body.advance(step);
		var far = human.pose.bonePosition(HumanBone.UpperArmL), hand = human.pose.bonePosition(HumanBone.HandL);
		var length = body.description.upperArm + body.description.forearm;
		if (far[2] - hand[2] < 0.6 * length || Math.abs(hand[0] - far[0]) > 0.25 * length)
			throw 'The arm not reaching does not hang under its shoulder while leaning: $far to $hand';
		body.setLean(0.0);
		for (_ in 0...120) body.advance(step);
		if (Math.abs(shoulderX(HumanBone.UpperArmR) - upright) > 0.01 || human.spineLean() > 1e-3)
			throw "Standing up did not undo the lean";
		human.dispose();
	}

	/**
	 * Sweeping the wrist through the point straight under the shoulder moves the elbow a little per
	 * step. A fixed elbow direction pointing down would make the bend plane undefined there, and the
	 * elbow would swing through a half turn around the arm on a one-degree step of the hand.
	 */
	static function elbowStaysPut(scene:Scene, asset:AnimationAsset, rig:HumanoidRig):Void {
		var human = new HumanCharacter(scene, asset, rig, null, "Sweeper");
		human.advance(0.0);
		var shoulder = human.pose.bonePosition(HumanBone.UpperArmR);
		var description = HumanDescription.measure(human.pose, human.height());
		var radius = (description.upperArm + description.forearm) * 0.75;
		var previous:Null<Array<Float>> = null;
		var worst = 0.0;
		for (step in 0...81) {
			var angle = (step - 50) * Math.PI / 180.0;
			var target = [shoulder[0] + radius * Math.sin(angle), shoulder[1], shoulder[2] - radius * Math.cos(angle)];
			human.reach(ArmR, target);
			human.advance(0.0);
			var elbow = human.pose.bonePosition(HumanBone.ForearmR);
			if (previous != null) worst = Math.max(worst, distance(elbow, previous));
			previous = elbow;
		}
		if (worst > 0.04)
			throw 'The elbow moved $worst m for a one-degree step of the wrist';
		human.dispose();
	}

	/**
	 * Sweeping the wrist round the shoulder along great circles in every plane, two degrees a step, moves the elbow a
	 * little per step wherever the arm points but straight out to its side or straight across, so the bend does not
	 * flip where the arm points opposite to the way the animation holds it. The library character, bent far over
	 * (lean and hinge together, as for a deep low top), holds its arm pointing the opposite way to where a reach goes.
	 */
	static function elbowStaysPutEverywhere(scene:Scene, bundled:AnimationAsset, bundledRig:HumanoidRig):Void {
		var library = AnimationAsset.load(assetDir() + "/quaternius-ual/ual-work.glb");
		var libraryRig = HumanoidRig.detect(library);
		var worst = 0.0, worstWhere = "";
		for (variant in 0...2) {
			var asset = variant == 0 ? bundled : library, rig = variant == 0 ? bundledRig : libraryRig;
			var human = new HumanCharacter(scene, asset, rig, null, "Sweeper");
			if (variant == 1) {
				human.setSpineLean(0.7);
				human.setSpineHinge(0.8);
			}
			human.advance(0.0);
			var radius = 0.75 * (function() {
				var description = HumanDescription.measure(human.pose, human.height());
				return description.upperArm + description.forearm;
			})();
			for (limb in [ArmL, ArmR]) {
				var shoulder = human.pose.bonePosition(limb == ArmL ? HumanBone.UpperArmL : HumanBone.UpperArmR);
				var elbowBone = limb == ArmL ? HumanBone.ForearmL : HumanBone.ForearmR;
				for (ring in 0...12) {
					// A great circle: the plane through the shoulder with normal (cos a cos b, sin a cos b, sin b).
					var a = ring * Math.PI / 6.0, b = (ring % 4) * Math.PI / 8.0;
					var normal = [Math.cos(a) * Math.cos(b), Math.sin(a) * Math.cos(b), Math.sin(b)];
					var u = Mat4.normalize(Mat4.cross(normal, Math.abs(normal[2]) < 0.9 ? [0.0, 0.0, 1.0] : [1.0, 0.0, 0.0]));
					var v = Mat4.cross(normal, u);
					var previous:Null<Array<Float>> = null;
					for (step in 0...180) {
						var angle = step * 2.0 * Math.PI / 180.0;
						var axis = [for (i in 0...3) Math.cos(angle) * u[i] + Math.sin(angle) * v[i]];
						// Straight out to the side or straight across has no defined bend; the field's one fault.
						if (Math.abs(axis[1]) > Math.cos(12.0 * Math.PI / 180.0)) { previous = null; continue; }
						human.reach(limb, [for (i in 0...3) shoulder[i] + radius * axis[i]]);
						human.advance(0.0);
						var elbow = human.pose.bonePosition(elbowBone);
						if (previous != null) {
							var moved = distance(elbow, previous);
							if (moved > worst) { worst = moved; worstWhere = 'variant $variant limb $limb ring $ring step $step'; }
						}
						previous = elbow;
					}
					human.release(limb);
				}
			}
			human.dispose();
		}
		Sys.println('ELBOW everywhere: worst step ${Math.round(worst * 1000) / 1000} m ($worstWhere)');
		if (worst > 0.06)
			throw 'The elbow moved $worst m for a two-degree step of the wrist ($worstWhere)';
	}

	/**
	 * A real FacilityRouter route through two lanes, adapted with FacilityWalk:
	 * the person ends at the destination station and faces along the last lane,
	 * not the straight line from the start.
	 */
	static function facilityRoute(scene:Scene, asset:AnimationAsset, rig:HumanoidRig):Void {
		var facility = new Facility("warehouse", "Warehouse A");
		facility.addZone(new Zone("floor", "Floor", "map", Footprint.rectangle(10.0, 10.0)));
		var start = new Station("start", "Start", "floor", "map", new Pose2(0.0, 0.0, 0.0));
		var corner = new Station("corner", "Corner", "floor", "map", new Pose2(4.0, 0.0, 0.0));
		var dest = new Station("dest", "Dest", "floor", "map", new Pose2(4.0, 3.0, Math.PI / 2));
		facility.addStation(start);
		facility.addStation(corner);
		facility.addStation(dest);
		facility.addLane(new Lane("start-corner", start.id, corner.id,
			new Path([start.pose, corner.pose], "map"), 1.0, 1.4));
		facility.addLane(new Lane("corner-dest", corner.id, dest.id,
			new Path([corner.pose, dest.pose], "map"), 1.0, 1.4));
		var route = new FacilityRouter(facility).route(start.id, dest.id);
		var points = FacilityWalk.routeFromFacilityRoute(route);
		if (points.length != 3)
			throw 'Expected 3 route points, got ${points.length}';

		var human = new HumanCharacter(scene, asset, rig, null, "FacilityWalker");
		var walker = new HumanWalker(human);
		walker.follow(points, route.maximumSpeedMetersPerSecond);
		var step = 1.0 / 60.0;
		for (_ in 0...600)
			walker.advance(step);
		var root = walker.rootTransform();
		if (walker.isWalking())
			throw "The facility walk did not stop at its destination";
		if (Math.abs(root[12] - dest.pose.x) > 1e-3 || Math.abs(root[13] - dest.pose.y) > 1e-3)
			throw 'The facility walk ended at ${root[12]}, ${root[13]}, not the destination station';
		// The last lane runs along +Y; the walker must face along it, not the
		// straight line from start to destination.
		if (Math.abs(root[0]) > 1e-3 || Math.abs(root[1] - 1.0) > 1e-3)
			throw "The facility walk does not face along the last lane";
		human.dispose();
	}

	/**
	 * The floor-up measures on motions whose answer is known: standing still has no skate and a mass well inside
	 * its feet, and a steady walk at the gait's own speed keeps a planted foot where it is. The same measures
	 * on a whole job are reported by the sim sweeps.
	 */
	static function naturalness(scene:Scene, asset:AnimationAsset, rig:HumanoidRig, label:String, slideLimit:Float = 0.08):Void {
		var human = new HumanCharacter(scene, asset, rig, null, "Natural");
		var body = new HumanBody(human);
		var step = 1.0 / 60.0;
		for (_ in 0...30) body.advance(step);
		var still = new Naturalness();
		for (_ in 0...120) {
			body.advance(step);
			still.sample(body, step);
		}
		if (still.maxSlide > 0.005 || still.minSupportMargin < -0.02)
			throw 'A worker standing still slid or tipped ($label): ${still.summary()}';
		var walking = new Naturalness();
		body.walker.follow([[0.0, 0.0], [20.0, 0.0]], 1.0);
		for (index in 0...360) {
			body.advance(step);
			// Once the body is up to speed: getting there drags a planted foot (see BODY.md), which is not the gait's doing.
			if (index >= 90) walking.sample(body, step);
		}
		if (walking.maxSlide > slideLimit || walking.plantedSeconds < 1.0)
			throw 'A steady walk slid its feet or never planted them ($label): ${walking.summary()}';
		human.dispose();
	}

	/**
	 * Turning on the spot: a character with turn clips steps through a right angle in the clip's own time and ends
	 * facing where it was asked, with its feet moving less than the root-spin the bundled worker makes of the same turn.
	 */
	static function turning(scene:Scene, bundled:AnimationAsset, bundledRig:HumanoidRig):Void {
		var turnOf = function(asset:AnimationAsset, rig:HumanoidRig, angle:Float):{seconds:Float, slide:Float, heading:Float, clips:Int} {
			var human = new HumanCharacter(scene, asset, rig, null, "Turner");
			var body = new HumanBody(human);
			var step = 1.0 / 60.0;
			for (_ in 0...30) body.advance(step);
			var natural = new Naturalness();
			var before = body.rootTransform();
			body.walker.face(Math.atan2(before[1], before[0]) + angle);
			var seconds = 0.0, clips = 0, last = -1;
			while (body.walker.isTurning() && seconds < 6.0) {
				body.advance(step);
				natural.sample(body, step);
				seconds += step;
				var playing = human.player.currentClip();
				if (playing != last) {
					clips++;
					last = playing;
				}
			}
			for (_ in 0...30) body.advance(step);
			var after = body.rootTransform();
			var heading = Math.atan2(after[1], after[0]) - Math.atan2(before[1], before[0]);
			while (heading > Math.PI) heading -= 2.0 * Math.PI;
			while (heading < -Math.PI) heading += 2.0 * Math.PI;
			human.dispose();
			return {seconds: seconds, slide: natural.maxSlide, heading: heading, clips: clips};
		};
		var plain = turnOf(bundled, bundledRig, Math.PI / 2.0);
		var asset = AnimationAsset.load(assetDir() + "/quaternius-ual/ual-work.glb");
		var rig = HumanoidRig.detect(asset);
		var quarter = turnOf(asset, rig, Math.PI / 2.0);
		var half = turnOf(asset, rig, Math.PI);
		var odd = turnOf(asset, rig, -2.0);
		Sys.println('TURN bundled 90: ${plain.seconds} s slide ${plain.slide}; library 90: ${quarter.seconds} s slide ${quarter.slide} clips ${quarter.clips}; 180: ${half.seconds} s slide ${half.slide}; -2 rad: ${odd.seconds} s slide ${odd.slide}');
		if (Math.abs(quarter.heading - Math.PI / 2.0) > 0.02 || Math.abs(half.heading - Math.PI) > 0.02 && Math.abs(half.heading + Math.PI) > 0.02 ||
			Math.abs(odd.heading + 2.0) > 0.02)
			throw 'A turn clip left the worker facing the wrong way: ${quarter.heading}, ${half.heading}, ${odd.heading}';
		if (quarter.clips < 2) throw "A right-angle turn did not play a turn clip";
		if (quarter.seconds > 3.0) throw 'A right-angle turn took ${quarter.seconds} s';
	}

	/** A retreat on a character with a backward walk plays it, and its planted feet hold still; without one it slides as before. */
	static function retreating(scene:Scene, bundled:AnimationAsset, bundledRig:HumanoidRig):Void {
		var asset = AnimationAsset.load(assetDir() + "/quaternius-ual/ual-work.glb");
		var rig = HumanoidRig.detect(asset);
		var human = new HumanCharacter(scene, asset, rig, null, "Retreater");
		var body = new HumanBody(human);
		var step = 1.0 / 60.0;
		for (_ in 0...30) body.advance(step);
		if (body.walker.backGait == null) throw "The library character has no backward gait";
		var natural = new Naturalness();
		var start = body.rootTransform();
		body.walker.retreatAlong([[start[12], start[13]], [start[12] - 2.0 * start[0], start[13] - 2.0 * start[1]]], 0.8);
		if (human.player.currentClip() != asset.clipIndex("walk_bwd")) throw "A retreat did not take the backward walk";
		var seconds = 0.0;
		while (body.walker.isWalking() && seconds < 6.0) {
			body.advance(step);
			if (seconds > 0.5) natural.sample(body, step);
			seconds += step;
		}
		var end = body.rootTransform();
		if (Math.abs(Math.sqrt(Math.pow(end[12] - start[12], 2) + Math.pow(end[13] - start[13], 2)) - 2.0) > 0.01 ||
			Math.abs(end[0] - start[0]) > 1e-6)
			throw "A backward retreat did not end two metres back, facing the same way";
		Sys.println('RETREAT planted ${natural.plantedSeconds} s, slide ${natural.maxSlide}');
		if (natural.plantedSeconds < 0.5 || natural.maxSlide > 0.2) throw 'A backward retreat slid its feet: ${natural.summary()}';
		human.dispose();
		var plain = new HumanCharacter(scene, bundled, bundledRig, null, "PlainRetreater");
		var plainBody = new HumanBody(plain);
		if (plainBody.walker.backGait != null) throw "The bundled worker claims a backward gait";
		plain.dispose();
	}

	/**
	 * A character whose legs are IK chains holds its planted feet in the world while the idle pose shows, so starting to
	 * walk does not drag them.
	 */
	static function holdingFeet(scene:Scene):Void {
		var asset = AnimationAsset.load(assetDir() + "/quaternius-ual/ual-work.glb");
		var rig = HumanoidRig.detect(asset);
		// How far the left foot, which stays planted as the first step is taken with the right, is dragged along the floor in the first sixth of a second.
		var dragOf = function(hold:Bool):Float {
			var human = new HumanCharacter(scene, asset, rig, null, "FootHolder");
			var posture = HumanPosture.forStature(human.height());
			posture.lockFeet = hold;
			var body = new HumanBody(human, null, posture);
			var step = 1.0 / 60.0;
			for (_ in 0...30) body.advance(step);
			var before = body.toWorld(human.pose.bonePosition(HumanBone.FootL));
			body.walker.follow([[0.0, 0.0], [30.0, 0.0]], 1.0);
			for (_ in 0...10) body.advance(step);
			var after = body.toWorld(human.pose.bonePosition(HumanBone.FootL));
			human.dispose();
			return Math.sqrt(Math.pow(after[0] - before[0], 2) + Math.pow(after[1] - before[1], 2));
		};
		var held = dragOf(true), loose = dragOf(false);
		if (!(held < 0.03) || !(loose > held + 0.05))
			throw 'Holding the feet did not stop the drag at the start of a walk: ${held} m held, ${loose} m loose';
	}

	/**
	 * A foot reached to a spot keeps the way it lies when asked to (`keep_end_rotation`): the ankle goes where it is sent and
	 * the foot's pitch does not follow the shin, where without it the pitch swings by the amount the leg bends.
	 */
	static function footKeepsItsPitch(scene:Scene):Void {
		var asset = AnimationAsset.load(assetDir() + "/quaternius-ual/ual-work.glb");
		var rig = HumanoidRig.detect(asset);
		var human = new HumanCharacter(scene, asset, rig, null, "FootPitch");
		human.player.play(asset.clipIndex("idle"), 0.0);
		human.advance(0.0);
		var pitch = function():Float {
			var ankle = human.pose.bonePosition(HumanBone.FootL), toe = human.pose.bonePosition(HumanBone.ToeL);
			return Math.atan2(toe[2] - ankle[2], Math.sqrt(Math.pow(toe[0] - ankle[0], 2) + Math.pow(toe[1] - ankle[1], 2))) * 180 / Math.PI;
		};
		var foot = human.pose.bonePosition(HumanBone.FootL);
		var before = pitch();
		var target = [foot[0] - 0.12, foot[1], foot[2] + 0.12];
		var bones = [rig.joint(HumanBone.ThighL), rig.joint(HumanBone.ShinL), rig.joint(HumanBone.FootL)];
		var swing = [0.0, 0.0];
		for (index in 0...2) {
			human.instance.setIk(2, bones[0], bones[1], bones[2], target, [1.0, 0.0, 0.0], 1.0, 1.0, index);
			human.advance(0.0);
			swing[index] = Math.abs(pitch() - before);
			if (distance(human.pose.bonePosition(HumanBone.FootL), target) > 0.01) throw "The ankle did not reach its target with the foot's pitch kept";
		}
		human.dispose();
		if (!(swing[0] > 15.0) || !(swing[1] < 3.0))
			throw 'Keeping the foot\'s rotation did not hold its pitch: it swung ${swing[0]} degrees loose and ${swing[1]} kept';
	}

	/** On the full library, a top too low for a crouch is taken from a knee; one a crouch reaches is not. */
	static function kneeling(scene:Scene):Void {
		var asset = AnimationAsset.load(assetDir() + "/quaternius-ual/ual-work.glb");
		var rig = HumanoidRig.detect(asset);
		var human = new HumanCharacter(scene, asset, rig, null, "Kneeler");
		var body = new HumanBody(human);
		if (!body.canKneel()) throw "The full library character cannot kneel";
		var plan = function(z:Float, top:Float):ApproachFor {
			var box:HumanTargetBox = {center: [0.6, 0.0, top - 0.05], halfExtents: [0.2, 0.2, 0.05], yaw: 0.0};
			var approach = new ApproachFor([0.6, 0.0, z], ArmR, 1.0, false, null, box);
			var job = new HumanJob(body).add(approach);
			job.advance(1.0 / 60.0);
			if (job.failure() != null) throw 'A target at $z m was not reachable: ${job.failure()}';
			body.cancel();
			return approach;
		};
		var floor = plan(0.34, 0.3);
		if (!(floor.kneel >= 0.5) || floor.crouch != 0.0)
			throw 'The planner did not kneel for a top 0.3 m high: kneel ${floor.kneel}, crouch ${floor.crouch}';
		var bench = plan(0.62, 0.6);
		if (bench.kneel != 0.0 || !(bench.crouch > 0.3)) throw 'The planner knelt for a bench a crouch reaches: kneel ${bench.kneel}, crouch ${bench.crouch}';
		var tooLow = new HumanJob(body).add(new ApproachFor([0.6, 0.0, 0.05], ArmR));
		tooLow.advance(1.0 / 60.0);
		if (tooLow.failure() == null || tooLow.failure().indexOf("even kneeling") < 0)
			throw 'A point on the floor did not say it is out of reach kneeling: ${tooLow.failure()}';
		// A kneel stands back up: the body is not left kneeling when it walks.
		body.setKneel(1.0);
		for (_ in 0...200) body.advance(1.0 / 60.0);
		if (body.kneelAmount() < 0.99) throw "The body did not kneel when asked";
		body.setKneel(0.0);
		for (_ in 0...200) body.advance(1.0 / 60.0);
		if (body.kneelAmount() != 0.0 || human.kneel() != 0.0) throw "The body did not stand up from a kneel";
		human.dispose();
	}

	/** A posture's lengths grow with the body; its angles, fractions and times do not. */
	static function postureStature():Void {
		var standard = HumanPosture.standard(), same = HumanPosture.forStature(HumanPosture.REFERENCE_STATURE);
		if (same.bellyFront != standard.bellyFront || same.carryOffset[0] != standard.carryOffset[0])
			throw "The reference stature changed the posture";
		var tall = HumanPosture.forStature(HumanPosture.REFERENCE_STATURE * 1.5);
		if (Math.abs(tall.bellyFront - standard.bellyFront * 1.5) > 1e-12 || Math.abs(tall.carryOffset[2] - standard.carryOffset[2] * 1.5) > 1e-12 ||
			Math.abs(tall.blendSpeed - standard.blendSpeed * 1.5) > 1e-12)
			throw "A taller body's posture lengths did not grow with it";
		if (tall.maxLean != standard.maxLean || tall.stretch != standard.stretch || tall.comfort != standard.comfort || tall.edgeGap != standard.edgeGap ||
			tall.hangSeconds != standard.hangSeconds)
			throw "A taller body's posture angles, fractions or times changed";
		var rejected = false;
		try HumanPosture.forStature(0.0) catch (_:Dynamic) rejected = true;
		if (!rejected) throw "A posture for no stature was accepted";
	}

	/** A facility's surfaces and slot items become boxes in its frame, turned with their owners. */
	static function facilitySurfaces():Void {
		var facility = new Facility("surfaces", "Surfaces");
		facility.addZone(new Zone("floor", "Floor", "map", Footprint.rectangle(12, 12)));
		// A rack turned a quarter turn, its top offset along its own x axis, holding a 10 x 8 x 6 cm item.
		var rack = new Rack("rack", "Rack", "floor", "map", new Pose2(2, 1, Math.PI / 2),
			[new RackSlot("B3", new RackSlotPose(0.25, 0, 1.2), [0.05, 0.04, 0.03])], new Surface(0.3, 0.2, 1.1, 0.1, 0.0, 0.25));
		// A station's table lies ahead of where the worker stands.
		var table = new Station("table", "Table", "floor", "map", new Pose2(4, 1, 0.0), new Surface(0.2, 0.2, 1.0, 0.65));
		var bare = new Station("bare", "Bare", "floor", "map", new Pose2(6, 1));
		facility.addRack(rack);
		facility.addStation(table);
		facility.addStation(bare);
		facility.addLane(new Lane("rack-table", "rack", "table", new Path([rack.pose, new Pose2(3, 1), table.pose], "map"), 1, 1));
		var targets = new FacilityTargets(facility);
		var top = targets.surfaceBox("rack");
		if (top == null || distance(top.center, [2.0, 1.1, 1.05]) > 1e-9 || Math.abs(top.yaw - (Math.PI / 2 + 0.25)) > 1e-9 ||
			top.halfExtents[0] != 0.3 || top.halfExtents[1] != 0.2)
			throw 'The rack top was not turned with the rack: $top';
		var bench = targets.surfaceBox("table");
		if (bench == null || distance(bench.center, [4.65, 1.0, 0.95]) > 1e-9 || bench.yaw != 0.0)
			throw 'The table did not lie ahead of its station: $bench';
		if (Math.abs(bench.center[2] + bench.halfExtents[2] - 1.0) > 1e-9)
			throw "The table box's top face is not at the surface height";
		if (targets.surfaceBox("bare") != null) throw "A station without a surface reported one";
		var item = targets.slotItemBox("rack", "B3");
		if (item == null || distance(item.center, [2.0, 1.25, 1.2]) > 1e-9 || item.halfExtents[0] != 0.05 || item.halfExtents[2] != 0.03)
			throw 'The slot item box did not resolve through the rack: $item';
		var threw = false;
		try targets.surfaceBox("missing") catch (_:Dynamic) threw = true;
		if (!threw) throw "An unknown station was accepted";
		// With no place point given, the part is set down on the station's surface, at its middle and resting on it.
		var place:Place = cast FacilityJobs.fetch(facility, "rack", "B3").deliver("table").orderedActions()[4];
		if (distance(place.target, [4.65, 1.0, 1.03]) > 1e-9)
			throw 'The part was not set down on the station surface: ${place.target}';
		var undescribed = false;
		try FacilityJobs.fetch(facility, "rack", "B3").deliver("bare") catch (_:Dynamic) undescribed = true;
		if (!undescribed) throw "A delivery with no place point and no station surface was accepted";
	}

	/** Rack-relative slots resolve in facility space and jobs use routed lanes. */
	static function facilityTargets(scene:Scene, asset:AnimationAsset, rig:HumanoidRig):Void {
		var facility = new Facility("fetch", "Fetch facility");
		facility.addZone(new Zone("floor", "Floor", "map", Footprint.rectangle(12, 12)));
		var dock = new Station("dock", "Dock", "floor", "map", new Pose2(0, 0));
		var rack = new Rack("rack", "Rack", "floor", "map", new Pose2(2, 1, Math.PI / 2),
			[new RackSlot("B3", new RackSlotPose(0.25, 0, 1.2))]);
		var table = new Station("table", "Table", "floor", "map", new Pose2(4, 1));
		facility.addStation(dock);
		facility.addRack(rack);
		facility.addStation(table);
		facility.addLane(new Lane("dock-rack", "dock", "rack",
			new Path([dock.pose, new Pose2(2, 0), rack.pose], "map"), 1, 1));
		facility.addLane(new Lane("rack-table", "rack", "table",
			new Path([rack.pose, new Pose2(3, 1), table.pose], "map"), 1, 1));
		var targets = new FacilityTargets(facility);
		var slot = targets.rackSlotPoint("rack", "B3");
		if (distance(slot, [2, 1.25, 1.2]) > 1e-9)
			throw 'Rack slot did not resolve through yaw: $slot';
		// A facility that describes no surface or item gives the job nothing to lean over or shape a grip to.
		if (targets.surfaceBox("rack") != null || targets.surfaceBox("table") != null || targets.slotItemBox("rack", "B3") != null)
			throw "A facility with no surfaces or items reported one";
		var route = targets.route("rack", "table");
		var points = FacilityWalk.routeFromFacilityRoute(route);
		if (points.length != 3 || distance([points[1][0], points[1][1], 0], [3, 1, 0]) > 1e-9)
			throw 'Fetch route did not follow facility lane: $points';
		var human = new HumanCharacter(scene, asset, rig, null, "Fetcher");
		var body = new HumanBody(human);
		var placePoint = [4.25, 1.0, 1.2];
		var job = FacilityJobs.fetch(facility, "rack", "B3").deliver("table", placePoint);
		job.bind(body);
		var checkedSlot = false, checkedPlace = false;
		for (_ in 0...1200) {
			job.advance(1.0 / 60.0);
			if (!checkedSlot && job.currentIndex() >= 1) {
				checkedSlot = true;
				var shoulder = body.toWorld(human.pose.bonePosition(HumanBone.UpperArmR));
				if (distance(shoulder, slot) > body.description.upperArm + body.description.forearm + 0.02)
					throw 'Fetch approach stopped outside slot reach: $shoulder to $slot';
			}
			if (!checkedPlace && job.currentIndex() >= 4) {
				checkedPlace = true;
				var hand = body.toWorld(human.pose.bonePosition(HumanBone.HandR));
				var reach = body.description.upperArm + body.description.forearm;
				if (distance(hand, placePoint) >= 0.9 * reach)
					throw 'Delivery did not approach the place point: hand=$hand target=$placePoint';
			}
			if (job.isDone()) break;
		}
		if (!checkedSlot || !checkedPlace || !job.isDone() || job.failure() != null)
			throw 'Facility fetch did not finish: ${job.failure()}';
		human.dispose();
	}

	/**
	 * A HumanReachTask walks to a spot, stops, reaches for a target with the IK
	 * weight ramped in and out (playing "interact" while holding), and releases
	 * cleanly.
	 */
	static function reachTask(scene:Scene, asset:AnimationAsset, rig:HumanoidRig):Void {
		var human = new HumanCharacter(scene, asset, rig, null, "ReachTasker");
		var walker = new HumanWalker(human);
		human.advance(0.0);
		var shoulder = human.pose.bonePosition(HumanBone.UpperArmR);
		var description = HumanDescription.measure(human.pose, human.height());
		var reachLength = description.upperArm + description.forearm;
		var direction = Mat4.normalize([1.0, -0.2, -0.3]);
		var target = [for (axis in 0...3) shoulder[axis] + direction[axis] * reachLength * 0.8];
		var task = new HumanReachTask(walker, [[0.0, 0.0], [1.5, 0.0]], 1.2, ArmR, target, 0.3, 0.4, null,
			"interact", 0.2);
		var step = 1.0 / 60.0;
		var checkedHold = false;
		for (_ in 0...400) {
			task.advance(step);
			if (task.phase == Holding && !checkedHold) {
				checkedHold = true;
				if (walker.isWalking())
					throw "The reach task is still walking while holding";
				var root = walker.rootTransform();
				if (Math.abs(root[12] - 1.5) > 1e-3 || Math.abs(root[13]) > 1e-3)
					throw 'The body did not stop at the spot: ${root[12]}, ${root[13]}';
				var hand = human.pose.bonePosition(HumanBone.HandR);
				if (distance(hand, target) > 0.01)
					throw 'The wrist did not reach the target: $hand vs $target';
				if (human.player.currentClip() != asset.clipIndex("interact"))
					throw "The interact clip is not playing while holding";
			}
			if (task.isDone())
				break;
		}
		if (!checkedHold)
			throw "The reach task never held its target";
		if (!task.isDone())
			throw "The reach task never finished";
		// A clean release: the IK no longer pins the wrist to the target.
		for (_ in 0...12)
			task.advance(step);
		var released = human.pose.bonePosition(HumanBone.HandR);
		if (distance(released, target) < 0.02)
			throw "The reach did not release cleanly; the wrist is still pinned to the target";
		var resetRoute = new HumanReachTask(walker, [[2.0, 1.0], [2.5, 1.0]], 1.0,
			ArmR, target, 0.2, 0.1);
		var resetRoot = walker.rootTransform();
		if (Math.abs(resetRoot[12] - 2.0) > 1e-5 || Math.abs(resetRoot[13] - 1.0) > 1e-5)
			throw "Reach task did not start at the first route point";
		resetRoute.advance(step);
		human.dispose();
	}

	/** World-space actions, persistent carry IK, failure, and cancellation. */
	static function jobs(scene:Scene, asset:AnimationAsset, rig:HumanoidRig):Void {
		var human = new HumanCharacter(scene, asset, rig, null, "JobWorker");
		var body = new HumanBody(human);
		body.advance(0.0);
		var shoulder = human.pose.bonePosition(HumanBone.UpperArmR);
		var pickPoint = body.toWorld([shoulder[0] + 0.32, shoulder[1] - 0.03, shoulder[2] - 0.18]);
		var approach = new ApproachFor(pickPoint, ArmR);
		var pick = new Pick(pickPoint, [ArmR], 0.25);
		var placePoint = [1.7, -0.15, pickPoint[2]];
		var job = new HumanJob(body)
			.add(approach)
			.add(pick)
			.add(new Carry(HumanCarryPosture.RightHand))
			.add(new WalkTo([1.2, 0.0], 1.0))
			.add(new ApproachFor(placePoint, ArmR))
			.add(new Place(placePoint, [ArmR], 0.2))
			.add(new Press(placePoint, ArmR, 0.1, 0.1))
			.add(new Wait(0.05))
			.add(new PlayClip("wave", 0.1));
		var approached = false, picked = false, carried = false, advanced = false, fullSpeed = false;
		var carriedSteps = 0;
		var step = 1.0 / 60.0;
		for (_ in 0...900) {
			var before = body.rootTransform()[12];
			var headingBefore = Math.atan2(body.rootTransform()[1], body.rootTransform()[0]);
			job.advance(step);
			// Walks continue from the current heading: the body never turns faster than its turn rate.
			var turned = Math.atan2(body.rootTransform()[1], body.rootTransform()[0]) - headingBefore;
			turned = Math.abs(Math.atan2(Math.sin(turned), Math.cos(turned)));
			if (turned > body.walker.turnRate * step + 1e-6)
				throw 'The body snapped its heading by $turned rad in one step';
			if (!approached && job.currentIndex() > 0) {
				approached = true;
				// The target lies straight ahead of the reaching shoulder, within reach.
				var root = body.rootTransform();
				var shoulderNow = body.toWorld(human.pose.bonePosition(HumanBone.UpperArmR));
				var dx = pickPoint[0] - shoulderNow[0], dy = pickPoint[1] - shoulderNow[1];
				var range = Math.sqrt(dx * dx + dy * dy);
				var reachLength = body.description.upperArm + body.description.forearm;
				if (distance(shoulderNow, pickPoint) > reachLength ||
					(root[0] * dx + root[1] * dy) / range < 0.99)
					throw 'Approach did not stand within reach facing the target: $root';
			}
			if (pick.grip && !picked && !body.isCarrying(ArmR)) {
				picked = true;
				var palm = body.gripPoint(ArmR);
				if (distance(palm, pickPoint) > 0.02 || pick.pickError > 0.02)
					throw 'The pick palm missed its world target: $palm vs $pickPoint (${pick.pickError})';
			}
			carriedSteps = body.isCarrying(ArmR) ? carriedSteps + 1 : 0;
			// The hand eases into the carry pose over a third of a second, then holds it.
			if (body.isCarrying(ArmR) && body.walker.isWalking() && carriedSteps > 25) {
				carried = true;
				// The fingers closed on what the hand carries.
				if (human.handCurl(ArmR) < 0.45) throw 'The carrying hand is only ${human.handCurl(ArmR)} curled';
				var hand = human.pose.bonePosition(HumanBone.HandR);
				if (distance(hand, body.carryTargetModel(ArmR)) > 0.03)
					throw 'The carrying hand left its chest-relative pose: $hand';
				var delta = body.rootTransform()[12] - before;
				if (delta > 1e-4) advanced = true;
				if (delta / step > 0.8 && delta / step < 1.1) fullSpeed = true;
			}
			if (job.isDone()) break;
		}
		if (!approached || !picked || !carried || !advanced || !fullSpeed || !job.isDone() ||
			job.failure() != null || body.grip)
			throw 'The action job did not finish physically: approach=$approached pick=$picked carry=$carried walk=$advanced failure=${job.failure()}';
		// With the job over, the hand relaxes from its grip.
		for (_ in 0...30) body.advance(step);
		if (Math.abs(human.handCurl(ArmR) - 0.25) > 0.02)
			throw 'The hand did not relax after the job: ${human.handCurl(ArmR)}';
		body.setCarry([ArmL, ArmR]);
		// Both hands ease into the carry pose, then hold it.
		for (_ in 0...30) body.advance(step);
		for (hand in [ArmL, ArmR]) {
			var bone = hand == ArmL ? HumanBone.HandL : HumanBone.HandR;
			if (distance(human.pose.bonePosition(bone), body.carryTargetModel(hand)) > 0.03)
				throw 'Two-hand carry missed the $hand hand target';
		}
		body.setCarry([ArmL]);
		body.advance(0.0);
		if (body.isCarrying(ArmR) || body.reachTargetWorld(ArmR) != null ||
			distance(human.pose.bonePosition(HumanBone.HandR), body.carryTargetModel(ArmR)) < 0.04)
			throw "Dropped right hand remained locked in its carry pose";
		body.setCarry([]);
		body.advance(0.0);
		if (body.isCarrying(ArmL) || body.reachTargetWorld(ArmL) != null ||
			distance(human.pose.bonePosition(HumanBone.HandL), body.carryTargetModel(ArmL)) < 0.04)
			throw "Dropped left hand remained locked in its carry pose";
		var malformed = new HumanJob(body).add(WalkTo.along([[0.0, 0.0], [1.0]], 1.0));
		malformed.advance(step);
		if (malformed.failure() == null || malformed.failure().indexOf("finite x and y") < 0)
			throw "Malformed route point was accepted";
		human.dispose();

		var unreachableHuman = new HumanCharacter(scene, asset, rig, null, "UnreachableWorker");
		var unreachableBody = new HumanBody(unreachableHuman);
		var unreachable = new HumanJob(unreachableBody).add(new ApproachFor([0.5, 0.0, 5.0], ArmR));
		unreachable.advance(step);
		var reason = unreachable.failure();
		if (!unreachable.isDone() || reason == null || reason.indexOf("Action 0") < 0 ||
			reason.indexOf("above reachable") < 0)
			throw 'An unreachable approach did not explain its failure: ${unreachable.failure()}';
		var tooLow = new HumanJob(unreachableBody).add(new ApproachFor([0.5, 0.0, 0.1], ArmR));
		tooLow.advance(step);
		var lowReason = tooLow.failure();
		if (lowReason == null || lowReason.indexOf("below standing arm reach") < 0)
			throw 'A low target did not explain its failure: $lowReason';
		var cancel = new HumanJob(unreachableBody).add(new Reach(ArmR, [0.4, -0.2, 1.3], 0.1))
			.add(new Wait(1.0)).add(new ReleaseLimb(ArmR, 0.1));
		for (_ in 0...10) cancel.advance(step);
		cancel.cancel();
		unreachableBody.advance(0.0);
		if (unreachableBody.reachWeight(ArmR) != 0.0 ||
			unreachableBody.reachTargetWorld(ArmR) != null || unreachableBody.walker.isWalking())
			throw "Cancelling a job left IK or walking active";
		unreachableHuman.dispose();
	}

	static function distance(a:Array<Float>, b:Array<Float>):Float {
		var delta = Mat4.subtract(a, b);
		return Math.sqrt(Mat4.dot(delta, delta));
	}

	static function inRange(value:Float, low:Float, high:Float, label:String):Void
		if (!(value >= low && value <= high))
			throw 'Implausible $label $value';

	static function capsuleIndex(proxy:HumanBodyProxy, name:String):Int {
		for (index in 0...proxy.capsules.length)
			if (proxy.capsules[index].name == name)
				return index;
		throw 'No $name capsule';
	}

	/** The capsule's local +Z axis in the placed frame. */
	static function capsuleAxis(placement:CapsulePlacement):Array<Float> {
		var q = placement.rotation;
		return [
			2.0 * (q[0] * q[2] + q[3] * q[1]),
			2.0 * (q[1] * q[2] - q[3] * q[0]),
			1.0 - 2.0 * (q[0] * q[0] + q[1] * q[1])
		];
	}

	/** A hemisphere centre: the capsule's +Z end for sign 1, its -Z end for -1. */
	static function capsuleEnd(capsule:HumanCapsule, placement:CapsulePlacement, sign:Float):Array<Float> {
		var axis = capsuleAxis(placement), half = capsule.length * 0.5 * sign;
		return [
			placement.center[0] + axis[0] * half,
			placement.center[1] + axis[1] * half,
			placement.center[2] + axis[2] * half
		];
	}

	static function drawCalls(scene:Scene):Int {
		var snapshot = scene.snapshot();
		var renderer = SceneRenderer.createHeadless();
		var stats = renderer.render(snapshot, new SceneView());
		snapshot.dispose();
		return haxe.Int64.toInt(stats.get_draw_calls());
	}

	static function checkRigid(m:Array<Float>, label:String):Void {
		var x = [m[0], m[1], m[2]], y = [m[4], m[5], m[6]], z = [m[8], m[9], m[10]];
		if (Math.abs(Mat4.dot(x, x) - 1) > 1e-4 || Math.abs(Mat4.dot(y, y) - 1) > 1e-4 || Math.abs(Mat4.dot(x, y)) > 1e-4
			|| Mat4.dot(Mat4.cross(x, y), z) < 0.999)
			throw '$label is not a rigid right-handed frame';
	}

	static function assetDir():String {
		var configured = Sys.getEnv("ANIMKIT_ASSET_DIR");
		for (root in configured != null ? [configured] : ["animkit/assets", "../../animkit/assets", "../animkit/assets"])
			if (FileSystem.exists(root + "/quaternius/worker.glb"))
				return root;
		throw "Cannot find animkit/assets; set ANIMKIT_ASSET_DIR";
	}
}
