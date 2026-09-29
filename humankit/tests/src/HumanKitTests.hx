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
import humankit.HumanCapsule;
import humankit.HumanDescription;
import humankit.HumanDisplay;
import humankit.HumanLimb;
import humankit.HumanReachTask;
import humankit.HumanWalker;
import humankit.HumanPose;
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
import materia.automation.facility.Zone;
import nativekit.scene.Scene;
import nativekit.scene.SceneRenderer;
import nativekit.scene.SceneView;
import robotkit.mobile.Footprint;
import robotkit.mobile.Pose2;
import robotkit.navigation.Path;
import sys.FileSystem;

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
		walking(scene, worker, rig);
		reaching(scene, worker, rig);
		facilityRoute(scene, worker, rig);
		facilityTargets(scene, worker, rig);
		reachTask(scene, worker, rig);
		placeReferencePoint(scene, worker, rig);
		jobs(scene, worker, rig);
		jobSpecs(scene, worker, rig);
		scene.dispose();
		Sys.println("humankit tests: ok");
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
			if (body.isCarrying(ArmR) && body.walker.isWalking()) {
				carried = true;
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
		body.setCarry([ArmL, ArmR]);
		body.advance(0.0);
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
