import animkit.AnimationAsset;
import humankit.CapsulePlacement;
import humankit.HumanBodyProxy;
import humankit.HumanBone;
import humankit.HumanCapsule;
import humankit.HumanDescription;
import humankit.HumanPose;
import humankit.HumanCharacter;
import humankit.HumanoidRig;
import humankit.Mat4;
import humankit.RigMapping;
import nativekit.scene.Scene;
import nativekit.scene.SceneRenderer;
import nativekit.scene.SceneView;
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
		human.dispose();
		scene.dispose();
		Sys.println("humankit tests: ok");
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
