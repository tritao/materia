import animkit.AnimationAsset;
import humankit.HumanBone;
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
