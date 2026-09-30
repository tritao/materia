import animkit.AnimationAsset;
import animkit.AnimationInstance;
import animkit.ClipPlayer;
import animkit.scene.SkinnedModel;
import nativekit.scene.Scene;
import sys.FileSystem;

class AnimKitTests {
	static function main():Void {
		var path = soldierPath();
		var asset = AnimationAsset.load(path);
		if (asset.warnings.length != 0)
			throw 'Unexpected import warnings: ${asset.warnings.join("; ")}';
		if (asset.primitives.length != 2 || !asset.primitives[0].skinned || !asset.primitives[0].hasTexcoords)
			throw "Soldier meshes did not import as skinned, textured primitives";
		if (asset.materials.length != 1 || asset.materials[0].baseColorImage != 0 || asset.imageCount != 1)
			throw "Soldier material did not keep its colormap texture";
		if (asset.imageWidth(0) <= 0 || asset.readImage(0).length != asset.imageWidth(0) * asset.imageHeight(0) * 4)
			throw "Soldier colormap did not decode to RGBA8";
		var walk = asset.clipIndex("walk");
		var idle = asset.clipIndex("idle");
		if (walk < 0 || idle < 0 || asset.clipDurations[walk] <= 0.0)
			throw "Soldier clips are missing";
		if (asset.readIndices(0).length != asset.primitives[0].indexCount * 4)
			throw "Index buffer size does not match the primitive";

		var instance = new AnimationInstance(asset);
		var rest = instance.bounds();
		// Kenney's soldier stands on the ground about 0.84 m tall, Z up.
		if (Math.abs(rest[2]) > 0.01 || rest[5] < 0.7 || rest[5] > 1.0)
			throw 'Unexpected rest bounds $rest';
		var restPositions = instance.readPositions(0);

		var player = new ClipPlayer(instance);
		player.play(walk, 0.0);
		player.advance(asset.clipDurations[walk] * 0.25);
		var walking = instance.readPositions(0);
		if (walking.compare(restPositions) == 0)
			throw "Walking did not deform the mesh";
		player.play(idle, 0.2);
		player.advance(0.1);
		if (player.currentClip() != idle)
			throw "Crossfade did not switch the current clip";
		if (instance.readJointMatrices().length != asset.jointNames.length * 64)
			throw "Joint matrix buffer has the wrong size";

		// A clip started in the middle of a crossfade continues from the blend it was in: the pose
		// must not snap to the current clip alone.
		player.restart(walk);
		player.advance(asset.clipDurations[walk] * 0.25);
		player.play(idle, 0.4);
		player.advance(0.2);
		var blended = instance.readJointMatrices();
		var settled = new ClipPlayer(instance);
		settled.restart(idle);
		settled.advance(0.2);
		var idleOnly = instance.readJointMatrices();
		if (matrixGap(blended, idleOnly) < 1e-3)
			throw "The mid-fade blend is indistinguishable from the current clip, so the test proves nothing";
		player.restart(walk);
		player.advance(asset.clipDurations[walk] * 0.25);
		player.play(idle, 0.4);
		player.advance(0.2);
		player.play(walk, 0.4);
		player.advance(0.0);
		var continued = instance.readJointMatrices();
		if (matrixGap(blended, continued) > 1e-3)
			throw 'Starting a clip mid-fade snapped the pose by ${matrixGap(blended, continued)}';

		var scene = Scene.create();
		var model = new SkinnedModel(scene, instance, null, "Soldier");
		if (model.primitiveNodes.length != 2)
			throw "Scene model did not create one node per primitive";
		// Instances keep posing after the asset is released; building models
		// reads the asset's static streams and images, so it happens first.
		asset.dispose();
		player.advance(0.5);
		model.update();
		scene.dispose();
		instance.dispose();

		var missing = false;
		try {
			AnimationAsset.load(path + ".missing");
		} catch (error:Dynamic) {
			missing = true;
		}
		if (!missing)
			throw "Loading a missing file did not fail";
		Sys.println("animkit haxe tests: ok");
	}

	static function matrixGap(a:haxe.io.Bytes, b:haxe.io.Bytes):Float {
		var worst = 0.0;
		for (index in 0...Std.int(a.length / 4)) worst = Math.max(worst, Math.abs(a.getFloat(index * 4) - b.getFloat(index * 4)));
		return worst;
	}

	static function soldierPath():String {
		var configured = Sys.getEnv("ANIMKIT_ASSET_DIR");
		var roots = configured != null ? [configured] : ["animkit/assets", "../assets", "assets"];
		for (root in roots) {
			var candidate = root + "/kenney/character-soldier.glb";
			if (FileSystem.exists(candidate))
				return candidate;
		}
		throw "Cannot find animkit/assets/kenney/character-soldier.glb; set ANIMKIT_ASSET_DIR";
	}
}
