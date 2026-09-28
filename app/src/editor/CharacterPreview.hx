package app.editor;

import animkit.AnimationAsset;
import animkit.AnimationInstance;
import animkit.ClipPlayer;
import animkit.scene.SkinnedModel;
import nativekit.scene.NodeId;
import nativekit.scene.Transform;

/**
 * Runtime-only animated glTF character for previewing AnimKit in the editor.
 * It is not part of the document: it walks a circle around the origin and is
 * re-created when the editor replaces its scene.
 */
class CharacterPreview {
	static inline var PATH_RADIUS:Float = 2.5;
	static inline var WALK_SPEED:Float = 0.6;

	final asset:AnimationAsset;
	final instance:AnimationInstance;
	final player:ClipPlayer;
	final moving:Bool;
	var owner:Null<EditorScene> = null;
	var model:Null<SkinnedModel> = null;
	var updatedNodes:Array<NodeId> = [];
	var angle:Float = 0.0;
	var lastTime:Float = -1.0;

	/** Loads the asset and starts clipName, or the first clip when it is absent. */
	public function new(path:String, ?clipName:String) {
		asset = AnimationAsset.load(path);
		for (warning in asset.warnings)
			Sys.println('materia: $path: $warning');
		instance = new AnimationInstance(asset);
		player = new ClipPlayer(instance);
		var name = clipName != null ? clipName : "walk";
		if (!player.playNamed(name, 0.0) && asset.clipNames.length > 0) {
			if (clipName != null)
				Sys.println('materia: $path has no clip "$clipName"; playing "${asset.clipNames[0]}"');
			player.play(0, 0.0);
		}
		var current = player.currentClip();
		moving = current >= 0 && asset.clipNames[current].toLowerCase().indexOf("walk") >= 0;
	}

	/** Advances the animation by wall-clock time and publishes it into scene. */
	public function advance(scene:EditorScene):Void {
		if (owner != scene) attach(scene);
		var now = Sys.time();
		var elapsed = lastTime < 0.0 ? 0.0 : Math.min(now - lastTime, 0.1);
		lastTime = now;
		player.advance(elapsed);
		var current = model;
		if (current == null) return;
		current.update();
		if (moving) {
			angle += elapsed * WALK_SPEED / PATH_RADIUS;
			var transaction = scene.runtimeContentScene().beginTransaction();
			transaction.setTransform(current.root, pathTransform());
			transaction.commit();
		}
		scene.publishRuntimeNodes(updatedNodes);
	}

	function attach(scene:EditorScene):Void {
		// A replaced editor scene disposed the previous model's nodes with it.
		owner = scene;
		var created = new SkinnedModel(scene.runtimeContentScene(), instance, null, "Character preview");
		model = created;
		updatedNodes = [created.root].concat(created.primitiveNodes);
		var transaction = scene.runtimeContentScene().beginTransaction();
		transaction.setTransform(created.root, pathTransform());
		transaction.commit();
	}

	/** Places the character on the circle, facing along it; AnimKit characters face +X. */
	function pathTransform():Transform {
		if (!moving)
			return Transform.identity().translated(0.0, -PATH_RADIUS, 0.0);
		var c = Math.cos(angle), s = Math.sin(angle);
		// Tangent of a counter-clockwise circle is (-sin, cos); +Z stays up.
		return Transform.identity()
			.set(0, -s).set(1, c)
			.set(4, -c).set(5, -s)
			.translated(PATH_RADIUS * c, PATH_RADIUS * s, 0.0);
	}

	public function dispose():Void {
		instance.dispose();
		asset.dispose();
	}
}
