package app.editor;

import animkit.AnimationAsset;
import animkit.AnimationInstance;
import animkit.ClipPlayer;
import animkit.scene.SkinnedModel;
import humankit.HumanBone;
import humankit.HumanCharacter;
import humankit.HumanGrip;
import humankit.HumanoidRig;
import nativekit.scene.NodeId;
import nativekit.scene.Transform;

/**
 * Runtime-only animated glTF character for previewing AnimKit and HumanKit in
 * the editor. It is not part of the document: it walks a circle around the
 * origin and is re-created when the editor replaces its scene. Humanoid rigs
 * become HumanKit characters and can hold a prop in the right hand; other
 * assets play as plain animated models.
 */
class CharacterPreview {
	static inline var PATH_RADIUS:Float = 2.5;
	/** Walking speed in body heights per second, a relaxed human pace. */
	static inline var WALK_HEIGHTS_PER_SECOND:Float = 0.75;
	/** Where a held prop is gripped, as a fraction of its length from its origin. */
	static inline var GRIP_FRACTION:Float = 0.4;

	final path:String;
	final clipName:Null<String>;
	final asset:AnimationAsset;
	final rig:Null<HumanoidRig>;
	final prop:Null<AnimationAsset>;
	var human:Null<HumanCharacter> = null;
	var instance:Null<AnimationInstance> = null;
	var player:Null<ClipPlayer> = null;
	var model:Null<SkinnedModel> = null;
	var owner:Null<EditorScene> = null;
	var updatedNodes:Array<NodeId> = [];
	var moving:Bool = false;
	var walkSpeed:Float = 1.0;
	var angle:Float = 0.0;
	var lastTime:Float = -1.0;

	/** Loads the character, and optionally a prop for its right hand. */
	public function new(path:String, ?clipName:String, ?propPath:String) {
		this.path = path;
		this.clipName = clipName;
		asset = AnimationAsset.load(path);
		for (warning in asset.warnings)
			Sys.println('materia: $path: $warning');
		var detected:Null<HumanoidRig> = null;
		try {
			detected = HumanoidRig.detect(asset);
		} catch (error:String) {
			Sys.println('materia: $path is not a humanoid rig; previewing it as a plain model');
		}
		rig = detected;
		prop = propPath == null ? null : AnimationAsset.load(propPath);
		if (prop != null && rig == null)
			Sys.println('materia: --character-hold needs a humanoid character; ignoring $propPath');
	}

	/** Advances the animation by wall-clock time and publishes it into scene. */
	public function advance(scene:EditorScene):Void {
		if (owner != scene) attach(scene);
		var now = Sys.time();
		var elapsed = lastTime < 0.0 ? 0.0 : Math.min(now - lastTime, 0.1);
		lastTime = now;
		var root:NodeId;
		var character = human;
		if (character != null) {
			character.advance(elapsed);
			root = character.root;
		} else {
			var presented = model;
			if (presented == null)
				return;
			player.advance(elapsed);
			presented.update();
			root = presented.root;
		}
		if (moving) {
			angle += elapsed * walkSpeed / PATH_RADIUS;
			var transaction = scene.runtimeContentScene().beginTransaction();
			transaction.setTransform(root, pathTransform());
			transaction.commit();
		}
		scene.publishRuntimeNodes(updatedNodes);
	}

	function attach(scene:EditorScene):Void {
		// A replaced editor scene disposed the previous nodes with it.
		releaseInstances();
		owner = scene;
		var sceneKit = scene.runtimeContentScene();
		var root:NodeId;
		if (rig != null) {
			var created = new HumanCharacter(sceneKit, asset, rig, null, "Character preview");
			if (prop != null)
				created.attach(prop, HumanBone.HandR, HumanGrip.handle(gripPoint(prop)), "Held prop");
			human = created;
			player = created.player;
			instance = created.instance;
			updatedNodes = created.changedNodes();
			root = created.root;
		} else {
			var created = new AnimationInstance(asset);
			instance = created;
			player = new ClipPlayer(created);
			var presented = new SkinnedModel(sceneKit, created, null, "Character preview");
			model = presented;
			updatedNodes = [presented.root].concat(presented.primitiveNodes);
			root = presented.root;
		}
		startClip();
		var transaction = sceneKit.beginTransaction();
		transaction.setTransform(root, pathTransform());
		transaction.commit();
	}

	/** Grips a prop part-way along its +X extent. */
	static function gripPoint(prop:AnimationAsset):Float {
		var rest = new AnimationInstance(prop);
		var bounds = rest.bounds();
		rest.dispose();
		return bounds[0] + (bounds[3] - bounds[0]) * GRIP_FRACTION;
	}

	function startClip():Void {
		var name = clipName != null ? clipName : "walk";
		if (!player.playNamed(name, 0.0) && asset.clipNames.length > 0) {
			if (clipName != null)
				Sys.println('materia: $path has no clip "$clipName"; playing "${asset.clipNames[0]}"');
			player.play(0, 0.0);
		}
		var current = player.currentClip();
		moving = current >= 0 && asset.clipNames[current].toLowerCase().indexOf("walk") >= 0;
		var bounds = instance.bounds();
		walkSpeed = WALK_HEIGHTS_PER_SECOND * Math.max(bounds[5] - bounds[2], 0.1);
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

	function releaseInstances():Void {
		var character = human;
		if (character != null)
			character.dispose();
		else if (instance != null)
			instance.dispose();
		human = null;
		instance = null;
		model = null;
	}

	public function dispose():Void {
		releaseInstances();
		if (prop != null)
			prop.dispose();
		asset.dispose();
	}
}
