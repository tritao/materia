package app.editor;

import animkit.AnimationAsset;
import animkit.AnimationInstance;
import animkit.ClipPlayer;
import animkit.scene.SkinnedModel;
import app.SessionParticipant;
import humankit.HumanBodyProxy;
import humankit.HumanBodyView;
import humankit.HumanBone;
import humankit.HumanCharacter;
import humankit.HumanDescription;
import humankit.HumanDisplay;
import humankit.HumanGrip;
import humankit.HumanPose;
import humankit.HumanoidRig;
import humankit.sim.HumanActor;
import nativekit.scene.NodeId;
import nativekit.scene.Transform;
import nativekit.sim.SimSession;

/**
 * Runtime-only animated glTF character for previewing AnimKit and HumanKit in
 * the editor. It is not part of the document: it walks a circle around the
 * origin and is re-created when the editor replaces its scene. Humanoid rigs
 * become HumanKit characters and can hold a prop in the right hand; other
 * assets play as plain animated models.
 *
 * A humanoid also takes part in the application simulation as a person: its
 * body proxy joins every session as a kinematic actor, and while a session is
 * active the character lives on simulation time, a few ticks ahead of the
 * physics so the actor always has a keyframe to move towards. It stands still
 * while the simulation is paused and walks by the wall clock without one.
 * A humanoid can be drawn as its mesh, its collision capsules, or its skeleton.
 */
class CharacterPreview implements SessionParticipant {
	static inline var PATH_RADIUS:Float = 2.5;
	/** Walking speed in body heights per second, a relaxed human pace. */
	static inline var WALK_HEIGHTS_PER_SECOND:Float = 0.75;
	/** Where a held prop is gripped, as a fraction of its length from its origin. */
	static inline var GRIP_FRACTION:Float = 0.4;
	/** How many ticks ahead of the physics the character's keyframes run. */
	static inline var LEAD_TICKS:Int = 3;

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
	/** The humanoid's rest pose and collision proxy, or null for other assets. */
	final restPose:Null<HumanPose>;
	final proxy:Null<HumanBodyProxy>;
	final display:HumanDisplay;
	var view:Null<HumanBodyView> = null;
	var session:Null<SimSession> = null;
	var actor:Null<HumanActor> = null;
	/** Simulation time the character was last advanced to. */
	var simulationTime:Float = 0.0;

	/** Loads the character, and optionally a prop for its right hand. */
	public function new(path:String, ?clipName:String, ?propPath:String, display:HumanDisplay = Mesh) {
		this.path = path;
		this.display = display;
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
		var measuredPose:Null<HumanPose> = null, measuredProxy:Null<HumanBodyProxy> = null;
		if (detected != null) {
			var rest = new AnimationInstance(asset);
			var bounds = rest.bounds();
			measuredPose = new HumanPose(detected, rest.readJointMatrices());
			rest.dispose();
			measuredProxy = HumanBodyProxy.standard(measuredPose,
				HumanDescription.measure(measuredPose, bounds[5] - bounds[2]));
		}
		restPose = measuredPose;
		proxy = measuredProxy;
		prop = propPath == null ? null : AnimationAsset.load(propPath);
		if (prop != null && rig == null)
			Sys.println('materia: --character-hold needs a humanoid character; ignoring $propPath');
	}

	/** A humanoid's body joins a new, stopped session where the character stands. */
	public function join(joined:SimSession):Void {
		session = joined;
		simulationTime = 0.0;
		var body = proxy, rest = restPose;
		var character = human;
		if (body == null || rest == null) {
			actor = null;
			return;
		}
		actor = new HumanActor(joined, body, character != null ? character.pose : rest, rootMatrix());
	}

	public function leave():Void {
		session = null;
		actor = null;
		lastTime = -1.0;
	}

	/**
	 * Advances the animation, by simulation time while a session is active and
	 * by wall-clock time otherwise, and publishes it into scene.
	 */
	public function advance(scene:EditorScene):Void {
		if (owner != scene) attach(scene);
		var elapsed:Float;
		var active = session;
		if (active != null) {
			var target = active.simulationTime() + LEAD_TICKS * active.fixedTimestep();
			// A reset rewinds simulation time: walk again from the start.
			if (target < simulationTime) {
				angle = 0.0;
				simulationTime = 0.0;
			}
			elapsed = target - simulationTime;
			simulationTime = target;
		} else {
			var now = Sys.time();
			elapsed = lastTime < 0.0 ? 0.0 : Math.min(now - lastTime, 0.1);
			lastTime = now;
		}
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
		var body = actor, drawn = view;
		if (body != null && character != null)
			body.pushPose(simulationTime, character.pose, rootMatrix());
		if (drawn != null && character != null)
			drawn.update(character.pose);
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
			var body = proxy;
			if (display != Mesh && body != null) {
				// The capsules or skeleton replace the skinned mesh; held props stay.
				var drawn = new HumanBodyView(sceneKit, created.root, body, created.pose);
				drawn.show(display);
				view = drawn;
				updatedNodes = updatedNodes.concat(drawn.nodes());
				var hide = sceneKit.beginTransaction();
				for (node in created.model.primitiveNodes)
					hide.setVisibility(node, false);
				hide.commit();
			}
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

	/** The character root's placement as a column-major matrix. */
	function rootMatrix():Array<Float> {
		var transform = pathTransform();
		return [for (index in 0...16) transform.element(index)];
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
		view = null;
	}

	public function dispose():Void {
		releaseInstances();
		if (prop != null)
			prop.dispose();
		asset.dispose();
	}
}
