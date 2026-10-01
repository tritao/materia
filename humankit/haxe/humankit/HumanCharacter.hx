package humankit;

import animkit.AnimationAsset;
import animkit.AnimationInstance;
import animkit.ClipPlayer;
import animkit.scene.SkinnedModel;
import nativekit.scene.NodeId;
import nativekit.scene.Scene;
import nativekit.scene.Transform;
import humankit.rig.HumanBone;
import humankit.rig.HumanPose;
import humankit.rig.HumanoidRig;
import humankit.rig.Mat4;

/** A rigid object carried by a bone, such as a tool in a hand. */
class HumanAttachment {
	public final bone:HumanBone;
	/** Node that follows the bone; the prop hangs beneath it. */
	public final node:NodeId;
	/** Transform of the prop in the bone's frame. */
	public var offset:Array<Float>;
	public final instance:AnimationInstance;
	public final model:SkinnedModel;

	public function new(bone:HumanBone, node:NodeId, offset:Array<Float>, instance:AnimationInstance,
			model:SkinnedModel) {
		this.bone = bone;
		this.node = node;
		this.offset = offset;
		this.instance = instance;
		this.model = model;
	}
}

/**
 * An animated humanoid in a SceneKit scene: the skinned mesh, its standard
 * skeleton pose, and rigid attachments that follow bones. Place it by
 * transforming root; advance() plays the current clip and moves everything.
 */
class HumanCharacter {
	public final asset:AnimationAsset;
	public final instance:AnimationInstance;
	public final player:ClipPlayer;
	public final rig:HumanoidRig;
	public final pose:HumanPose;
	public final model:SkinnedModel;
	public final attachments:Array<HumanAttachment> = [];
	/** The left and right hands' fingers; null for a rig without them. */
	final hands:Array<Null<HumanHand>>;
	var lean:Float = 0.0;
	var hinge:Float = 0.0;
	/** The asset's crouching-in-place clip, or -1 when it has none; see setCrouch. */
	final crouchClip:Int;
	/** The going-down clip the crouch is posed from, when the asset has one; else the crouch clip is blended in. */
	final crouchDown:Null<HumanCrouch>;
	var crouchDepth:Float = 0.0;
	/** The going-down part of a kneeling clip, when the asset has one; a kneel is posed from it as a crouch is from the crouch clip. */
	final kneelDown:Null<HumanCrouch>;
	var kneelDepth:Float = 0.0;
	/**
	 * The joint-turn sources each feature applies its turns under (see AnimationInstance.setJointRotation),
	 * so features that turn the same joint compose instead of overwriting one another. A feature added
	 * later takes the next number; lower sources apply first.
	 */
	static inline var LEAN:Int = 1;
	static inline var LEFT_FINGERS:Int = 2;
	static inline var RIGHT_FINGERS:Int = 3;
	static inline var HINGE:Int = 4;
	public var root(get, never):NodeId;

	final scene:Scene;

	/** Presents asset in scene; the rig is detected from its joint names unless given. */
	public function new(scene:Scene, asset:AnimationAsset, ?rig:HumanoidRig, ?parent:NodeId, ?name:String) {
		this.scene = scene;
		this.asset = asset;
		this.rig = rig != null ? rig : HumanoidRig.detect(asset);
		instance = new AnimationInstance(asset);
		pose = new HumanPose(this.rig, instance.readJointMatrices());
		player = new ClipPlayer(instance);
		crouchDown = HumanCrouch.measure(asset, this.rig, asset.clipIndex("crouch_enter"));
		crouchClip = crouchDown != null ? crouchDown.clip : asset.clipIndex("crouch_idle");
		kneelDown = HumanCrouch.measure(asset, this.rig, asset.clipIndex("pickup_kneeling"), true);
		hands = [HumanHand.find(asset, this.rig, instance, HumanBone.HandL, LEFT_FINGERS, pose),
			HumanHand.find(asset, this.rig, instance, HumanBone.HandR, RIGHT_FINGERS, pose)];
		model = new SkinnedModel(scene, instance, parent, name != null ? name : "Human");
	}

	function get_root():NodeId
		return model.root;

	/** Model height at the rest pose, in metres. */
	public function height():Float {
		var bounds = instance.bounds();
		return bounds[5] - bounds[2];
	}

	/**
	 * Attaches a rigid prop to a bone. The prop is placed at offset in the
	 * bone's frame (see HumanPose for the hand frame). The prop's asset must
	 * stay live until this call returns.
	 */
	public function attach(prop:AnimationAsset, bone:HumanBone, ?offset:Array<Float>, ?name:String):HumanAttachment {
		if (!rig.has(bone))
			throw 'Cannot attach to $bone: the rig lacks it';
		var transaction = scene.beginTransaction();
		var node = transaction.createNode();
		transaction.setParent(node, model.root);
		transaction.setName(node, name != null ? name : 'Attachment ($bone)');
		transaction.commit();
		var propInstance = new AnimationInstance(prop);
		var propModel = new SkinnedModel(scene, propInstance, node, name);
		var attachment = new HumanAttachment(bone, node, offset != null ? offset : Mat4.identity(), propInstance,
			propModel);
		attachments.push(attachment);
		placeAttachments();
		return attachment;
	}

	/**
	 * Reaches a limb's wrist or ankle for target ([x, y, z] in model space)
	 * on top of the animation, from the next advance on. The elbow or knee
	 * points along pole, a model-space direction: by default an elbow bends the
	 * way the animation bends it, which stays continuous wherever the hand goes,
	 * and knees point forward. weight blends from the animation (0) to the
	 * full reach (1). Throws when the rig's limb is not one chain, as with
	 * Quaternius legs, whose feet hang off the body as IK controls.
	 */
	public function reach(limb:HumanLimb, target:Array<Float>, weight:Float = 1.0, ?pole:Array<Float>):Void {
		var bones = limbBones(limb);
		var arm = limb == ArmL || limb == ArmR;
		// A foot reached to a spot keeps the orientation it was animated with, instead of tilting with the leg.
		instance.setIk(limb, rig.joint(bones[0]), rig.joint(bones[1]), rig.joint(bones[2]), target,
			pole != null ? pole : arm ? [0.0, 0.0, 0.0] : [1.0, 0.0, 0.0], weight, 1.0, arm ? 0.0 : 1.0);
	}

	/**
	 * Curls a hand's fingers, from open (0) to a fist (1), on top of the animation. A rig without finger
	 * joints ignores it. Takes effect from the next advance.
	 */
	public function setHandCurl(hand:HumanLimb, curl:Float):Void {
		if (hand != ArmL && hand != ArmR) throw "Only a hand has fingers to curl";
		var fingers = hands[hand == ArmL ? 0 : 1];
		if (fingers != null) fingers.setCurl(curl);
	}

	/**
	 * Leans the upper body forward by angle radians (0 upright), on top of the animation, by pitching
	 * the spine's upper joints together. Takes effect from the next advance.
	 */
	public function setSpineLean(angle:Float):Void {
		if (Math.abs(angle - lean) < 1e-5) return;
		lean = angle;
		var joints:Array<Int> = [], rotations:Array<Array<Float>> = [], weights:Array<Float> = [];
		var bones = [rig.joint(HumanBone.Spine2), rig.joint(HumanBone.Chest)];
		// Mostly the chest joint, which sits about at table height: the belly below stays put and the chest
		// can overhang a table, the way a person leans over one.
		var shares = [0.25, 0.75];
		for (index in 0...bones.length) {
			var turn = angle * shares[index];
			if (bones[index] < 0 || Math.abs(turn) < 1e-5) continue;
			joints.push(bones[index]);
			rotations.push([Math.sin(turn * 0.5), 0.0, 0.0, Math.cos(turn * 0.5)]);
			weights.push(1.0);
		}
		if (joints.length == 0) instance.clearJointRotations(LEAN);
		else instance.setJointRotations(LEAN, joints, rotations, weights);
	}

	public function spineLean():Float
		return lean;

	/**
	 * Bends the body forward at the hips by angle radians (0 upright), on top of the animation and of any lean: the whole trunk
	 * pitches about the lowest spine joint, so the shoulders travel far forward and down, as when someone bends over a table to
	 * reach across it. A rig without that joint ignores it. Takes effect from the next advance.
	 */
	public function setSpineHinge(angle:Float):Void {
		if (Math.abs(angle - hinge) < 1e-5) return;
		hinge = angle;
		var joint = rig.joint(HumanBone.Spine);
		if (joint < 0 || Math.abs(angle) < 1e-5) instance.clearJointRotations(HINGE);
		else instance.setJointRotations(HINGE, [joint], [[Math.sin(angle * 0.5), 0.0, 0.0, Math.cos(angle * 0.5)]], [1.0]);
	}

	public function spineHinge():Float
		return hinge;

	/**
	 * Whether the legs are IK chains (thigh, shin and foot one below the other), so a foot can be held where it is.
	 * The bundled worker's feet hang off the body as controls instead, and cannot.
	 */
	public function legsAreChains():Bool {
		for (side in 0...2) {
			var thigh = rig.joint(side == 0 ? ThighL : ThighR), shin = rig.joint(side == 0 ? ShinL : ShinR), foot = rig.joint(side == 0 ? FootL : FootR);
			if (thigh < 0 || shin < 0 || foot < 0) return false;
			if (asset.jointParents[foot] != shin || asset.jointParents[shin] != thigh) return false;
		}
		return true;
	}

	/** Whether the asset has a crouch clip to lower the body with. */
	public function canCrouch():Bool
		return crouchClip >= 0;

	/**
	 * Lowers the body toward a crouch, by mixing the asset's crouching clip over the animation: 0 stands, 1 is
	 * the clip's full crouch. The legs and pelvis come from the clip, so the feet stay near the floor (they are not
	 * pinned: one may lift a few centimetres at full depth). Throws when the asset has no crouch clip. A crouch takes the
	 * place of a kneel, as a kneel does of a crouch. Takes effect from the next advance, and is meant for a worker standing still.
	 */
	public function setCrouch(amount:Float):Void {
		if (crouchClip < 0) throw "The character has no crouch clip";
		crouchDepth = Math.max(0.0, Math.min(1.0, amount));
		if (crouchDepth > 0.0) kneelDepth = 0.0;
		applyDown();
	}

	/**
	 * The overlay that shows the way down the body is in: a kneel if there is one, else a crouch, else none. Posed from the
	 * going-down clip, held at the time where the body is this far down, which is an authored pose at every depth;
	 * without a going-down clip the crouch clip is blended in at its depth as the weight.
	 */
	function applyDown():Void {
		var kneeling = kneelDown;
		if (kneeling != null && kneelDepth > 0.0) {
			player.setOverlay(kneeling.clip, Math.min(1.0, kneelDepth / 0.3), kneeling.timeFor(kneelDepth));
			return;
		}
		if (crouchDepth <= 0.0 || crouchClip < 0) {
			player.setOverlay(-1, 0.0);
			return;
		}
		var down = crouchDown;
		if (down != null) player.setOverlay(crouchClip, Math.min(1.0, crouchDepth / 0.3), down.timeFor(crouchDepth));
		else player.setOverlay(crouchClip, crouchDepth);
	}

	public function crouch():Float
		return crouchDepth;

	/** Whether the asset has a kneeling clip to lower the body further with than a crouch can. */
	public function canKneel():Bool
		return kneelDown != null;

	/**
	 * Lowers the body onto a knee, with the arm reaching down, by holding the going-down part of the asset's kneeling clip
	 * at the point where the pelvis is that fraction of the way down: 0 stands, 1 is the lowest the clip goes. It takes the
	 * place of a crouch (the two are different ways down, not stages of one), so asking for a kneel stands the crouch back up
	 * and the other way round. Throws when the asset has no kneeling clip.
	 */
	public function setKneel(amount:Float):Void {
		if (kneelDown == null) throw "The character has no kneeling clip";
		kneelDepth = Math.max(0.0, Math.min(1.0, amount));
		if (kneelDepth > 0.0) crouchDepth = 0.0;
		applyDown();
	}

	public function kneel():Float
		return kneelDepth;

	/** Sets how far down the body is in both ways at once (a kneel, if any, wins); for measuring and putting back. */
	public function setDown(crouchAmount:Float, kneelAmount:Float):Void {
		crouchDepth = crouchClip < 0 ? 0.0 : Math.max(0.0, Math.min(1.0, crouchAmount));
		kneelDepth = kneelDown == null ? 0.0 : Math.max(0.0, Math.min(1.0, kneelAmount));
		if (kneelDepth > 0.0) crouchDepth = 0.0;
		applyDown();
	}

	/**
	 * Curls each finger of a hand on its own: values are indexed HumanHand.THUMB to PINKY, each 0 open to
	 * 1 a fist. A rig without finger joints ignores it.
	 */
	public function setHandCurls(hand:HumanLimb, curls:Array<Float>):Void {
		if (hand != ArmL && hand != ArmR) throw "Only a hand has fingers to curl";
		var fingers = hands[hand == ArmL ? 0 : 1];
		if (fingers != null) fingers.setCurls(curls);
	}

	/** How curled each finger of a hand is (THUMB to PINKY), or all zero for a rig without them. */
	public function handCurls(hand:HumanLimb):Array<Float> {
		var fingers = hands[hand == ArmL ? 0 : 1];
		return [for (kind in 0...HumanHand.FINGERS) fingers == null ? 0.0 : fingers.curlOf(kind)];
	}

	/** The model-space position of a finger's tip on a hand (HumanHand.THUMB to PINKY), or null when it has none. */
	public function fingertip(hand:HumanLimb, finger:Int):Null<Array<Float>> {
		var fingers = hands[hand == ArmL ? 0 : 1];
		return fingers == null ? null : fingers.tipPosition(finger);
	}

	/** How curled a hand's fingers are, or 0 for a rig without them. */
	public function handCurl(hand:HumanLimb):Float {
		var fingers = hands[hand == ArmL ? 0 : 1];
		return fingers == null || fingers.curl < 0.0 ? 0.0 : fingers.curl;
	}

	/** Returns a limb to its animation. */
	public function release(limb:HumanLimb):Void
		instance.clearIk(limb);

	static function limbBones(limb:HumanLimb):Array<HumanBone>
		return switch limb {
			case ArmL: [UpperArmL, ForearmL, HandL];
			case ArmR: [UpperArmR, ForearmR, HandR];
			case LegL: [ThighL, ShinL, FootL];
			case LegR: [ThighR, ShinR, FootR];
			default: throw 'Unknown limb $limb';
		};

	/**
	 * Advances the current clip and applies every turn and reach, so `pose` describes the body now. The scene is not
	 * touched: a simulation advances many times for each frame it draws, and skinning and uploading a mesh nobody sees
	 * is the largest cost of doing it. Whoever draws the character calls `publish` once per frame.
	 */
	public function advance(seconds:Float):Void {
		player.pose(seconds);
		pose.update(instance.readJointMatrices());
	}

	/**
	 * Skins the mesh to the state the character is in now and moves it and its attachments in the scene. The state
	 * is the one the latest `advance` left (a measurement puts back what it changes), so a probe between an advance
	 * and a publish cannot reach the screen.
	 */
	public function publish():Void {
		instance.evaluate();
		model.update();
		placeAttachments();
	}

	/**
	 * Evaluates the pose as it would be now, at the current animation time and with every turn and reach applied, and
	 * makes `pose` current, without skinning the model or moving attachments. For measuring (what would the shoulder
	 * do under a lean, where would the hand be if released): nothing a measurement poses reaches the scene, and
	 * the next `advance` evaluates the pose again as usual.
	 */
	public function probe():Void {
		player.pose(0.0);
		pose.update(instance.readJointMatrices());
	}

	/** Nodes whose geometry or world transform changed in the last advance. */
	public function changedNodes():Array<NodeId> {
		var nodes = [model.root].concat(model.primitiveNodes);
		for (attachment in attachments)
			nodes = nodes.concat([attachment.node, attachment.model.root]).concat(attachment.model.primitiveNodes);
		return nodes;
	}

	function placeAttachments():Void {
		if (attachments.length == 0)
			return;
		var transaction = scene.beginTransaction();
		for (attachment in attachments) {
			var matrix = Mat4.multiply(pose.boneFrame(attachment.bone), attachment.offset);
			var transform = Transform.identity();
			for (i in 0...16)
				transform.set(i, matrix[i]);
			transaction.setTransform(attachment.node, transform);
		}
		transaction.commit();
	}

	public function dispose():Void {
		for (attachment in attachments)
			attachment.instance.dispose();
		instance.dispose();
	}
}
