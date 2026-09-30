package humankit;

import animkit.AnimationAsset;
import animkit.AnimationInstance;
import animkit.ClipPlayer;
import animkit.scene.SkinnedModel;
import nativekit.scene.NodeId;
import nativekit.scene.Scene;
import nativekit.scene.Transform;

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
		instance.setIk(limb, rig.joint(bones[0]), rig.joint(bones[1]), rig.joint(bones[2]), target,
			pole != null ? pole : arm ? [0.0, 0.0, 0.0] : [1.0, 0.0, 0.0], weight);
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

	/** Advances the current clip and moves the mesh and attachments to the new pose. */
	public function advance(seconds:Float):Void {
		player.advance(seconds);
		model.update();
		pose.update(instance.readJointMatrices());
		placeAttachments();
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
