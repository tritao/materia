package animkit;

import AnimKitNative;
import haxe.io.Bytes;

/**
 * One posed copy of an asset. Set up to four blend layers, evaluate, then read
 * the CPU-skinned vertex streams and joint matrices in SceneKit space.
 */
class AnimationInstance {
	public static inline var MAX_LAYERS:Int = 4;
	public static inline var MAX_IK_CHAINS:Int = 4;

	public final asset:AnimationAsset;
	final owner:Ownedak_instance_handle;
	var disposed:Bool = false;

	/** Creates an instance at the rest pose, already evaluated. */
	public function new(asset:AnimationAsset) {
		this.asset = asset;
		var created = AnimKitNative.ak_instance_create(asset.handle());
		AnimationAsset.check(created.status, "instance.create");
		owner = created.out_instance;
	}

	/**
	 * Sets one blend layer. A layer contributes when clip is valid and weight is
	 * positive; weights are normalized across layers. Time is in seconds and
	 * wraps when loop is set, otherwise it clamps to the clip.
	 */
	public function setLayer(layer:Int, clip:Int, time:Float, weight:Float, loop:Bool = true):Void {
		AnimationAsset.check(AnimKitNative.ak_instance_set_layer(handle(), layer, clip, time, weight,
			loop ? 1 : 0), "instance.layer");
	}

	/**
	 * Sets one of MAX_IK_CHAINS two-bone chains solved after the layers blend:
	 * end reaches target and mid points along pole, both [x, y, z] in scene
	 * space (pole is a direction; a zero pole bends the limb the way the animation
	 * does, which never flips as the target moves). Weight 0 disables the chain.
	 */
	public function setIk(chain:Int, start:Int, mid:Int, end:Int, target:Array<Float>, pole:Array<Float>,
			weight:Float = 1.0, soften:Float = 1.0):Void {
		var ik = new ak_two_bone_ik();
		ik.set_struct_size(ak_two_bone_ik.size());
		ik.set_start_joint(start);
		ik.set_mid_joint(mid);
		ik.set_end_joint(end);
		for (axis in 0...3) {
			ik.set_target(axis, target[axis]);
			ik.set_pole(axis, pole[axis]);
		}
		ik.set_weight(weight);
		ik.set_soften(soften);
		AnimationAsset.check(AnimKitNative.ak_instance_set_ik(handle(), chain, ik), "instance.ik");
	}

	/** Disables one inverse kinematics chain. */
	public function clearIk(chain:Int):Void
		AnimationAsset.check(AnimKitNative.ak_instance_set_ik(handle(), chain, null), "instance.ik");

	/** Clears every layer, returning the instance to the rest pose on evaluate. */
	public function clearLayers():Void {
		for (layer in 0...MAX_LAYERS)
			setLayer(layer, -1, 0.0, 0.0);
	}

	/** Samples, blends, and skins the current layers. */
	public function evaluate():Void
		AnimationAsset.check(AnimKitNative.ak_instance_evaluate(handle()), "instance.evaluate");

	/** Deformed float32 xyz positions of one primitive. */
	public function readPositions(primitive:Int):Bytes {
		var read = AnimKitNative.ak_instance_read_positions(handle(), primitive);
		AnimationAsset.check(read.status, "instance.positions");
		return read.data;
	}

	/** Deformed unit float32 xyz normals of one primitive. */
	public function readNormals(primitive:Int):Bytes {
		var read = AnimKitNative.ak_instance_read_normals(handle(), primitive);
		AnimationAsset.check(read.status, "instance.normals");
		return read.data;
	}

	/** One column-major float32 4x4 model matrix per joint, 64 bytes each. */
	public function readJointMatrices():Bytes {
		var read = AnimKitNative.ak_instance_read_joint_matrices(handle());
		AnimationAsset.check(read.status, "instance.joints");
		return read.data;
	}

	/** Deformed bounds as [minX, minY, minZ, maxX, maxY, maxZ]. */
	public function bounds():Array<Float> {
		var value = new ak_bounds();
		value.set_struct_size(ak_bounds.size());
		AnimationAsset.check(AnimKitNative.ak_instance_get_bounds(handle(), value), "instance.bounds");
		return [value.get_minimum(0), value.get_minimum(1), value.get_minimum(2),
			value.get_maximum(0), value.get_maximum(1), value.get_maximum(2)];
	}

	public function dispose():Void {
		if (disposed)
			return;
		disposed = true;
		owner.close();
	}

	function handle():ak_instance_handle {
		if (disposed)
			throw "Animation instance has been disposed";
		return owner.borrow();
	}
}
