package humankit;

/**
 * A person's collision stand-in: fifteen capsules between standard bones
 * (head, abdomen, chest, and three segments per limb), sized from a
 * HumanDescription and placed from a HumanPose every frame. Simulations use it
 * where the skinned mesh would be too costly to collide with.
 */
class HumanBodyProxy {
	public final capsules:Array<HumanCapsule>;
	public final description:HumanDescription;

	function new(description:HumanDescription, capsules:Array<HumanCapsule>) {
		this.description = description;
		this.capsules = capsules;
	}

	/**
	 * The standard proxy for a rig, measured at its rest pose. Radii are
	 * proportions of stature; capsule lengths follow the rig's bones, scaled
	 * by description.scale like the character's visual root.
	 */
	public static function standard(rest:HumanPose, description:HumanDescription):HumanBodyProxy {
		var s = description.stature;
		var capsules:Array<HumanCapsule> = [];
		function add(name:String, from:HumanBone, to:HumanBone, extension:Float, radius:Float, trim:Float = 0.0):Void {
			var span = segmentLength(rest, from, to) * (1.0 + extension) * description.scale - trim;
			capsules.push(new HumanCapsule(name, from, to, extension, trim, radius, span));
		}
		// The head reaches the crown: extend neck-to-head up to stature minus
		// the head's radius, measured in the rig's own units.
		var headRadius = 0.055 * s;
		var neck = requirePosition(rest, Neck), head = requirePosition(rest, Head);
		var restStature = s / description.scale, restRadius = headRadius / description.scale;
		var up = head[2] - neck[2];
		var headExtension = up > 1e-6 ? Math.max(0.0, (restStature - restRadius - head[2]) / up) : 0.0;
		add("head", Neck, Head, headExtension, headRadius);
		var waist:HumanBone = rest.rig.has(Spine2) ? Spine2 : rest.rig.has(Spine) ? Spine : Chest;
		add("abdomen", Pelvis, waist, 0.0, 0.068 * s);
		add("chest", waist, Neck, 0.0, 0.074 * s);
		for (side in [0, 1]) {
			var left = side == 0;
			var suffix = left ? ".L" : ".R";
			add("upper_arm" + suffix, left ? UpperArmL : UpperArmR, left ? ForearmL : ForearmR, 0.0, 0.029 * s);
			add("forearm" + suffix, left ? ForearmL : ForearmR, left ? HandL : HandR, 0.0, 0.023 * s);
			var knuckle:HumanBone = left ? MiddleL : MiddleR;
			if (rest.rig.has(knuckle))
				add("hand" + suffix, left ? HandL : HandR, knuckle, 0.8, 0.02 * s);
			add("thigh" + suffix, left ? ThighL : ThighR, left ? ShinL : ShinR, 0.0, 0.043 * s);
			// The foot covers the ankle; a shin reaching it would sink below the sole.
			add("shin" + suffix, left ? ShinL : ShinR, left ? FootL : FootR, 0.0, 0.031 * s, 0.031 * s);
			// The foot runs from the ankle to the toe tip, or past the ball of
			// the foot by the toes' typical length when the rig has no tip.
			var tip:HumanBone = left ? ToeTipL : ToeTipR, toe:HumanBone = left ? ToeL : ToeR;
			if (rest.rig.has(tip))
				add("foot" + suffix, left ? FootL : FootR, tip, 0.0, 0.026 * s);
			else if (rest.rig.has(toe))
				add("foot" + suffix, left ? FootL : FootR, toe, 0.3, 0.026 * s);
		}
		return new HumanBodyProxy(description, capsules);
	}

	/** Every capsule's placement for a pose, through a root transform (column-major 4x4). */
	public function place(pose:HumanPose, ?root:Array<Float>):Array<CapsulePlacement> {
		var transform = root != null ? root : Mat4.identity();
		return [
			for (capsule in capsules) {
				var from = transformPoint(transform, requirePosition(pose, capsule.from));
				var to = transformPoint(transform, requirePosition(pose, capsule.to));
				var axis = Mat4.subtract(to, from);
				var unit = Mat4.normalize(axis);
				var end = [
					to[0] + axis[0] * capsule.extension - unit[0] * capsule.trim,
					to[1] + axis[1] * capsule.extension - unit[1] * capsule.trim,
					to[2] + axis[2] * capsule.extension - unit[2] * capsule.trim
				];
				new CapsulePlacement([(from[0] + end[0]) * 0.5, (from[1] + end[1]) * 0.5, (from[2] + end[2]) * 0.5],
					rotationFromZ(unit));
			}
		];
	}

	/** The rotation (x, y, z, w) taking +Z onto a unit direction by the shortest arc. */
	public static function rotationFromZ(direction:Array<Float>):Array<Float> {
		var w = 1.0 + direction[2];
		if (w < 1e-9)
			return [1.0, 0.0, 0.0, 0.0];
		// cross(+Z, direction) = (-dy, dx, 0)
		var x = -direction[1], y = direction[0];
		var norm = Math.sqrt(x * x + y * y + w * w);
		return [x / norm, y / norm, 0.0, w / norm];
	}

	static function transformPoint(m:Array<Float>, p:Array<Float>):Array<Float>
		return [
			m[0] * p[0] + m[4] * p[1] + m[8] * p[2] + m[12],
			m[1] * p[0] + m[5] * p[1] + m[9] * p[2] + m[13],
			m[2] * p[0] + m[6] * p[1] + m[10] * p[2] + m[14]
		];

	static function segmentLength(pose:HumanPose, from:HumanBone, to:HumanBone):Float {
		var delta = Mat4.subtract(requirePosition(pose, to), requirePosition(pose, from));
		return Math.sqrt(Mat4.dot(delta, delta));
	}

	static function requirePosition(pose:HumanPose, bone:HumanBone):Array<Float> {
		var position = pose.bonePosition(bone);
		if (position == null)
			throw 'The rig lacks $bone';
		return position;
	}
}
