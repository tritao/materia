package humankit;

/** Animation and locomotion state shared by every action in a worker's job. */
class HumanBody {
	public final character:HumanCharacter;
	public final walker:HumanWalker;
	public final description:HumanDescription;
	/** True between a completed pick and its matching place. */
	public var grip(default, null):Bool = false;

	final active:Array<Bool> = [false, false, false, false];
	final modelTargets:Array<Bool> = [false, false, false, false];
	final targets:Array<Array<Float>> = [[], [], [], []];
	final weights:Array<Float> = [0.0, 0.0, 0.0, 0.0];
	final poles:Array<Null<Array<Float>>> = [null, null, null, null];
	var carrying:Array<HumanLimb> = [];

	public function new(character:HumanCharacter, ?walker:HumanWalker) {
		this.character = character;
		this.walker = walker == null ? new HumanWalker(character) : walker;
		if (this.walker.character != character)
			throw "A human body needs its character's walker";
		description = HumanDescription.measure(character.pose, character.height());
	}

	public function rootTransform():Array<Float>
		return walker.rootTransform();

	public function toModel(worldPoint:Array<Float>):Array<Float> {
		var inverse = Mat4.rigidInverse(rootTransform());
		return transformPoint(inverse, worldPoint);
	}

	public function toWorld(modelPoint:Array<Float>):Array<Float>
		return transformPoint(rootTransform(), modelPoint);

	static function transformPoint(matrix:Array<Float>, point:Array<Float>):Array<Float>
		return [
			matrix[0] * point[0] + matrix[4] * point[1] + matrix[8] * point[2] + matrix[12],
			matrix[1] * point[0] + matrix[5] * point[1] + matrix[9] * point[2] + matrix[13],
			matrix[2] * point[0] + matrix[6] * point[1] + matrix[10] * point[2] + matrix[14]
		];

	public function setReachWorld(limb:HumanLimb, target:Array<Float>, weight:Float,
			?pole:Array<Float>):Void {
		var index:Int = limb;
		active[index] = true;
		modelTargets[index] = false;
		targets[index] = target.copy();
		weights[index] = Math.max(0.0, Math.min(1.0, weight));
		poles[index] = pole == null ? null : pole.copy();
	}

	/** Legacy model-space reach target used by HumanReachTask. */
	public function setReachModel(limb:HumanLimb, target:Array<Float>, weight:Float,
			?pole:Array<Float>):Void {
		var index:Int = limb;
		active[index] = true;
		modelTargets[index] = true;
		targets[index] = target.copy();
		weights[index] = Math.max(0.0, Math.min(1.0, weight));
		poles[index] = pole == null ? null : pole.copy();
	}

	public function reachWeight(limb:HumanLimb):Float
		return weights[limb];

	public function reachPole(limb:HumanLimb):Null<Array<Float>>
		return poles[limb];

	public function reachTargetWorld(limb:HumanLimb):Null<Array<Float>> {
		var index:Int = limb;
		if (!active[index]) return null;
		return modelTargets[index] ? toWorld(targets[index]) : targets[index].copy();
	}

	public function clearReach(limb:HumanLimb):Void {
		var index:Int = limb;
		active[index] = false;
		weights[index] = 0.0;
		poles[index] = null;
		character.release(limb);
	}

	public function setCarry(hands:Array<HumanLimb>):Void {
		for (hand in carrying) if (hands.indexOf(hand) < 0) clearReach(hand);
		carrying = hands.copy();
		for (hand in hands) clearReach(hand);
	}

	public function isCarrying(limb:HumanLimb):Bool
		return carrying.indexOf(limb) >= 0;

	/** Current carry target in model space, tied to the animated chest. */
	public function carryTargetModel(limb:HumanLimb):Array<Float> {
		var chest = character.pose.bonePosition(Chest);
		if (chest == null) chest = character.pose.bonePosition(Pelvis);
		if (chest == null) throw "The character has no chest or pelvis";
		var side = limb == ArmL ? 1.0 : -1.0;
		return [chest[0] + 0.24, chest[1] + side * 0.12, chest[2] - 0.22];
	}

	public function setGrip(held:Bool):Void
		grip = held;

	public function cancel():Void {
		walker.stop();
		carrying = [];
		grip = false;
		for (limb in [ArmL, ArmR, LegL, LegR]) clearReach(limb);
	}

	/** Advances gait once, then reapplies current world targets over that pose. */
	public function advance(seconds:Float):Void {
		walker.advance(seconds);
		var changed = false;
		for (limb in [ArmL, ArmR, LegL, LegR]) {
			var index:Int = limb;
			if (active[index]) {
				character.reach(limb, modelTargets[index] ? targets[index] : toModel(targets[index]),
					weights[index], poles[index]);
				changed = true;
			} else if (isCarrying(limb)) {
				character.reach(limb, carryTargetModel(limb));
				changed = true;
			}
		}
		if (changed) character.advance(0.0);
	}
}
