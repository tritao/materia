package humankit;

/** Stands at arm's reach of a point and faces it before the reach begins. */
class ApproachFor extends HumanActionBase {
	public final target:Array<Float>;
	public final limb:HumanLimb;
	public final speed:Float;
	var faceAngle:Float = 0.0;
	var turnIssued:Bool = false;

	public function new(target:Array<Float>, limb:HumanLimb, speed:Float = 1.0) {
		super();
		this.target = target.copy();
		this.limb = limb;
		this.speed = speed;
	}

	override public function start(worker:HumanBody):Void {
		super.start(worker);
		if (target.length < 3) { fail("Approach target needs x, y, z"); return; }
		if (limb != ArmL && limb != ArmR) { fail("Approach requires an arm"); return; }
		var shoulder = worker.character.pose.bonePosition(limb == ArmL ? UpperArmL : UpperArmR);
		var pelvis = worker.character.pose.bonePosition(Pelvis);
		if (shoulder == null || pelvis == null) { fail("The rig lacks an arm or pelvis"); return; }
		var reach = worker.description.upperArm + worker.description.forearm;
		var localTarget = worker.toModel(target);
		if (localTarget[2] > shoulder[2] + reach * 0.9) {
			fail("Target is above reachable height");
			return;
		}
		if (localTarget[2] < pelvis[2] - 0.05) {
			fail("Target is below waist height; crouching is unsupported");
			return;
		}
		var root = worker.rootTransform();
		var dx = target[0] - root[12], dy = target[1] - root[13];
		var distance = Math.sqrt(dx * dx + dy * dy);
		var ux = distance > 1e-8 ? dx / distance : root[0];
		var uy = distance > 1e-8 ? dy / distance : root[1];
		var standOff = Math.max(0.2, reach * 0.72);
		var standX = target[0] - ux * standOff;
		var standY = target[1] - uy * standOff;
		faceAngle = Math.atan2(target[1] - standY, target[0] - standX);
		if (Math.sqrt(Math.pow(standX - root[12], 2) + Math.pow(standY - root[13], 2)) > 0.005)
			worker.walker.follow([[root[12], root[13]], [standX, standY]], speed);
		else {
			worker.walker.face(faceAngle);
			turnIssued = true;
		}
	}

	override public function advance(seconds:Float):Void {
		if (!done && !turnIssued && !worker.walker.isWalking()) {
			worker.walker.face(faceAngle);
			turnIssued = true;
		}
	}

	override public function isDone():Bool
		return done || (turnIssued && !worker.walker.isTurning());
}
