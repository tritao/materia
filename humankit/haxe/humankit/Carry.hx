package humankit;

/** Establishes a persistent carry posture for subsequent walking actions. */
class Carry extends HumanActionBase {
	public final posture:HumanCarryPosture;

	public function new(posture:HumanCarryPosture) {
		super();
		this.posture = posture;
	}

	override public function start(worker:HumanBody):Void {
		super.start(worker);
		var hands:Array<HumanLimb> = switch posture {
			case HumanCarryPosture.None: [];
			case HumanCarryPosture.LeftHand: [ArmL];
			case HumanCarryPosture.RightHand: [ArmR];
			case HumanCarryPosture.BothHands: [ArmL, ArmR];
			default: [];
		};
		worker.setCarry(hands);
		done = true;
	}
}
