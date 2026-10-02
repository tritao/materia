package humankit.action;

import humankit.HumanBody;
import humankit.HumanLimb;
import humankit.HumanTargetBox;

/**
 * Walks to where the limb's shoulder has a comfortable reach to a point and
 * faces so the point lies straight ahead of that shoulder, before a reach.
 * The stand-off accounts for the height between shoulder and point: a low
 * point is approached closer than one at shoulder height.
 *
 * Given the surface the point lies on, the worker also stands far enough back
 * that the belly stays clear of its near edge, and leans the upper body
 * forward over it to keep the shoulder where the reach is comfortable.
 */
class ApproachFor extends HumanActionBase {
	public var target(default, null):Array<Float>;
	public final limb:HumanLimb;
	public final speed:Float;
	public final bothHands:Bool;
	/**
	 * How far short of the surface's edge the belly ends, in metres, once the lean and the stretch are spent:
	 * zero when the stance clears it. A surface too deep for the arm to reach over (a crouch does not help: it
	 * lowers the shoulder, not carries it over the edge), or a body that cannot crouch at a low one, leaves this above
	 * zero; the job still runs, with the belly over the edge by about this much.
	 */
	public var shortfall(default, null):Float = 0.0;
	/** How deep a crouch the worker takes at the stand (0 upright, 1 the clip's full crouch), once it has arrived. */
	public var crouch(default, null):Float = 0.0;
	/** How deep a kneel the worker takes instead of a crouch, for a point too low for one (0 when it does not kneel). */
	public var kneel(default, null):Float = 0.0;
	/** How far the upper body leans over the surface at the stand, in radians, once the worker has arrived. */
	public var lean(default, null):Float = 0.0;
	/** How far it bends at the hips on top of that, for a surface too deep to reach across by leaning alone. */
	public var hinge(default, null):Float = 0.0;
	var postureIssued:Bool = false;
	/** The walk to the stand, held back while the body stands up from a crouch. */
	var waiting:Null<Array<Array<Float>>> = null;
	var faceAngle:Float = 0.0;
	var turnIssued:Bool = false;
	final targetProvider:Null<Void->Array<Float>>;
	final support:Null<HumanTargetBox>;

	public function new(target:Array<Float>, limb:HumanLimb, speed:Float = 1.0, bothHands:Bool = false,
		?targetProvider:Void->Array<Float>, ?support:HumanTargetBox) {
		super();
		this.support = support;
		this.target = target.copy();
		this.limb = limb;
		this.speed = speed;
		this.bothHands = bothHands;
		this.targetProvider = targetProvider;
	}

	override public function start(worker:HumanBody):Void {
		super.start(worker);
		if (targetProvider != null) target = targetProvider().copy();
		if (target.length < 3) { fail("Approach target needs x, y, z"); return; }
		if (limb != ArmL && limb != ArmR) { fail("Approach requires an arm"); return; }
		var chosen = new StancePlanner(worker, target, limb, bothHands, support).plan();
		var failure = chosen.failure;
		if (failure != null) { fail(failure); return; }
		shortfall = chosen.shortfall;
		crouch = chosen.crouch;
		kneel = chosen.kneel;
		lean = chosen.lean;
		hinge = chosen.hinge;
		worker.setLean(lean);
		worker.setArmsDown(support != null);
		var root = worker.rootTransform();
		var standX = target[0] - chosen.ux * chosen.standDistance + chosen.uy * chosen.lateral;
		var standY = target[1] - chosen.uy * chosen.standDistance - chosen.ux * chosen.lateral;
		faceAngle = Math.atan2(chosen.uy, chosen.ux) - (bothHands ? worker.standingTwist() : 0.0);
		if (Math.sqrt(Math.pow(standX - root[12], 2) + Math.pow(standY - root[13], 2)) > 0.005) {
			var route = [[root[12], root[13]], [standX, standY]];
			// A crouched body does not walk: it stands up first, and the walk starts when it has.
			if (worker.downAmount() > 1e-3 || !worker.downReached()) {
				worker.setCrouch(0.0);
				worker.setKneel(0.0);
				waiting = route;
			} else
				worker.walker.continueAlong(route, speed);
		} else {
			worker.walker.face(faceAngle);
			turnIssued = true;
		}
	}

	override public function advance(seconds:Float):Void {
		var route = waiting;
		if (route != null && worker.downAmount() <= 1e-3) {
			waiting = null;
			worker.walker.continueAlong(route, speed);
		}
		if (!done && waiting == null && !turnIssued && !worker.walker.isWalking()) {
			worker.walker.face(faceAngle);
			turnIssued = true;
		}
		// The body lowers and leans once it stands where it will work and has turned to face the point, not on the
		// way there, and the reach waits until it has: a shoulder still on its way would put the point out of reach.
		if (!done && turnIssued && !postureIssued && !worker.walker.isTurning()) {
			postureIssued = true;
			worker.setHinge(hinge);
			worker.setCrouch(crouch);
			worker.setKneel(kneel);
		}
	}

	override public function isDone():Bool
		return done || (turnIssued && !worker.walker.isTurning() && postureIssued && worker.downReached() && worker.leanReached() && worker.walker.settled());
}
