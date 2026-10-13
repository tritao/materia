package processkit.pressbrake;

/** Immutable robot request image; the script and wired robot use the same handshake. */
class PressBrakeRobotRequest {
	public final requestStroke:Bool;
	public final requestClampOnly:Bool;
	public final requestOpen:Bool;
	public final programStep:Int;
	public final clear:Bool;
	public final holding:Bool;
	public final faultReset:Bool;
	public final safetyReady:Bool;

	public function new(requestStroke:Bool = false,
		requestClampOnly:Bool = false,
		requestOpen:Bool = false,
		programStep:Int = 0,
		clear:Bool = false,
		holding:Bool = false,
		faultReset:Bool = false,
		safetyReady:Bool = true) {
		this.requestStroke = requestStroke;
		this.requestClampOnly = requestClampOnly;
		this.requestOpen = requestOpen;
		this.programStep = programStep;
		this.clear = clear;
		this.holding = holding;
		this.faultReset = faultReset;
		this.safetyReady = safetyReady;
	}
}
