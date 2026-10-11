package processkit.pressbrake;

/** One immutable controller image; ram travel is millimetres downward from top. */
class PressBrakeInput {
	public final requestStroke:Bool;
	public final requestClampOnly:Bool;
	public final requestOpen:Bool;
	public final programStep:Int;
	public final clear:Bool;
	public final holding:Bool;
	public final faultReset:Bool;
	public final safetyReady:Bool;
	public final fingerLeft:Bool;
	public final fingerRight:Bool;
	public final configuredStep:Int;
	public final axesInPosition:Bool;
	public final depthStopInPosition:Bool;
	public final backgaugeWithdrawn:Bool;
	public final atTop:Bool;
	public final atClamp:Bool;
	public final atBottom:Bool;
	public final decompressed:Bool;
	public final ramPositionMm:Float;

	public function new(requestStroke:Bool,
		requestClampOnly:Bool,
		requestOpen:Bool,
		programStep:Int,
		clear:Bool,
		holding:Bool,
		faultReset:Bool,
		safetyReady:Bool,
		fingerLeft:Bool,
		fingerRight:Bool,
		configuredStep:Int,
		axesInPosition:Bool,
		depthStopInPosition:Bool,
		backgaugeWithdrawn:Bool,
		atTop:Bool,
		atClamp:Bool,
		atBottom:Bool,
		decompressed:Bool,
		ramPositionMm:Float) {
		this.requestStroke = requestStroke;
		this.requestClampOnly = requestClampOnly;
		this.requestOpen = requestOpen;
		this.programStep = programStep;
		this.clear = clear;
		this.holding = holding;
		this.faultReset = faultReset;
		this.safetyReady = safetyReady;
		this.fingerLeft = fingerLeft;
		this.fingerRight = fingerRight;
		this.configuredStep = configuredStep;
		this.axesInPosition = axesInPosition;
		this.depthStopInPosition = depthStopInPosition;
		this.backgaugeWithdrawn = backgaugeWithdrawn;
		this.atTop = atTop;
		this.atClamp = atClamp;
		this.atBottom = atBottom;
		this.decompressed = decompressed;
		this.ramPositionMm = ramPositionMm;
	}
}
