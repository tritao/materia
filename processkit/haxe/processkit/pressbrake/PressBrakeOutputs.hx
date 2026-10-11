package processkit.pressbrake;

/** The complete handshake image and requests after one controller update. */
class PressBrakeOutputs {
	public final phase:PressBrakePhase;
	public final alarm:PressBrakeAlarm;
	public final command:PressBrakeCommand;
	public final ready:Bool;
	public final backgaugeInPosition:Bool;
	public final atTop:Bool;
	public final clamping:Bool;
	public final atBottom:Bool;
	public final decompressed:Bool;
	public final retractBackgauge:Bool;
	public final ramPositionMm:Float;

	public function new(phase:PressBrakePhase,
		alarm:PressBrakeAlarm,
		command:PressBrakeCommand,
		ready:Bool,
		backgaugeInPosition:Bool,
		atTop:Bool,
		clamping:Bool,
		atBottom:Bool,
		decompressed:Bool,
		retractBackgauge:Bool,
		ramPositionMm:Float) {
		this.phase = phase;
		this.alarm = alarm;
		this.command = command;
		this.ready = ready;
		this.backgaugeInPosition = backgaugeInPosition;
		this.atTop = atTop;
		this.clamping = clamping;
		this.atBottom = atBottom;
		this.decompressed = decompressed;
		this.retractBackgauge = retractBackgauge;
		this.ramPositionMm = ramPositionMm;
	}
}
