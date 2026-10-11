package processkit.pressbrake;

/** Pure press-side handshake; progress follows observations, never nominal flow time. */
class PressBrakeInterface {
	public final requestTimeoutSeconds:Float;
	var phase:PressBrakePhase = Idle;
	var alarm:PressBrakeAlarm = None;
	var elapsed:Float = 0;
	var previous:Null<PressBrakeInput>;
	var step:Int = -1;
	var clampOnly:Bool = false;

	public function new(requestTimeoutSeconds:Float) {
		if (!Math.isFinite(requestTimeoutSeconds) || requestTimeoutSeconds <= 0)
			throw "Press interface timeout must be finite and positive";
		this.requestTimeoutSeconds = requestTimeoutSeconds;
	}

	/** Consume one sampled image. A maintained request never starts a second stroke. */
	public function update(input:PressBrakeInput, dt:Float):PressBrakeOutputs {
		if (input == null || !Math.isFinite(dt) || dt <= 0)
			throw "Press interface needs finite positive controller time";
		var old = previous;
		var stroke = input.requestStroke && (old == null || !old.requestStroke);
		var clamp = input.requestClampOnly && (old == null || !old.requestClampOnly);
		var open = input.requestOpen && (old == null || !old.requestOpen);
		var reset = input.faultReset && (old == null || !old.faultReset);
		previous = input;
		var valid = Math.isFinite(input.ramPositionMm) && input.programStep >= 0 &&
			input.configuredStep >= 0 && !(input.atTop && (input.atClamp || input.atBottom)) &&
			!(input.atClamp && input.atBottom);
		if (alarm == None && !valid) fail(InvalidFeedback);
		if (alarm == None && !input.safetyReady) fail(SafetyInterlock);
		if (alarm != None && reset && valid && input.safetyReady && input.atTop &&
			input.decompressed && input.clear && !input.requestStroke &&
			!input.requestClampOnly && !input.requestOpen) {
			alarm = None;
			begin(Idle);
			step = -1;
		}
		if (alarm == None) {
			if ((stroke ? 1 : 0) + (clamp ? 1 : 0) + (open ? 1 : 0) > 1)
				fail(ConflictingRequests);
			else if (phase != Idle && (input.programStep != step || input.configuredStep != step))
				fail(ProgramChanged);
			else if (phase == ClosingToClamp && (!input.fingerLeft || !input.fingerRight))
				fail(RegistrationLost);
			else if (phase == ClosingToClamp && !input.holding) fail(RobotHoldLost);
			else if (phase != Idle && !input.depthStopInPosition) fail(PositioningLost);
			else if ((phase == ClosingToClamp || phase == Clamped) && !input.axesInPosition)
				fail(PositioningLost);
			else if (phase != Idle && (stroke || clamp)) fail(Busy);
		}
		if (alarm == None && (stroke || clamp)) {
			if (!input.atTop || !input.decompressed || !input.axesInPosition || !input.depthStopInPosition ||
				input.programStep != input.configuredStep || !input.fingerLeft ||
				!input.fingerRight || !input.holding) fail(StrokeInterlock);
			else {
				step = input.programStep;
				clampOnly = clamp;
				begin(ClosingToClamp);
			}
		}
		if (alarm == None && open) {
			if (!((phase == Clamped || phase == Withdrawing) && input.atClamp || phase == Bottom && input.atBottom) || (!input.holding && !input.clear))
				fail(OpenInterlock);
			else begin(Decompressing);
		}
		if (alarm == None) {
			var entered = phase;
			switch phase {
				case ClosingToClamp: if (input.atClamp) begin(clampOnly ? Clamped : Withdrawing);
				case Withdrawing: if (input.backgaugeWithdrawn) begin(Working);
				case Working: if (input.atBottom) begin(Bottom);
				case Decompressing: if (input.decompressed) begin(Opening);
				case Opening: if (input.atTop && input.decompressed) { begin(Idle); step = -1; }
				case _:
			}
			if (phase == entered && pending()) {
				elapsed += dt;
				if (elapsed >= requestTimeoutSeconds) fail(timeoutAlarm());
			}
		}
		var command:PressBrakeCommand = switch phase {
			case ClosingToClamp: CloseToClamp;
			case Working: CloseToDepth;
			case Decompressing: Decompress;
			case Opening: Open;
			case _: Hold;
		};
		var owned = phase == Clamped || phase == Withdrawing || phase == Working ||
			phase == Bottom || phase == Decompressing;
		return new PressBrakeOutputs(phase, alarm, command,
			alarm == None && phase == Idle && input.atTop && input.decompressed &&
				input.axesInPosition && input.depthStopInPosition && input.programStep == input.configuredStep,
			input.axesInPosition, input.atTop, owned, input.atBottom, input.decompressed,
			owned || phase == Opening, input.ramPositionMm);
	}

	function begin(next:PressBrakePhase):Void { phase = next; elapsed = 0; }
	function fail(code:PressBrakeAlarm):Void { alarm = code; begin(Fault); }
	function pending():Bool return switch phase {
		case ClosingToClamp, Withdrawing, Working, Decompressing, Opening: true;
		case _: false;
	};
	function timeoutAlarm():PressBrakeAlarm return switch phase {
		case ClosingToClamp: ClampTimeout;
		case Withdrawing: WithdrawalTimeout;
		case Working: DepthTimeout;
		case Decompressing: DecompressionTimeout;
		case Opening: OpenTimeout;
		case _: None;
	};
}
