import processkit.pressbrake.PressBrakeInterface;
import processkit.pressbrake.PressBrakeInput;
import processkit.pressbrake.PressBrakeOutputs;
import processkit.pressbrake.PressBrakeAlarm;
import processkit.pressbrake.PressBrakePhase;
import processkit.pressbrake.PressBrakeCommand;

/** Controller-image tests independent of flow speeds and simulation backends. */
class PressBrakeInterfaceTests {
	static var assertions:Int = 0;
	static function check(value:Bool, message:String):Void {
		assertions++;
		if (!value) throw message;
	}
	static function fresh():PressBrakeInterface return new PressBrakeInterface(0.05);
	static function tick(fsm:PressBrakeInterface, image:PressImage):PressBrakeOutputs
		return fsm.update(image.sample(), 0.01);
	static function fault(output:PressBrakeOutputs, expected:PressBrakeAlarm, message:String):Void {
		check(output.alarm == expected && output.phase == Fault && output.command == Hold && !output.ready,
			message + ': ${output.alarm}');
	}
	static function clamp(fsm:PressBrakeInterface, image:PressImage, only:Bool = false):PressBrakeOutputs {
		image.requestStroke = !only;
		image.requestClampOnly = only;
		var output = tick(fsm, image);
		check(output.command == CloseToClamp && !output.clamping, "valid registration starts approach");
		image.atTop = false; image.atClamp = true; image.decompressed = false; image.ramPositionMm = 78.5;
		output = tick(fsm, image);
		check(output.clamping && output.retractBackgauge && output.command == Hold,
			"measured pinch point holds while requesting backgauge withdrawal");
		return output;
	}
	static function bottom(fsm:PressBrakeInterface, image:PressImage):PressBrakeOutputs {
		var output = clamp(fsm, image);
		image.backgaugeWithdrawn = true; image.axesInPosition = false;
		// Registration is no longer required after the press takes ownership.
		image.fingerLeft = false; image.fingerRight = false; image.holding = false;
		output = tick(fsm, image);
		check(output.command == CloseToDepth && output.phase == Working, "working stroke waits for measured gauge withdrawal");
		image.atClamp = false; image.atBottom = true; image.ramPositionMm = 83;
		output = tick(fsm, image);
		check(output.phase == Bottom && output.atBottom && output.command == Hold, "bottom feedback stops closing");
		return output;
	}
	static function timeout(fsm:PressBrakeInterface, image:PressImage, expected:PressBrakeAlarm):Void {
		var output = tick(fsm, image);
		for (_ in 0...6) output = tick(fsm, image);
		fault(output, expected, "each unacknowledged phase reaches its named timeout");
	}
	public static function main():Void run();
	public static function run():Void {
		assertions = 0;
		var image = new PressImage(), fsm = fresh();
		var output = tick(fsm, image);
		check(output.ready && output.atTop && output.backgaugeInPosition && output.ramPositionMm == 0,
			"idle publishes the complete observed image");
		for (condition in 0...9) {
			image = new PressImage(); fsm = fresh(); image.requestStroke = true;
			switch condition {
				case 0: image.axesInPosition = false;
				case 1: image.fingerLeft = false;
				case 2: image.fingerRight = false;
				case 3: image.holding = false;
				case 4: image.atTop = false;
				case 5: image.decompressed = false;
				case 6: image.programStep = 1;
				case 7: image.safetyReady = false;
				case 8: image.depthStopInPosition = false;
			}
			fault(tick(fsm, image), condition == 7 ? SafetyInterlock : StrokeInterlock, "every closing interlock is checked");
		}
		image = new PressImage(); fsm = fresh();
		image.requestStroke = true; image.requestClampOnly = true;
		fault(tick(fsm, image), ConflictingRequests, "simultaneous requests do not pick a winner");
		image = new PressImage(); fsm = fresh(); image.requestOpen = true;
		fault(tick(fsm, image), OpenInterlock, "open is refused without a held clamp or completed bend");
		for (condition in 0...5) {
			image = new PressImage(); fsm = fresh(); image.requestStroke = true;
			output = tick(fsm, image);
			switch condition {
				case 0: image.fingerLeft = false;
				case 1: image.holding = false;
				case 2: image.axesInPosition = false;
				case 3: image.configuredStep = 1;
				case 4: image.safetyReady = false;
			}
			var expected:PressBrakeAlarm = switch condition {
				case 0: RegistrationLost; case 1: RobotHoldLost; case 2: PositioningLost;
				case 3: ProgramChanged; case _: SafetyInterlock;
			};
			fault(tick(fsm, image), expected, "approach observations retain their failure cause");
		}
		image = new PressImage(); fsm = fresh();
		output = bottom(fsm, image);
		image.requestOpen = true; image.clear = true;
		output = tick(fsm, image);
		check(output.phase == Decompressing && output.command == Decompress, "open first requests decompression");
		image.decompressed = true;
		output = tick(fsm, image);
		check(output.phase == Opening && output.command == Open && output.retractBackgauge, "observed decompression permits opening");
		image.atBottom = false; image.atTop = true;
		output = tick(fsm, image);
		check(output.phase == Idle && output.command == Hold, "top feedback ends the stroke");
		image.axesInPosition = true;
		output = tick(fsm, image);
		check(output.ready && output.phase == Idle, "maintained request does not repeat a stroke");
		image.requestStroke = false; image.requestOpen = false; tick(fsm, image);
		image.requestStroke = true; image.fingerLeft = true; image.fingerRight = true; image.holding = true;
		check(tick(fsm, image).command == CloseToClamp, "fresh request can start the next stroke");

		image = new PressImage(); fsm = fresh(); output = clamp(fsm, image, true);
		for (_ in 0...10) output = tick(fsm, image);
		check(output.phase == Clamped && output.alarm == None, "acknowledged clamp-only dwell does not expire");
		image.requestOpen = true; output = tick(fsm, image);
		check(output.command == Decompress, "held clamp accepts an open request");
		image = new PressImage(); fsm = fresh(); output = clamp(fsm, image);
		image.requestStroke = false; tick(fsm, image); image.requestStroke = true;
		fault(tick(fsm, image), Busy, "a fresh stroke during a stroke is refused");

		image = new PressImage(); fsm = fresh(); image.requestStroke = true;
		timeout(fsm, image, ClampTimeout);
		image = new PressImage(); fsm = fresh(); output = clamp(fsm, image);
		timeout(fsm, image, WithdrawalTimeout);
		image = new PressImage(); fsm = fresh(); output = clamp(fsm, image);
		image.backgaugeWithdrawn = true; output = tick(fsm, image); image.atClamp = false;
		timeout(fsm, image, DepthTimeout);
		image = new PressImage(); fsm = fresh(); output = bottom(fsm, image);
		image.requestOpen = true; image.clear = true;
		timeout(fsm, image, DecompressionTimeout);
		image = new PressImage(); fsm = fresh(); output = bottom(fsm, image);
		image.requestOpen = true; image.clear = true; tick(fsm, image); image.decompressed = true;
		timeout(fsm, image, OpenTimeout);

		image = new PressImage(); fsm = fresh(); image.requestStroke = true; image.holding = false;
		output = tick(fsm, image); image.faultReset = true;
		fault(tick(fsm, image), StrokeInterlock, "reset with an asserted motion request is refused");
		image.requestStroke = false; image.clear = true; image.faultReset = false; tick(fsm, image);
		image.faultReset = true; output = tick(fsm, image);
		check(output.alarm == None && output.ready, "safe fresh reset clears the latched alarm");
		for (condition in 0...4) {
			image = new PressImage(); fsm = fresh();
			switch condition {
				case 0: image.atClamp = true;
				case 1: image.ramPositionMm = Math.NaN;
				case 2: image.programStep = -1;
				case 3: image.configuredStep = -1;
			}
			fault(tick(fsm, image), InvalidFeedback, "invalid feedback is diagnosed before motion");
		}
		image = new PressImage(); fsm = fresh(); output = clamp(fsm, image);
		image.backgaugeWithdrawn = true; tick(fsm, image); image.depthStopInPosition = false;
		fault(tick(fsm, image), PositioningLost, "depth stop remains positioned during the working stroke");
		image = new PressImage(); fsm = fresh(); output = bottom(fsm, image);
		image.requestOpen = true;
		fault(tick(fsm, image), OpenInterlock, "opening requires a held or clear robot");
		image = new PressImage(); fsm = fresh(); output = clamp(fsm, image);
		image.requestOpen = true;
		check(tick(fsm, image).command == Decompress, "open can cancel a full stroke at the held pinch point");
		image = new PressImage(); fsm = fresh(); output = bottom(fsm, image);
		image.requestOpen = true; image.clear = true; tick(fsm, image); image.decompressed = true;
		tick(fsm, image); image.decompressed = false; image.atBottom = false; image.atTop = true;
		check(tick(fsm, image).phase == Opening, "top alone does not acknowledge a still-pressurized return");
		for (point in 0...5) {
			image = new PressImage(); fsm = fresh();
			if (point == 0) { image.requestStroke = true; tick(fsm, image); }
			else if (point == 1) output = clamp(fsm, image, true);
			else if (point == 2) { output = clamp(fsm, image); image.backgaugeWithdrawn = true; tick(fsm, image); }
			else {
				output = bottom(fsm, image);
				image.requestOpen = true; image.clear = true; tick(fsm, image);
				if (point == 4) { image.decompressed = true; tick(fsm, image); }
			}
			image.safetyReady = false;
			fault(tick(fsm, image), SafetyInterlock, "safety loss holds every active hydraulic phase");
		}
		for (condition in 0...4) {
			image = new PressImage(); fsm = fresh(); image.requestOpen = true; tick(fsm, image);
			image.requestOpen = false; image.faultReset = true; image.clear = true;
			switch condition {
				case 0: image.atTop = false;
				case 1: image.decompressed = false;
				case 2: image.clear = false;
				case 3: image.safetyReady = false;
			}
			fault(tick(fsm, image), OpenInterlock, "reset does not bypass any recovery interlock");
		}
		for (seconds in [0.0, -1.0, Math.NaN, Math.POSITIVE_INFINITY]) {
			var rejected = false;
			try { var invalid = new PressBrakeInterface(seconds); } catch (_:Dynamic) { rejected = true; }
			check(rejected, "timeout configuration must be finite and positive");
			rejected = false;
			try { var invalid = fresh().update(new PressImage().sample(), seconds); } catch (_:Dynamic) { rejected = true; }
			check(rejected, "controller time must be finite and positive");
		}
		Sys.println('Press brake interface tests passed ($assertions assertions)');
	}
}

private class PressImage {
	public var requestStroke = false;
	public var requestClampOnly = false;
	public var requestOpen = false;
	public var programStep = 0;
	public var clear = false;
	public var holding = true;
	public var faultReset = false;
	public var safetyReady = true;
	public var fingerLeft = true;
	public var fingerRight = true;
	public var configuredStep = 0;
	public var axesInPosition = true;
	public var depthStopInPosition = true;
	public var backgaugeWithdrawn = false;
	public var atTop = true;
	public var atClamp = false;
	public var atBottom = false;
	public var decompressed = true;
	public var ramPositionMm = 0.0;
	public function new() {}
	public function sample():PressBrakeInput return new PressBrakeInput(requestStroke, requestClampOnly,
		requestOpen, programStep, clear, holding, faultReset, safetyReady, fingerLeft, fingerRight,
		configuredStep, axesInPosition, depthStopInPosition, backgaugeWithdrawn, atTop, atClamp, atBottom, decompressed, ramPositionMm);
}
