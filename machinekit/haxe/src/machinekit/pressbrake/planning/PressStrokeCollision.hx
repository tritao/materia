package machinekit.pressbrake.planning;

import cadkit.sheetmetal.Sheet;
import cadkit.sheetmetal.SheetDiagnosis;

/** Check actual material through every sampled stroke pose against all installed tooling and machine volumes. */
class PressStrokeCollision {
	final press:PressGeometry;
	final material:PressMaterialQuery;
	final contact:PressContactQuery;
	public function new(press:PressGeometry, prisms:SheetCollisionQuery) {
		if (press == null) throw new SheetDiagnosis("press.stroke-collision", "Stroke collision needs installed press geometry");
		this.press = press; material = new PressMaterialQuery(prisms); contact = new PressContactQuery();
	}
	public function query(sheet:Sheet, setup:BendSetupResult, ?gauge:BackgaugeStroke, maximumError:Float = 0.01):PressStrokeCollisionResult {
		var station = press.station(setup.station), stroke = setup.stroke();
		if (stroke.length < 2)
			throw new SheetDiagnosis("press.stroke-collision", "Stroke collision needs both stroke endpoints");
		var motion = gauge == null ? new BackgaugeStroke(setup.registration, 0) : gauge;
		if (motion.registeredX != setup.registration.x || motion.r != setup.registration.r)
			throw new SheetDiagnosis("press.gauge-retract", "Backgauge stroke must start at the setup registration");
		var unresolved:PressStrokeCollisionResult = Clear;
		for (sample in 0...stroke.length) {
			var pose = stroke[sample], angle = pose.evaluation.bend(setup.bend).angle;
			var travel = station.openTipHeight - station.dieTopHeight + pose.depth;
			var bodies = PressCollisionGeometry.sheet(sheet, pose, maximumError);
			var tools = PressCollisionGeometry.toolContacts(press, travel);
			var toolBodies:Array<PressCollisionBody> = [];
			var machine:Array<PressCollisionBody> = [];
			// The first pose holds flat stock at first punch contact while X withdraws. Later
			// bend samples use the withdrawn position; no finger or carrier is exempted.
			var gaugePositions = sample == 0 ? motion.withdrawal() : [motion.strokeX];
			for (x in gaugePositions)
				for (body in PressCollisionGeometry.machineMembers(press, travel, x, motion.r)) machine.push(body);
			for (body in bodies) {
				for (i in 0...tools.length) {
					var tool = tools[i], overlap = contact.overlap(sheet, pose, body, tool);
					if (overlap == null) {
						if (toolBodies.length == 0) toolBodies = PressCollisionGeometry.tools(press, travel, maximumError);
						if (toolBodies.length != tools.length)
							throw new SheetDiagnosis("press.stroke-collision", "Exact and approximate installed tool rows disagree");
						overlap = material.overlap(sheet, pose, body, toolBodies[i]);
					}
					if (overlap == true) return Collision(body.owner, tool.owner, sample, angle);
					if (overlap == null) switch unresolved {
						case Clear: unresolved = Unresolved(body.owner, tool.owner, sample, angle);
						case _:
					}
				}
				for (obstacle in machine) {
					var overlap = material.overlap(sheet, pose, body, obstacle);
					if (overlap == true) return Collision(body.owner, obstacle.owner, sample, angle);
					if (overlap == null) switch unresolved {
						case Clear: unresolved = Unresolved(body.owner, obstacle.owner, sample, angle);
						case _:
					}
				}
			}
		}
		return unresolved;
	}
	/** Named diagnoses carry authored identities and the sampled angle, never parsed prism labels. */
	public function check(sheet:Sheet, setup:BendSetupResult, ?gauge:BackgaugeStroke, maximumError:Float = 0.01):Void {
		switch query(sheet, setup, gauge, maximumError) {
			case Clear:
			case Collision(stock, obstacle, sample, angle):
				throw diagnosis("press.collision", " intersects ", stock, obstacle, sample, angle);
			case Unresolved(stock, obstacle, sample, angle):
				throw diagnosis("press.collision-resolution", " needs finer material geometry against ", stock, obstacle, sample, angle);
		}
	}
	static function diagnosis(code:String, reason:String, stock:PressCollisionOwner, obstacle:PressCollisionOwner,
			sample:Int, angle:Float):SheetDiagnosis {
		var a = ownerName(stock), b = ownerName(obstacle);
		return new SheetDiagnosis(code, a + reason + b + " at " + angle * 180 / Math.PI + " degrees (sample " + sample + ")",
			[a, b, "sample:" + sample]);
	}
	static function ownerName(owner:PressCollisionOwner):String return switch owner {
		case SheetWall(id): "wall:" + id;
		case SheetBend(id): "bend:" + id;
		case Punch(station, segment): "punch:" + station + ":segment:" + segment;
		case Die(station, segment): "die:" + station + ":segment:" + segment;
		case MachineMember(id): "machine:" + id;
	};
}
