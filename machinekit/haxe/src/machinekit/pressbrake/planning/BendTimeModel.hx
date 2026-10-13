package machinekit.pressbrake.planning;

import cadkit.sheetmetal.SheetDiagnosis;
import machinekit.fluid.CylinderPhaseSpeeds;

/** Explicit nominal handling assumptions plus the authored hydraulic phase speeds. */
class BendTimeModel {
	public final dwellSec:Float;
	final phases:CylinderPhaseSpeeds;
	final handlingSec:Float;
	final flipSec:Float;
	final turnSec:Float;
	final regripSec:Float;
	final stationSec:Float;
	final transferSpeed:Float;
	final gaugeSpeed:Float;
	public function new(phases:CylinderPhaseSpeeds, handlingSec:Float, flipSec:Float, turnSec:Float,
			regripSec:Float, stationSec:Float, transferSpeed:Float, gaugeSpeed:Float, dwellSec:Float) {
		if (phases == null) throw new SheetDiagnosis("press.time-model","Bend timing needs authored hydraulic phase speeds");
		for (value in [handlingSec,flipSec,turnSec,regripSec,stationSec,dwellSec])
			if (!Math.isFinite(value) || value < 0) throw new SheetDiagnosis("press.time-model","Handling and dwell times must be finite nonnegative seconds");
		if (!Math.isFinite(transferSpeed) || !Math.isFinite(gaugeSpeed) || transferSpeed <= 0 || gaugeSpeed <= 0)
			throw new SheetDiagnosis("press.time-model","Handling and backgauge speeds must be finite positive mm/s");
		this.phases = phases; this.handlingSec = handlingSec; this.flipSec = flipSec; this.turnSec = turnSec;
		this.regripSec = regripSec; this.stationSec = stationSec; this.transferSpeed = transferSpeed;
		this.gaugeSpeed = gaugeSpeed; this.dwellSec = dwellSec;
	}
	public function cycleTime(travel:Float):Float {
		if (!Math.isFinite(travel) || travel < 0 || travel > phases.stroke)
			throw new SheetDiagnosis("press.time-model-travel","Bend travel exceeds the hydraulic timing profile");
		return phases.extensionTime(0,travel)+phases.returnTime(travel,0)+dwellSec;
	}
	/** Omit optional handling, station and registration moves to obtain an admissible action bound. */
	public function minimumActionTime(travel:Float,retract:Float):Float return cycleTime(travel)+handlingSec+retract/gaugeSpeed;
	/** Visiting both stock faces requires a flip, regardless of the chosen remaining order. */
	public function minimumFlipTime(faceAUp:Bool,needsA:Bool,needsB:Bool):Float
		return (needsA && needsB) || (faceAUp ? needsB : needsA) ? flipSec : 0;
	public function measure(previous:Null<BendProgramStep>, setup:BendSetupResult, gauge:BackgaugeStroke, flip:Bool, regrip:Bool):BendTimeBreakdown {
		var turn = previous != null && previous.setup.orientation.reverse != setup.orientation.reverse;
		var previousCenter = previous == null ? 0.0 : previous.setup.orientation.bedCenter;
		var station = Math.abs(setup.orientation.bedCenter-previousCenter)/transferSpeed;
		if (previous != null && previous.setup.station != setup.station) station += stationSec;
		var x = previous == null ? 0.0 : previous.gauge.strokeX, r = previous == null ? 0.0 : previous.gauge.r;
		var backgauge = (Math.abs(gauge.registeredX-x)+Math.abs(gauge.r-r)+gauge.strokeX-gauge.registeredX)/gaugeSpeed;
		var bend = cycleTime(setup.ramTravel);
		return new BendTimeBreakdown(handlingSec+(turn ? turnSec : 0),flip ? flipSec : 0,regrip ? regripSec : 0,station,backgauge,bend);
	}
	public static function sameGrip(a:SheetGrip,b:SheetGrip):Bool
		return a.wall == b.wall && a.center.x == b.center.x && a.center.y == b.center.y && a.faceA == b.faceA;
}
