package machinekit.pressbrake.planning;

import cadkit.sheetmetal.Sheet;
import cadkit.sheetmetal.SheetDiagnosis;
import cadkit.sheetmetal.process.BendProcess;
import machinekit.pressbrake.planning.GripCandidateResult.GripAccepted;
import machinekit.pressbrake.planning.GripCandidateResult.GripRejected;

/** Deterministic best-first search over checked finite setups and authored wall grips. */
class BendSequencer {
	final press:PressGeometry;
	final process:BendProcess;
	final timing:BendTimeModel;
	final gripCheck:GripCheck;
	final collision:PressStrokeCollision;
	final setup:BendSetup;
	var failures:Array<BendSequenceFailure>;
	var deepest:Int;
	var attempts:Int;
	var limited:Bool;
	public function new(press:PressGeometry,process:BendProcess,timing:BendTimeModel,gripCheck:GripCheck,prisms:SheetCollisionQuery) {
		if (press == null || process == null || timing == null || gripCheck == null || prisms == null)
			throw new SheetDiagnosis("press.sequence-input","Sequence search needs press, process, timing, grip and collision checks");
		this.press = press; this.process = process; this.timing = timing; this.gripCheck = gripCheck;
		collision = new PressStrokeCollision(press,prisms); setup = new BendSetup(press);
		failures = []; deepest = -1; attempts = 0; limited = false;
	}
	public function plan(sheet:Sheet,grips:Array<SheetGrip>,retract:Float,?lot:String,maxNodes:Int = 10000,maxSetups:Int = 100000):BendSearchResult {
		if (grips == null || !Math.isFinite(retract) || retract < 0 || maxNodes < 1 || maxSetups < 1)
			throw new SheetDiagnosis("press.sequence-input","Sequence search needs finite withdrawal, grips and positive budgets");
		var validated = sheet.evaluate(sheet.flat());
		failures = []; deepest = -1; attempts = 0; limited = false;
		var count = sheet.bends().length, serial = 0, expanded = 0;
		var minimumTimes = minimumActions(sheet,retract,lot);
		var queue = new BendSearchQueue(); queue.push(new BendSearchNode([],[],0,remainingTime(sheet,[],true,minimumTimes),serial));
		var best:Map<String,Float> = new Map(), cache:Map<String,Array<BendSearchAction>> = new Map();
		while (!queue.empty()) {
			var node = queue.take(); if (node == null) throw new SheetDiagnosis("press.sequence-queue","Search queue lost its minimum state");
			var known = best.get(node.key()); if (known != null && known < node.seconds) continue;
			if (node.completed.length == count) return Planned(new BendProgram(node.steps));
			if (expanded >= maxNodes) return SearchLimit(failures.copy(),
				new SheetDiagnosis("press.sequence-node-budget","Search reached " + expanded + " expanded states",["limit:"+maxNodes]));
			expanded++;
			var completedKey = node.completed.join(","), actions = cache.get(completedKey);
			if (actions == null) {
				actions = actionsFor(sheet,node.completed,grips,retract,lot,maxSetups);
				cache.set(completedKey,actions);
			}
			if (limited) return SearchLimit(failures.copy(),
				new SheetDiagnosis("press.sequence-setup-budget","Search reached " + attempts + " setup candidates",["limit:"+maxSetups]));
			for (action in actions) {
				var completed = node.completed.copy(); completed.push(action.setup.bend); completed.sort(compareInt);
				var nextCompletedKey = completed.join(",");
				var remaining = remainingTime(sheet,completed,action.setup.orientation.faceAUp,minimumTimes);
				var preparation = new BendPreparation(node.last(),action.setup,action.gauge,timing);
				for (gripIndex in 0...action.grips.length) {
					var grip = action.grips[gripIndex];
					var seconds = node.seconds+preparation.time(grip).total; serial++;
					var key = BendSearchNode.keyFor(nextCompletedKey,action.id,gripIndex), previous = best.get(key);
					if (previous == null || seconds < previous) {
						var step = new BendProgramStep(preparation,grip);
						var steps = node.steps.copy(); steps.push(step);
						var next = new BendSearchNode(steps,completed,seconds,seconds+remaining,serial,action.id,gripIndex);
						best.set(key,seconds); queue.push(next);
					}
				}
			}
		}
		return NoSequence(failures.copy());
	}
	function actionsFor(sheet:Sheet,completed:Array<Int>,grips:Array<SheetGrip>,retract:Float,lot:Null<String>,maxSetups:Int):Array<BendSearchAction> {
		var state = sheet.flat(), result:Array<BendSearchAction> = [], gauges = BendCandidates.gauges(sheet);
		var seen:Array<BendSetupResult> = [];
		for (bend in sheet.bends()) if (completed.indexOf(bend.id) >= 0) { var applied = state.set(bend.id,bend.angle); }
		for (bend in sheet.bends()) if (completed.indexOf(bend.id) < 0)
			for (station in press.installedTools.stations()) for (reverse in [false,true]) {
				var centers = BendCandidates.centers(sheet,bend,station,press,bend.angle > 0,reverse);
				if (centers.length == 0) record(completed,bend.id,station.id,
					new SheetDiagnosis("press.tool-support","No bed-center candidate supports the authored material-bearing seam",["bend:"+bend.id,station.id]));
				for (center in centers) for (edge in gauges) {
					if (attempts >= maxSetups) { limited = true; return result; }
					attempts++;
					var orientation = new BendOrientation(bend.angle > 0,reverse,center);
					try {
						var planned = setup.plan(sheet,state,bend.id,station,orientation,edge,process,lot);
						var duplicate = false;
						for (previous in seen) if (sameSetup(previous,planned)) { duplicate = true; break; }
						if (duplicate) continue;
						seen.push(planned);
						var gauge = new BackgaugeStroke(planned.registration,retract);
						checkStroke(sheet,planned,gauge);
						var accepted:Array<SheetGrip> = [];
						for (outcome in gripCheck.checkCandidates(sheet,planned,grips,gauge)) switch outcome {
							case GripAccepted(grip): accepted.push(grip);
							case GripRejected(grip,failure): record(completed,bend.id,station.id,failure,orientation,edge,grip);
						}
						if (accepted.length > 0) result.push(new BendSearchAction(attempts,planned,gauge,accepted));
						else record(completed,bend.id,station.id,new SheetDiagnosis("press.no-grip-candidate","No supplied wall-grip candidate clears this stroke",["bend:"+bend.id,station.id]),orientation,edge);
					} catch (failure:SheetDiagnosis) record(completed,bend.id,station.id,failure,orientation,edge);
				}
			}
		return result;
	}
	static function sameSetup(a:BendSetupResult,b:BendSetupResult):Bool {
		var pa = a.process, pb = b.process;
		return a.bend == b.bend && a.station == b.station && a.orientation.faceAUp == b.orientation.faceAUp
			&& a.orientation.reverse == b.orientation.reverse && a.orientation.bedCenter == b.orientation.bedCenter
			&& a.registration.x == b.registration.x && a.registration.r == b.registration.r && a.ramTravel == b.ramTravel
			&& pa.targetAngle == pb.targetAngle && pa.loadedAngle == pb.loadedAngle && pa.innerRadius == pb.innerRadius
			&& pa.loadedRadius == pb.loadedRadius && pa.bendAllowance == pb.bendAllowance && pa.overbend == pb.overbend
			&& pa.depth == pb.depth && pa.analyticDepth == pb.analyticDepth && pa.depthOffset == pb.depthOffset
			&& pa.force == pb.force && pa.minimumFlange == pb.minimumFlange && pa.calibrated == pb.calibrated
			&& pa.radiusBend == pb.radiusBend && pa.recommendedOpening == pb.recommendedOpening;
	}
	function minimumActions(sheet:Sheet,retract:Float,lot:Null<String>):Map<Int,Float> {
		var result:Map<Int,Float> = new Map();
		for (bend in sheet.bends()) {
			var minimum = Math.POSITIVE_INFINITY;
			for (station in press.installedTools.stations()) {
			try {
				var prediction = process.predict(sheet.material,station.processTooling(),bend.angle,bend.edgeA.span.length(),lot);
				var contact = press.station(station.id), travel = contact.openTipHeight-contact.dieTopHeight+prediction.depth;
				minimum = Math.min(minimum,timing.minimumActionTime(travel,retract));
			} catch (failure:SheetDiagnosis) {}
			}
			result.set(bend.id,Math.isFinite(minimum) ? minimum : 0);
		}
		return result;
	}
	function remainingTime(sheet:Sheet,completed:Array<Int>,faceAUp:Bool,minimumTimes:Map<Int,Float>):Float {
		var seconds = 0.0, needsA = false, needsB = false;
		for (bend in sheet.bends()) if (completed.indexOf(bend.id) < 0) {
			var minimum = minimumTimes.get(bend.id);
			if (minimum != null) seconds += minimum;
			if (bend.angle > 0) needsA = true; else needsB = true;
		}
		return seconds+timing.minimumFlipTime(faceAUp,needsA,needsB);
	}
	function checkStroke(sheet:Sheet,planned:BendSetupResult,gauge:BackgaugeStroke):Void {
		var unresolved:Null<SheetDiagnosis> = null;
		for (error in [0.01,0.0025,0.000625]) {
			try { collision.check(sheet,planned,gauge,error); return; }
			catch (failure:SheetDiagnosis) {
				if (failure.code != "press.collision-resolution") throw failure;
				unresolved = failure;
			}
		}
		if (unresolved != null) throw unresolved;
		throw new SheetDiagnosis("press.collision-resolution","Stroke refinement did not produce a material certificate");
	}
	function record(completed:Array<Int>,bend:Int,station:String,failure:SheetDiagnosis,
			?orientation:BendOrientation,?gaugeEdge:GaugeEdge,?grip:SheetGrip):Void {
		if (completed.length < deepest) return;
		if (completed.length > deepest) { deepest = completed.length; failures = []; }
		var candidate = new BendSequenceFailure(completed,bend,station,failure,orientation,gaugeEdge,grip);
		for (stored in failures) if (candidate.same(stored)) return;
		if (failures.length >= 64) {
			for (stored in failures) if (stored.sameReason(candidate)) return;
			// Retain a new reason by replacing a redundant reason, rather than a unique witness.
			for (i in 0...failures.length) for (j in 0...i)
				if (failures[i].sameReason(failures[j])) { failures[i] = candidate; return; }
			return;
		}
		failures.push(candidate);
	}
	static function compareInt(a:Int,b:Int):Int return a < b ? -1 : a > b ? 1 : 0;
}
