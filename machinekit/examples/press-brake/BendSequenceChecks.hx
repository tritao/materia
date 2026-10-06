import cadkit.sheetmetal.BendRule;
import cadkit.sheetmetal.SheetDemoParts;
import cadkit.sheetmetal.SheetMaterial;
import cadkit.sheetmetal.SheetDiagnosis;
import cadkit.sheetmetal.Sheet;
import machinekit.fluid.CylinderPhaseSpeeds;
import cadkit.sheetmetal.process.AirBending;
import cadkit.sheetmetal.process.ProcessBendRule;
import machinekit.pressbrake.planning.BendProgram;
import machinekit.pressbrake.planning.BendProgramStep;
import machinekit.pressbrake.planning.BendPreparation;
import machinekit.pressbrake.planning.SheetGrip;
import machinekit.pressbrake.planning.BendSequencer;
import machinekit.pressbrake.planning.BendTimeModel;
import machinekit.pressbrake.planning.GeometricGripCheck;
import machinekit.pressbrake.planning.GripperBox;
import machinekit.pressbrake.planning.PressGeometry;
import machinekit.pressbrake.planning.PrismSatQuery;
import machinekit.pressbrake.ToolLayout;
import machinekit.pressbrake.ToolRow;
import machinekit.pressbrake.ToolSegment;
import machinekit.pressbrake.ToolStation;
import machinekit.pressbrake.ToolingCatalogue;
import machinekit.pressbrake.PunchKind;

/** Check sequence programs against process-compatible acceptance parts. */
class BendSequenceChecks {
	public static function main():Void run();
	public static function run():Void {
		checkTimeBounds();
		var press = new PressBrake(true,null,null,30), geometry = PressGeometry.of(press), prisms = new PrismSatQuery();
		var tooling = press.toolLayout.stations()[0].processTooling();
		var stock = new SheetMaterial("sequence-steel",1.5,BendRule.Process(new ProcessBendRule(tooling)),{uts:400,yield:250,youngs:205000});
		var grips = new GeometricGripCheck(geometry,new GripperBox(10,10,15),prisms);
		var timing = new BendTimeModel(press.phaseSpeeds,1,3,1,4,2,200,100,0.3);
		var planner = new BendSequencer(geometry,new AirBending(),timing,grips,prisms);
		var parts = [SheetDemoParts.p1(stock),SheetDemoParts.p2(stock),SheetDemoParts.p3(stock),SheetDemoParts.p4(stock),SheetDemoParts.p5(stock)];
		for (part in 0...parts.length) {
			var sheet = parts[part], label = "P"+(part+1);
			var activePress = press;
			if (part >= 2) {
				var raised = ToolingCatalogue.raisedV12();
				var punch = part == 3 ? ToolingCatalogue.gooseneck() : ToolingCatalogue.straight();
				var punchRow = part == 4 ? new ToolRow([new ToolSegment(Punch(punch),20),
					new ToolSegment(Punch(punch),50),new ToolSegment(Punch(punch),40),
					new ToolSegment(Punch(punch),50),new ToolSegment(Punch(punch),20)],[20,0,0,20])
					: new ToolRow([new ToolSegment(Punch(punch),100),new ToolSegment(Punch(punch),50)],[0]);
				var dieRow = part == 4 ? new ToolRow([new ToolSegment(SingleV(raised),20),new ToolSegment(SingleV(raised),100),new ToolSegment(SingleV(raised),40),new ToolSegment(SingleV(raised),20)],[20,0,20])
					: new ToolRow([new ToolSegment(SingleV(raised),200)],[]);
				var layout = new ToolLayout(1600,[new ToolStation(part == 3 ? "gooseneck-V12" : part == 4 ? "gapped-V12" : "raised-V12",punchRow,dieRow,part == 4 ? 690 : 700)]);
				var elevated = new PressBrake(true,layout,null,30), derived = PressGeometry.of(elevated);
				activePress = elevated;
				grips = new GeometricGripCheck(derived,new GripperBox(10,10,15),prisms);
				timing = new BendTimeModel(elevated.phaseSpeeds,1,3,1,4,2,200,100,0.3);
				planner = new BendSequencer(derived,new AirBending(),timing,grips,prisms);
			}
			trace("Planning " + label + " with " + grips.candidates(sheet).length + " grip candidates");
			Sys.stdout().flush();
			switch planner.plan(sheet,grips.candidates(sheet),20,null,100000) {
				case Planned(program):
					checkProgram(sheet,program);
					checkPreparation(program.steps()[0],timing);
					checkTooling(activePress,program,part);
					if (part < 2) checkLimitsAndRepeat(sheet,planner,grips,program,part == 0);
					if (part == 2) {
						var flipped = false;
						for (step in program.steps()) if (step.flip) flipped = true;
						if (!flipped) throw "P3 must retain its required stock-face flip";
					}
					trace(label + " planned: " + program.expectedTime + " seconds");
					for (step in program.steps()) trace("bend " + step.setup.bend + " station " + step.setup.station + " center " + step.setup.orientation.bedCenter + " reverse " + step.setup.orientation.reverse + " grip " + step.grip.wall + " at " + step.grip.center.x + "," + step.grip.center.y + " face " + step.grip.faceA);
				case NoSequence(failures):
					if (failures.length > 0) trace("Deepest failed state: " + failures[0].completed().join(","));
					for (failure in failures) trace(failure.diagnosis.toString());
					throw label + " sequence missing";
				case SearchLimit(failures,reason):
					trace(reason.toString());
					if (failures.length > 0) trace("Deepest failed state: " + failures[0].completed().join(","));
					for (failure in failures) trace(failure.diagnosis.toString());
					throw label + " search exhausted its budget";
			}
		}
	}
	static function checkProgram(sheet:Sheet,program:BendProgram):Void {
		var steps = program.steps(), ids:Array<Int> = [], seconds = 0.0;
		if (steps.length != sheet.bends().length) throw "Every authored bend must be planned";
		for (step in steps) {
			if (ids.indexOf(step.setup.bend) >= 0) throw "A program must bend each seam exactly once";
			ids.push(step.setup.bend); seconds += step.time.total;
			step.gauge.validate(step.setup.registration);
			near(step.gauge.strokeX-step.gauge.registeredX,20,"Program retains the checked withdrawal");
			if (step.time.total <= 0) throw "Every physical bend takes positive nominal time";
			if (step.flip && step.time.flip != 3) throw "Flip marker and time disagree";
			if (step.regrip && step.time.regrip != 4) throw "Regrip marker and time disagree";
		}
		near(program.expectedTime,seconds,"Program time sums its checked actions");
		var state = program.finalState();
		for (bend in sheet.bends()) near(state.angle(bend.id),bend.angle,"Final program reaches the authored released angle");
		var evaluated = sheet.evaluate(state);
		var removed = steps.pop();
		if (program.steps().length != sheet.bends().length) throw "Returned steps must not mutate the program";
		if (sheet.bends().length > 0) {
			var first = sheet.bends()[0], modified = state.set(first.id,0);
			near(program.finalState().angle(first.id),first.angle,"Final state must be derived afresh");
		}
	}
	static function checkPreparation(previous:BendProgramStep,timing:BendTimeModel):Void {
		var preparation = new BendPreparation(previous,previous.setup,previous.gauge,timing);
		var held = new BendProgramStep(preparation,previous.grip);
		var other = new SheetGrip(previous.grip.wall,previous.grip.center,!previous.grip.faceA);
		var changed = new BendProgramStep(preparation,other);
		if (held.regrip || !changed.regrip || held.flip || changed.flip)
			throw "Holding the same grip or changing its stock face must retain the correct preparation markers";
		near(changed.time.total-held.time.total,4,"Changing the grip adds only the authored regrip time");
		near(changed.time.handling,held.time.handling,"Grip choice retains handling time");
		near(changed.time.station,held.time.station,"Grip choice retains station time");
		near(changed.time.backgauge,held.time.backgauge,"Grip choice retains backgauge time");
		near(changed.time.bend,held.time.bend,"Grip choice retains hydraulic cycle time");
	}
	static function checkTooling(press:PressBrake,program:BendProgram,part:Int):Void {
		var steps = program.steps();
		if (part == 3) {
			var last = steps[steps.length-1], station = installed(press,last.setup.station);
			switch station.punches.profile {
				case Punch(profile): if (profile.kind != Gooseneck) throw "P4's last bend must use the actual gooseneck section";
				case _: throw "P4's last bend must retain an installed punch";
			}
		}
		if (part == 4) for (step in steps) {
			var station = installed(press,step.setup.station);
			if (station.punches.span <= station.punches.length || station.dies.span <= station.dies.length)
				throw "P5's selected station must retain physical gaps in both installed rows";
			for (segment in station.punches.segments()) if (segment.horn != null)
				throw "P5's supported gapped-row sequence must not depend on an end-relief horn";
		}
	}
	static function installed(press:PressBrake,id:String):ToolStation {
		for (station in press.toolLayout.stations()) if (station.id == id) return station;
		throw "Program station must belong to its authored machine";
	}
	static function checkLimitsAndRepeat(sheet:Sheet,planner:BendSequencer,grips:GeometricGripCheck,program:BendProgram,single:Bool):Void {
		switch planner.plan(sheet,grips.candidates(sheet),20,null,10000,1) {
			case SearchLimit(_,reason):
				if (reason.code != "press.sequence-setup-budget") throw "Setup budget must retain its own diagnosis";
			case _: throw "Setup budget exhaustion must not claim physical infeasibility";
		}
		switch planner.plan(sheet,grips.candidates(sheet),20,null,1) {
			case Planned(repeated):
				if (!single) throw "Two bends require more than one expanded state";
				checkRepeat(program,repeated);
			case SearchLimit(_,reason):
				if (single || reason.code != "press.sequence-node-budget") throw "Node budget diagnosis does not match the search";
			case _: throw "Node budget exhaustion must not claim physical infeasibility";
		}
		if (!single) switch planner.plan(sheet,grips.candidates(sheet),20) {
			case Planned(repeated): checkRepeat(program,repeated);
			case _: throw "Repeating a feasible search must find the same program";
		}
	}
	static function checkRepeat(first:BendProgram,second:BendProgram):Void {
		near(second.expectedTime,first.expectedTime,"Repeat search retains nominal time");
		var a = first.steps(), b = second.steps();
		if (a.length != b.length) throw "Repeat search retains bend count";
		for (i in 0...a.length) {
			var left = a[i], right = b[i];
			if (left.setup.bend != right.setup.bend || left.setup.station != right.setup.station
				|| left.setup.orientation.faceAUp != right.setup.orientation.faceAUp
				|| left.setup.orientation.reverse != right.setup.orientation.reverse
				|| left.setup.orientation.bedCenter != right.setup.orientation.bedCenter
				|| left.setup.gaugeEdge.wall != right.setup.gaugeEdge.wall
				|| left.setup.gaugeEdge.edge.loop != right.setup.gaugeEdge.edge.loop
				|| left.setup.gaugeEdge.edge.segment != right.setup.gaugeEdge.edge.segment
				|| !BendTimeModel.sameGrip(left.grip,right.grip)
				|| left.flip != right.flip || left.regrip != right.regrip
				|| left.gauge.registeredX != right.gauge.registeredX || left.gauge.r != right.gauge.r
				|| left.gauge.strokeX != right.gauge.strokeX) throw "Repeat search must retain the same typed actions";
		}
	}
	static function checkTimeBounds():Void {
		var timing = new BendTimeModel(new CylinderPhaseSpeeds(100,20,5,10,40),1,3,1,4,2,200,100,0.3);
		near(timing.cycleTime(60),12.3,"Hydraulic timing integrates approach, working, return and dwell");
		near(timing.cycleTime(20),3.3,"Short stroke never enters the working phase");
		near(timing.minimumActionTime(60,20),13.5,"Action bound includes handling and checked withdrawal");
		near(timing.minimumFlipTime(true,true,true),3,"Mixed remaining faces require a flip");
		near(timing.minimumFlipTime(true,false,true),3,"Opposite remaining face requires a flip");
		near(timing.minimumFlipTime(false,true,false),3,"Opposite underside requires a flip");
		near(timing.minimumFlipTime(false,false,true),0,"Matching remaining face requires no flip");
		near(timing.minimumFlipTime(true,false,false),0,"Completed program requires no flip");
		var rejected = false;
		try { var invalid = timing.cycleTime(101); }
		catch (failure:SheetDiagnosis) { rejected = failure.code == "press.time-model-travel"; }
		if (!rejected) throw "Timing must refuse travel outside the hydraulic profile";
	}
	static function near(actual:Float,expected:Float,message:String):Void {
		if (!Math.isFinite(actual) || Math.abs(actual-expected) > 1e-8) throw message + ": " + actual + " instead of " + expected;
	}
}
