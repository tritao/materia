package machinekit.pressbrake.planning;

import cadkit.sheetmetal.Point2;
import cadkit.sheetmetal.Sheet;
import cadkit.sheetmetal.SheetDiagnosis;

/** Geometric stand-in follows its authored wall through the stroke, without exempting contact bodies. */
class GeometricGripCheck implements GripCheck {
	public final envelope:GripperBox;
	final press:PressGeometry;
	final prisms:SheetCollisionQuery;
	final material:PressMaterialQuery;
	final contact:PressContactQuery;
	public function new(press:PressGeometry, envelope:GripperBox, prisms:SheetCollisionQuery) {
		if (press == null || envelope == null || prisms == null)
			throw new SheetDiagnosis("press.grip-check", "Grip checks need press geometry, an explicit envelope and a collision query");
		this.press = press; this.envelope = envelope; this.prisms = prisms;
		material = new PressMaterialQuery(prisms); contact = new PressContactQuery();
	}
	/** Deterministic finite candidates are filtered by full patch support, including interior holes. */
	public function candidates(sheet:Sheet):Array<SheetGrip> {
		var result:Array<SheetGrip> = [];
		for (wall in sheet.walls()) {
			var lowX = Math.POSITIVE_INFINITY, lowY = Math.POSITIVE_INFINITY;
			var highX = Math.NEGATIVE_INFINITY, highY = Math.NEGATIVE_INFINITY;
			for (loop in wall.outline.loops()) for (segment in loop) for (point in segment.extrema()) {
				lowX = Math.min(lowX,point.x); highX = Math.max(highX,point.x);
				lowY = Math.min(lowY,point.y); highY = Math.max(highY,point.y);
			}
			lowX += envelope.width/2; highX -= envelope.width/2;
			lowY += envelope.length/2; highY -= envelope.length/2;
			if (highX < lowX || highY < lowY) continue;
			var xs = coordinates(lowX,highX), ys = coordinates(lowY,highY);
			for (x in xs) for (y in ys) for (face in [true,false]) {
				var grip = new SheetGrip(wall.id,new Point2(x,y),face);
				if (envelope.supported(sheet,grip)) result.push(grip);
			}
		}
		return result;
	}
	static function coordinates(low:Float, high:Float):Array<Float>
		return high-low <= 1e-8 ? [low] : [low,low+(high-low)/4,(low+high)/2,high-(high-low)/4,high];
	public function check(sheet:Sheet, setup:BendSetupResult, grip:SheetGrip, gauge:BackgaugeStroke):Void {
		var result = checkCandidates(sheet,setup,[grip],gauge)[0];
		switch result {
			case GripAccepted(_):
			case GripRejected(_,diagnosis): throw diagnosis;
		}
	}
	/** Share sampled press and stock geometry across all grips tested for the same setup. */
	public function checkCandidates(sheet:Sheet, setup:BendSetupResult, grips:Array<SheetGrip>, gauge:BackgaugeStroke):Array<GripCandidateResult> {
		var result:Array<GripCandidateResult> = [], supported:Array<Bool> = [], hasSupported = false;
		for (grip in grips) {
			var fits = envelope.supported(sheet,grip);
			supported.push(fits);
			if (fits) hasSupported = true;
			if (fits) result.push(GripAccepted(grip));
			else result.push(GripRejected(grip,new SheetDiagnosis("press.grip-support", "The entire gripper patch must lie on wall material", ["wall:" + grip.wall])));
		}
		if (!hasSupported) return result;
		gauge.validate(setup.registration);
		var station = press.station(setup.station), stroke = setup.stroke();
		if (stroke.length < 2) throw new SheetDiagnosis("press.grip-check", "Grip checks need both stroke endpoints");
		var toolsBySample:Array<Array<PressToolContact>> = [], machineBySample:Array<Array<PressCollisionBody>> = [];
		var materialBySample:Array<Array<Array<PressCollisionBody>>> = [];
		for (sample in 0...stroke.length) {
			var pose = stroke[sample], travel = station.openTipHeight-station.dieTopHeight+pose.depth;
			toolsBySample.push(PressCollisionGeometry.toolContacts(press,travel));
			var machine:Array<PressCollisionBody> = [];
			var positions = sample == 0 ? gauge.withdrawal() : [gauge.strokeX];
			for (x in positions) for (body in PressCollisionGeometry.machineMembers(press,travel,x,gauge.r)) machine.push(body);
			machineBySample.push(machine);
			materialBySample.push([]);
		}
		var errors = [0.01,0.0025,0.000625], checked:Array<GripCandidateResult> = [];
		for (gripIndex in 0...grips.length) {
			var grip = grips[gripIndex];
			if (!supported[gripIndex]) continue;
			var rejected:Null<SheetDiagnosis> = null;
			for (sample in 0...stroke.length) {
				var pose = stroke[sample], box = envelope.at(sheet,pose,grip);
				var obstacle = new PressCollisionBody(MachineMember("gripper"),[box],0);
				for (tool in toolsBySample[sample]) if (contact.exactPrisms([box],tool)) {
					rejected = failure("press.grip-collision",tool.owner,sample,grip); break;
				}
				if (rejected != null) break;
				for (body in machineBySample[sample]) if (rejected == null) {
					for (piece in body.pieces()) if (prisms.overlap(box,piece)) {
						rejected = failure("press.grip-collision",body.owner,sample,grip); break;
					}
				}
				if (rejected != null) break;
				var unresolved:Null<PressCollisionBody> = null;
				for (level in 0...errors.length) {
					unresolved = null;
					while (materialBySample[sample].length <= level)
						materialBySample[sample].push(PressCollisionGeometry.sheet(sheet,pose,errors[materialBySample[sample].length]));
					for (body in materialBySample[sample][level]) {
						var overlap = material.overlap(sheet,pose,body,obstacle);
						if (overlap == true) {
							rejected = failure("press.grip-collision",body.owner,sample,grip); break;
						}
						if (overlap == null && unresolved == null) unresolved = body;
					}
					if (rejected != null) break;
					if (unresolved == null) break;
				}
				if (rejected != null) break;
				if (unresolved != null) {
					rejected = failure("press.grip-resolution",unresolved.owner,sample,grip); break;
				}
			}
			if (rejected == null) checked.push(GripAccepted(grip)); else checked.push(GripRejected(grip,rejected));
		}
		var checkedIndex = 0;
		for (gripIndex in 0...grips.length) if (supported[gripIndex]) result[gripIndex] = checked[checkedIndex++];
		return result;
	}
	static function failure(code:String, owner:PressCollisionOwner, sample:Int, grip:SheetGrip):SheetDiagnosis {
		var obstacle = PressStrokeCollision.ownerName(owner);
		return new SheetDiagnosis(code,"Gripper on wall " + grip.wall + " fails against " + obstacle + " at sample " + sample,
			["wall:" + grip.wall,obstacle,"sample:" + sample]);
	}
}
