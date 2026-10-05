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
		if (!envelope.supported(sheet,grip))
			throw new SheetDiagnosis("press.grip-support", "The entire gripper patch must lie on wall material", ["wall:" + grip.wall]);
		gauge.validate(setup.registration);
		var station = press.station(setup.station), stroke = setup.stroke();
		if (stroke.length < 2) throw new SheetDiagnosis("press.grip-check", "Grip checks need both stroke endpoints");
		for (sample in 0...stroke.length) {
			var pose = stroke[sample], travel = station.openTipHeight-station.dieTopHeight+pose.depth;
			var box = envelope.at(sheet,pose,grip), obstacle = new PressCollisionBody(MachineMember("gripper"),[box],0);
			for (tool in PressCollisionGeometry.toolContacts(press,travel))
				if (contact.exactPrisms([box],tool)) fail("press.grip-collision",tool.owner,sample,grip);
			var positions = sample == 0 ? gauge.withdrawal() : [gauge.strokeX];
			for (x in positions) for (body in PressCollisionGeometry.machineMembers(press,travel,x,gauge.r))
				for (piece in body.pieces()) if (prisms.overlap(box,piece)) fail("press.grip-collision",body.owner,sample,grip);
			for (body in PressCollisionGeometry.sheet(sheet,pose)) {
				var overlap = material.overlap(sheet,pose,body,obstacle);
				if (overlap == true) fail("press.grip-collision",body.owner,sample,grip);
				if (overlap == null) fail("press.grip-resolution",body.owner,sample,grip);
			}
		}
	}
	static function fail(code:String, owner:PressCollisionOwner, sample:Int, grip:SheetGrip):Void {
		var obstacle = PressStrokeCollision.ownerName(owner);
		throw new SheetDiagnosis(code,"Gripper on wall " + grip.wall + " fails against " + obstacle + " at sample " + sample,
			["wall:" + grip.wall,obstacle,"sample:" + sample]);
	}
}
