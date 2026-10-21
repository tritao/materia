import materia.casting.FixtureConstraints;
import materia.casting.FixtureConstraints.FixtureConstraint;
import materia.casting.FixtureConstraints.FixtureCoordinate;
import materia.casting.FixtureConstraints.FixtureConstraintRole;

class FixtureConstraintTests {
	static var assertions = 0;
	static function check(ok:Bool, message:String):Void {
		assertions++;
		if (!ok) throw message;
	}
	static function rejects(fn:Void -> Void, expected:String):Void {
		var error = "";
		try fn() catch (e:Dynamic) error = Std.string(e);
		check(error.indexOf(expected) >= 0, 'Expected "$expected", got "$error"');
	}
	static function contact(id:String, x:Float, y:Float, nx:Float, ny:Float, nz:Float):FixtureConstraint
		return {id: id, sourceId: id + "-source", role: Location, point: {x:x,y:y,z:0}, normal: {x:nx,y:ny,z:nz}};
	static function fixture():Array<FixtureConstraint> return [
		contact("pad-a",0,0,0,0,1), contact("pad-b",100,0,0,0,1), contact("pad-c",0,100,0,0,1),
		contact("pin-x",0,0,1,0,0), contact("pin-y",0,0,0,1,0), contact("slot-y",100,0,0,1,0)
	];
	static function analyze(rows:Array<FixtureConstraint>):materia.casting.FixtureConstraints.FixtureConstraintReport
		return FixtureConstraints.analyze(rows,{x:0,y:0,z:0},100,1e-9);
	public static function run():Void {
		var contacts=fixture(),report=analyze(contacts);
		check(report.rank==6 && report.freeMotions.length==0,"three pads, round pin and correctly directed slot constrain six DOFs");
		check(report.redundantLocations.length==0,"3-2-1 locating scheme has no redundant contacts");
		var before=haxe.Json.stringify(contacts);
		var reversed=contacts.copy();reversed.reverse();
		check(haxe.Json.stringify(analyze(reversed))==haxe.Json.stringify(report),"contact order does not change the report");
		check(haxe.Json.stringify(contacts)==before,"analysis does not mutate source contacts");
		var missingSlot=contacts.filter(c->c.id!="slot-y");
		var free=analyze(missingSlot);
		check(free.rank==5 && free.freeMotions.length==1,"absent slot releases one DOF");
		check(free.freeMotions[0].affectedCoordinates.length==1 && free.freeMotions[0].affectedCoordinates[0]==Rz,
			"actual pin pair leaves rotation about its centre free");
		var shifted=FixtureConstraints.analyze(missingSlot,{x:-20,y:-30,z:0},100,1e-9);
		var motion=shifted.freeMotions[0];
		check(Math.abs(motion.rotationRad.z-0.01)<1e-12,"free rotation uses radians, not scaled matrix coordinates");
		check(Math.abs(motion.translationMm.x-0.3)<1e-12 && Math.abs(motion.translationMm.y+0.2)<1e-12,
			"offset origin reports the coupled translation needed to keep the pin fixed");
		var wrong=fixture();wrong[5]=contact("slot-y",100,0,1,0,0);
		var wrongSlot=analyze(wrong);
		check(wrongSlot.rank==5 && wrongSlot.freeMotions[0].affectedCoordinates[0]==Rz,
			"a slot in the wrong direction cannot replace the missing yaw constraint");
		check(wrongSlot.redundantLocations.length==1,"wrong slot duplicates one actual pin constraint");
		check(wrongSlot.redundantLocations[0].constraintIds.join(",")=="pin-x,slot-y",
			"redundancy names only the pin and slot participating in the dependency");
		check(wrongSlot.redundantLocations[0].affectedCoordinates.length==1
			&& wrongSlot.redundantLocations[0].affectedCoordinates[0]==Tx,"wrong slot overconstrains translation X");
		var over=fixture();over.push(contact("pad-d",100,100,0,0,1));
		var redundant=analyze(over);
		check(redundant.rank==6 && redundant.redundantLocations.length==1,"four rigid pads overconstrain the primary datum");
		var dependency=redundant.redundantLocations[0];
		check(dependency.constraintIds.join(",")=="pad-a,pad-b,pad-c,pad-d","all and only four coplanar pads participate");
		var expected=[1.0,-1,-1,1];
		for(i in 0...expected.length)check(Math.abs(dependency.coefficients[i]-expected[i])<1e-12,"reported pad dependency coefficient");
		check(dependency.affectedCoordinates.length==3 && dependency.affectedCoordinates[0]==Tz
			&& dependency.affectedCoordinates[1]==Rx && dependency.affectedCoordinates[2]==Ry,"redundant primary support reports its three affected directions");
		var clamp=contact("clamp",50,50,0,0,-1);clamp.role=Preload;
		contacts.push(clamp);
		var clamped=analyze(contacts);
		check(clamped.rank==6 && clamped.redundantLocations.length==0,"clamping force is not a seventh locating datum");
		check(clamped.preloadIds.join(",")=="clamp","preload remains explicit for separate force checks");
		var empty=analyze([]);
		check(empty.rank==0 && empty.freeMotions.length==6,"an unsupported insert reports all six free motions");
		rejects(() -> analyze([contact("not-unit",0,0,2,0,0)]),"normal must be unit length");
		rejects(() -> analyze([contacts[0],contacts[0]]),"Duplicate fixture constraint");
		rejects(() -> FixtureConstraints.analyze(fixture(),{x:0,y:0,z:0},0,1e-9),"rank scale or tolerance");
		var invalid=contact("invalid",0,0,1,0,0);invalid.role=cast "unknown";
		rejects(() -> analyze([invalid]),"Unknown fixture constraint role");
		Sys.println('FixtureConstraintTests: $assertions assertions passed');
	}
}
