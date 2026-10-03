import cadkit.modeling.AssemblyState;
import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.structural.FrameAssembly;
import machinekit.structural.RectTube;
import machinekit.welding.WeldSeam;
import machinekit.welding.WeldSeams;
import machinekit.welding.Weldment;
import machinekit.welding.Weldment.WeldmentSeams;
import materia.assembly.AssemblyFrames;

/**
 * The weldment's seams come from its geometry: found where the members meet, named after the two faces,
 * framed on the faces' bisector, kept as the members are edited, and missing (reported) when they don't touch.
 */
class WeldSeamChecks {
	static final PLATE_SEAMS = ["work/basePlate:box.+z|work/upright:box.+y", "work/basePlate:box.+z|work/upright:box.-y"];

	static function near(actual:Float, expected:Float, message:String, tolerance:Float = 1e-6):Void {
		if (!(Math.abs(actual - expected) <= tolerance))
			throw '$message: expected $expected, got $actual';
	}

	static function nearVector(actual:Vector, x:Float, y:Float, z:Float, message:String):Void {
		near(actual.x, x, '$message x');
		near(actual.y, y, '$message y');
		near(actual.z, z, '$message z');
	}

	/** Finds the cell's seams (in the workpiece's frame) and checks them. */
	public static function run(cell:WeldingCell, state:AssemblyState):Array<WeldSeam> {
		var seams = cell.weldment().findIn(cell, cell.solvedPoses(state)).require();
		checkTJoint(seams);
		checkTubes(seams);
		checkEdits(cell, seams);
		checkGap();
		checkJoints();
		Sys.println('robot welder: ${seams.length} seams from the weldment, ' +
			[for (seam in seams) '${Math.round(seam.length())}'].join("/") + " mm");
		return seams;
	}

	static function between(seams:Array<WeldSeam>, a:String, b:String):Array<WeldSeam>
		return [for (seam in seams) if (seam.a.member == a && seam.b.member == b) seam];

	/** The plate and upright make two fillet seams along the upright's long sides, whose frames bisect the T at 45 degrees. */
	static function checkTJoint(seams:Array<WeldSeam>):Void {
		var tee = between(seams, "work/basePlate", "work/upright");
		var names = [for (seam in tee) seam.name()];
		names.sort(Reflect.compare);
		if (names.join(",") != PLATE_SEAMS.join(",")) throw 'The T-joint should have a seam on each side of the upright, got $names';
		for (seam in tee) {
			if (seam.joint != JointType.Fillet) throw 'Seam ${seam.name()} should be a fillet, not ${seam.joint}';
			near(seam.length(), WeldingWorkpiece.PLATE_LENGTH, "seam length matches the plate", 1e-6);
			near(seam.normalAngle(), Math.PI / 2, "the T-joint's faces are square");
			// The seam runs the length of the plate along the upright's face, on the plate's top.
			var side = seam.b.face == "box.-y" ? -1.0 : 1.0;
			var half = WeldingWorkpiece.PLATE_LENGTH / 2, y = side * WeldingWorkpiece.UPRIGHT_THICKNESS / 2;
			// The direction of travel follows the faces (plate normal x upright normal), so the two sides run opposite ways.
			var run = -side;
			nearVector(seam.start, -run * half, y, WeldingWorkpiece.PLATE_THICKNESS, "seam start");
			nearVector(seam.stop, run * half, y, WeldingWorkpiece.PLATE_THICKNESS, "seam stop");

			// Without push or work angle the torch axis is 45 degrees to both faces and square to the seam, on the open side.
			var square = seam.configured(null, null, 0, 0);
			for (fraction in [0.0, 0.5, 1.0]) {
				var frame = square.frameAtParameter(fraction);
				near(frame.torchAxis.dot(seam.normalA), Math.cos(Math.PI / 4), "torch axis to the plate", 1e-9);
				near(frame.torchAxis.dot(seam.normalB), Math.cos(Math.PI / 4), "torch axis to the upright", 1e-9);
				near(frame.torchAxis.dot(frame.tangent), 0, "torch axis square to the seam", 1e-9);
				near(frame.position.x, -run * half + run * fraction * 2 * half, "frame position along the seam", 1e-9);
			}
			// The default push angle swings the wire toward the direction of travel and nothing else.
			var pushed = seam.frameAtParameter(0.5);
			near(pushed.wire().dot(pushed.tangent), Math.sin(WeldSeam.PUSH_ANGLE), "push angle", 1e-9);
			near(pushed.torchAxis.dot(seam.normalA), pushed.torchAxis.dot(seam.normalB), "push keeps the work angle", 1e-9);
			// The robot's frame has +Z along the wire and +X along the travel.
			var tcp = pushed.toAssemblyFrame();
			var z = AssemblyFrames.transformVector(tcp, 0, 0, 1), x = AssemblyFrames.transformVector(tcp, 1, 0, 0);
			var wire = pushed.wire();
			near(z.x * wire.x + z.y * wire.y + z.z * wire.z, 1, "tcp z along the wire", 1e-9);
			if (!(x.x * pushed.tangent.x + x.y * pushed.tangent.y + x.z * pushed.tangent.z > 0.9)) throw "tcp x should follow the travel";
			// A work angle leans the axis toward the first face (the plate), so it makes a steeper angle to the plate's top.
			var leaned = seam.configured(null, null, 0, 0.2).frameAtParameter(0.5);
			if (!(leaned.torchAxis.dot(seam.normalA) > Math.cos(Math.PI / 4) + 0.05)) throw "work angle should lean toward the first face";
			near(seam.reversed().start.x, seam.stop.x, "reversed seam starts at the stop");
			if (seam.reversed().name() != seam.name()) throw "reversing a seam keeps its name";
		}
	}

	/** Each post is welded to the beam all round: one seam per side, and the four sides chain into a loop. */
	static function checkTubes(seams:Array<WeldSeam>):Void {
		for (post in ["work/postLeft", "work/postRight"]) {
			var sides = between(seams, "work/beam", post);
			if (sides.length != 4) throw 'Post $post should be welded all round with four seams, got ${sides.length}';
			var names:Map<String, Bool> = new Map();
			for (seam in sides) {
				names.set(seam.name(), true);
				if (seam.joint != JointType.Fillet) throw 'Seam ${seam.name()} should be a fillet';
				near(seam.length(), 40, "tube side length", 1e-6);
				near(seam.start.z, 40, "post seams lie on the beam's top", 1e-6);
				near(seam.bisector().z, Math.sqrt(0.5), "tube seam torch axis rises at 45 degrees", 1e-9);
			}
			var count = 0;
			for (_ in names) count++;
			if (count != 4) throw "The four sides of a post are four differently named seams";
			var chains = WeldSeams.chains(sides);
			if (chains.length != 1 || !chains[0].closed || chains[0].seams.length != 4) throw "The sides of a post should chain into one closed loop";
			for (i in 0...4) {
				var seam = chains[0].seams[i], next = chains[0].seams[(i + 1) % 4];
				near(seam.stop.subtract(next.start).length(), 0, "chained seams run on end to end", 1e-6);
			}
		}
		// The beam and the plate are separate members that do not touch.
		if (between(seams, "work/basePlate", "work/beam").length != 0) throw "The plate and the beam do not touch";
	}

	/** Editing a size renames nothing and moves the seam with its edge. */
	static function checkEdits(cell:WeldingCell, seams:Array<WeldSeam>):Void {
		function names(list:Array<WeldSeam>):String {
			var result = [for (seam in list) StringTools.replace(seam.name(), "work/", "")];
			result.sort(Reflect.compare);
			return result.join(",");
		}
		function standalone(work:WeldingWorkpiece):Array<WeldSeam>
			return work.weldment().findIn(work, work.solvedPoses()).require();
		var alone = standalone(new WeldingWorkpiece());
		if (names(alone) != names(seams)) throw "The workpiece's seams should be the same on their own as in the cell";
		// The seams are in the workpiece's frame, so where the cell puts the workpiece does not change them.
		for (seam in seams) {
			var same = [for (other in alone) if (other.name() == StringTools.replace(seam.name(), "work/", "")) other][0];
			nearVector(same.start, seam.start.x, seam.start.y, seam.start.z, "seam start is the same in the cell");
			nearVector(same.stop, seam.stop.x, seam.stop.y, seam.stop.z, "seam stop is the same in the cell");
		}
		// A shorter upright and wider tubes: every seam keeps its name and follows its edge.
		var shorter = WeldingWorkpiece.PLATE_LENGTH - 40;
		var edited = standalone(new WeldingWorkpiece(shorter, 50));
		for (seam in alone) {
			var after = [for (other in edited) if (other.name() == seam.name()) other];
			if (after.length != 1) throw 'Editing a size should keep the seam ${seam.name()}, got ${[for (other in edited) other.name()]}';
			if (seam.a.member == "basePlate") {
				near(after[0].length(), shorter, "the shorter upright's seam is shorter", 1e-6);
				near(Math.abs(after[0].start.x), shorter / 2, "the seam ends where the upright does", 1e-6);
			} else {
				near(after[0].length(), 50, "the wider tube's seam is longer", 1e-6);
				near(after[0].start.z, 50, "the wider tube's seams are higher on the beam", 1e-6);
			}
		}
		// The upright no longer ends flush with the plate, so its ends now have seams of their own.
		var ends = [for (seam in edited) if (seam.b.member == "upright" && (seam.b.face == "box.+x" || seam.b.face == "box.-x")) seam];
		if (ends.length != 2) throw 'A shorter upright should gain a seam at each end, got ${ends.length}';
		for (seam in ends) near(seam.length(), WeldingWorkpiece.UPRIGHT_THICKNESS, "end seam", 1e-6);
	}

	/** A gap between members gives no seam, and says so. */
	static function checkGap():Void {
		function weld(gap:Float):WeldmentSeams {
			var parts:Map<String, Part> = new Map();
			parts.set("plate", Part.box(100, 100, 10));
			parts.set("upright", Part.box(100, 8, 40).translated(new Vector(0, 0, 10 + gap)));
			var result = new Weldment("plate", ["plate", "upright"]).join("plate", "upright", 4).find(parts);
			for (part in parts) part.close();
			return result;
		}
		var touching = weld(0);
		if (touching.seams.length != 2 || touching.diagnostics.hasErrors()) throw "A plate and an upright that touch should have two seams";
		var apart = weld(0.5);
		if (apart.seams.length != 0 || !apart.diagnostics.hasErrors()) throw "Members with a gap should report no seam";
		var message = apart.diagnostics.items[0].message;
		if (apart.diagnostics.items[0].code != "weld.no-seam" || message.indexOf("0.5 mm apart") < 0)
			throw 'The missing seam should be reported with the gap, got "$message"';
		var threw = false;
		try apart.require() catch (error:Dynamic) threw = true;
		if (!threw) throw "require() should refuse a weldment with a missing seam";
	}

	/** Lap, butt and corner joints are told from fillets by their faces. */
	static function checkJoints():Void {
		function find(a:Part, b:Part):Array<WeldSeam> {
			var seams = WeldSeams.find({id: "a", part: a}, {id: "b", part: b});
			a.close();
			b.close();
			return seams;
		}
		var lap = find(Part.box(200, 100, 10), Part.box(100, 80, 8).translated(new Vector(0, 0, 10)));
		if (lap.length != 4) throw 'A plate lying on a plate should have a seam along each of its four edges, got ${lap.length}';
		for (seam in lap) if (seam.joint != JointType.Lap) throw 'Seam ${seam.name()} should be a lap, not ${seam.joint}';
		var butt = find(Part.box(100, 50, 10).translated(new Vector(-50, 0, 0)), Part.box(100, 50, 10).translated(new Vector(50, 0, 0)));
		if (butt.length != 4) throw 'Two plates butted edge to edge should have four seams, got ${butt.length}';
		for (seam in butt) {
			if (seam.joint != JointType.Butt) throw 'Seam ${seam.name()} should be a butt, not ${seam.joint}';
			near(seam.normalAngle(), 0, "a butt's faces are flat");
		}
		// Two tubes mitred into an elbow meet on a square corner at the inside and the outside, and on flat faces top and bottom.
		var frame = new FrameAssembly();
		frame.point("a", -100, 0, 0);
		frame.point("b", 0, 0, 0);
		frame.point("c", 0, 100, 0);
		var tube = new RectTube(40, 40, 3);
		frame.member("first", "a", "b", tube, null, Square, Mitre(40));
		frame.member("second", "b", "c", tube, null, Mitre(40), Square);
		var elbow = find(frame.geometry("first"), frame.geometry("second"));
		var corners = [for (seam in elbow) if (seam.joint == JointType.Corner) seam], flats = [for (seam in elbow) if (seam.joint == JointType.Butt) seam];
		if (corners.length != 2 || flats.length != 2) throw 'A mitred elbow should have two corner and two butt seams, got ${elbow.length}';
		for (seam in corners) {
			near(seam.length(), 40, "corner seam length", 1e-6);
			near(seam.normalAngle(), Math.PI / 2, "a corner's faces are square");
			near(Math.abs(seam.bisector().x), Math.sqrt(0.5), "the corner's torch axis is diagonal", 1e-9);
		}
		for (seam in flats) near(seam.length(), 40 * Math.sqrt(2), "the mitre is 45 degrees", 1e-6);
	}
}
