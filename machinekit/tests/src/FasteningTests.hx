import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import cadkit.threads.ThreadSpec;
import materia.project.ThreadSpecData.ThreadHand;
import materia.project.ThreadSpecData.ThreadProfile;
import cadkit.threads.Threads;
import cadkit.threads.ThreadedFace.ThreadStart;
import CadKit;
import machinekit.assembly.FlangeBearingAssembly;
import machinekit.assembly.FlangeTappedPlate;
import machinekit.component.MachineComponent;
import machinekit.component.ComponentDetail;
import machinekit.component.CollisionHullFacet;
import machinekit.component.Solids;
import machinekit.standard.SocketHeadCapScrew;
import machinekit.standard.DeepGrooveBearing;
import machinekit.standard.HexNut;
import machinekit.standard.HexBolt;
import machinekit.screwdriving.FastenedAssembly;
import machinekit.screwdriving.Fastening.TighteningAngleWindow;
import machinekit.screwdriving.Fastenings;
import machinekit.screwdriving.Fastenings.FasteningsResult;
import machinekit.screwdriving.FlangeBearingMount;
import machinekit.screwdriving.NemaMotorMount;
import materia.assembly.AssemblyFrames;

class FasteningClamp extends MachineComponent {
	final diameter:Float;
	public function new(diameter:Float = 6.6) {
		super("TEST-CLAMP", "test clamp", "steel", true); this.diameter = diameter;
		addConnector("head", Face, Solids.axial(0, 0, 0));
	}
	override public function hasGeometry():Bool return true;
	override public function geometry(detail:ComponentDetail = Preview):Part {
		var block = Part.box(20, 20, 5);
		var base = block.translated(new Vector(0, 0, -5)); block.close();
		return Solids.cut(base, [Solids.named(Part.cylinderSpan(diameter / 2, -5.1, 0.1), "clearance")]);
	}
}

class FasteningTap extends MachineComponent {
	final spec:ThreadSpec;
	final shift:Float;
	public function new(spec:ThreadSpec, material:String = "steel", shift:Float = 0) {
		super("TEST-TAP", "test tap", material, true); this.spec = spec; this.shift = shift;
	}
	override public function hasGeometry():Bool return true;
	override public function geometry(detail:ComponentDetail = Preview):Part {
		var block = Part.box(20, 20, 20);
		var base = block.translated(new Vector(0, 0, -25)); block.close();
		var tool = Threads.tappedBore(spec, Math.min(5, spec.majorDiameter - 0.2), 15);
		var tap = tool.translated(new Vector(shift, 0, -5)); tool.close();
		return Solids.cut(base, [tap]);
	}
}

class FasteningObstacle extends MachineComponent {
	final bored:Bool;
	public function new(bored:Bool) { super("TEST-OBSTACLE", "test obstacle", "steel", true); this.bored = bored; }
	override public function hasGeometry():Bool return true;
	override public function geometry(detail:ComponentDetail = Preview):Part {
		var block = Part.box(20, 20, 2);
		var base = block.translated(new Vector(0, 0, -7)); block.close();
		return bored ? Solids.cut(base, [Solids.named(Part.cylinderSpan(3.3, -7.1, -4.9), "plain")]) : base;
	}
}

class FasteningConicalTap extends MachineComponent {
	final spec:ThreadSpec;
	public function new(spec:ThreadSpec) { super("TEST-CONE", "test cone", "steel", true); this.spec = spec; }
	override public function hasGeometry():Bool return true;
	override public function geometry(detail:ComponentDetail = Preview):Part {
		var block = Part.box(20, 20, 20);
		var base = block.translated(new Vector(0, 0, -25)); block.close();
		var tool = Part.revolve([{r: 0.0, z: -20.0}, {r: 2.0, z: -20.0}, {r: 2.5, z: -5.0}, {r: 0.0, z: -5.0}]);
		var names:Array<String> = [];
		for (index in 0...tool.shape.subshapeCount(CadKit.ShapeKind.Face)) {
			var face = tool.shape.subshape(CadKit.ShapeKind.Face, index);
			names.push(face.surfaceKind() == CadKit.SurfaceKind.Cone ? "bore-wall" : ""); face.close();
		}
		var named = tool.shape.withElementNames(CadKit.ShapeKind.Face, names); tool.close();
		var tap = new Part(Threads.attach(named, "bore-wall", spec, AxisMaximum, 15)); named.close();
		return Solids.cut(base, [tap]);
	}
}

/** Failures are checked against actual seated geometry, including rebuilt occurrences. */
class FasteningTests {
	static var assertions = 0;
	static function check(value:Bool, message:String):Void { assertions++; if (!value) throw message; }
	static function near(a:Float, b:Float):Void check(Math.abs(a - b) < 1e-4, 'fastening expected $b, got $a');
	static function convexHull(points:Array<Vector>):Array<Vector> {
		var sorted:Array<Vector> = [];
		for (point in points) {
			var duplicate = false;
			for (existing in sorted) if (Math.abs(existing.x-point.x) < 1e-8 && Math.abs(existing.y-point.y) < 1e-8) duplicate = true;
			if (!duplicate) sorted.push(point);
		}
		sorted.sort(function(a,b) {
			if (a.x < b.x) return -1;
			if (a.x > b.x) return 1;
			if (a.y < b.y) return -1;
			if (a.y > b.y) return 1;
			return 0;
		});
		var lower:Array<Vector> = [];
		for (point in sorted) {
			while (lower.length >= 2 && cross(lower[lower.length-2],lower[lower.length-1],point) <= 1e-10) lower.pop();
			lower.push(point);
		}
		var upper:Array<Vector> = [], cursor = sorted.length - 1;
		while (cursor >= 0) {
			var point = sorted[cursor];
			while (upper.length >= 2 && cross(upper[upper.length-2],upper[upper.length-1],point) <= 1e-10) upper.pop();
			upper.push(point);
			cursor--;
		}
		if (lower.length > 0) lower.pop();
		if (upper.length > 0) upper.pop();
		return lower.concat(upper);
	}
	static function cross(origin:Vector, first:Vector, second:Vector):Float
		return (first.x-origin.x)*(second.y-origin.y)-(first.y-origin.y)*(second.x-origin.x);
	static function hullContainsAt(hull:Array<Float>, x:Float, y:Float, z:Float):Bool {
		if (hull == null || hull.length % 3 != 0) return false;
		var points:Array<Vector> = [], lower = Math.POSITIVE_INFINITY, upper = Math.NEGATIVE_INFINITY;
		for (index in 0...Std.int(hull.length/3)) {
			var px = hull[index*3], py = hull[index*3+1], pz = hull[index*3+2];
			if (pz < lower) lower = pz;
			if (pz > upper) upper = pz;
			points.push(new Vector(px,py));
		}
		if (z < lower-1e-8 || z > upper+1e-8) return false;
		var polygon = convexHull(points), sign = 0;
		if (polygon.length < 3) return false;
		for (index in 0...polygon.length) {
			var side = cross(polygon[index],polygon[(index+1)%polygon.length],new Vector(x,y));
			if (Math.abs(side) <= 1e-8) continue;
			var next = side > 0 ? 1 : -1;
			if (sign != 0 && sign != next) return false;
			sign = next;
		}
		return true;
	}
	static function hullsContainAt(hulls:Array<Array<Float>>, x:Float, y:Float, z:Float):Bool {
		for (hull in hulls) if (hullContainsAt(hull,x,y,z)) return true;
		return false;
	}
	static function error(result:FasteningsResult, code:String, validCount:Int = 0):Void {
		var found = false;
		for (item in result.diagnostics.items) if (item.code == code) found = true;
		check(found, 'missing diagnostic $code: ${result.diagnostics.items}');
		check(result.fastenings.length == validCount, "a failed connection never yields a fastening");
	}
	static function fixture(screw:SocketHeadCapScrew, tap:MachineComponent = null, clamp:MachineComponent = null):FastenedAssembly {
		var assembly = new FastenedAssembly();
		assembly.addComponent("clamp", clamp == null ? new FasteningClamp() : clamp);
		assembly.addComponent("screw", screw);
		assembly.addComponent("tap", tap == null ? new FasteningTap(screw.thread.mating()) : tap);
		assembly.addMate("seat", "fixed", "clamp", "head", "screw", "head");
		return assembly;
	}
	public static function main():Void run();
	public static function run():Void {
		assertions = 0;
		var flange = new FlangeBearingMount(DeepGrooveBearing.metric("608"));
		var found = flange.fastenings().require();
		check(found.length == 4, "flange gives four fastenings");
		var unfitted = new FlangeBearingMount(DeepGrooveBearing.metric("608"), 10, false);
		check(flange.components().filter(item -> item.id == "flange/bearing").length == 1 &&
			unfitted.components().filter(item -> item.id == "flange/bearing").length == 0,
			"An unfitted housing omits the bearing while the default assembly retains it");
		var unfittedFastenings = unfitted.fastenings().require();
		check(unfittedFastenings.length == found.length, "Unfitted housing retains all CAD fastenings");
		for (i in 0...found.length) {
			check(unfittedFastenings[i].name() == found[i].name(), "Bearing fitting does not change fastening identity");
			near(unfittedFastenings[i].engagementLength, found[i].engagementLength);
		}
		var housingCollision = CollisionHullFacet.of(flange.flange.housing), plateCollision = CollisionHullFacet.of(flange.plate);
		if (housingCollision == null || plateCollision == null) throw "Flange fixture needs authored collision pieces";
		check(housingCollision.hulls.length < 128 && plateCollision.hulls.length < 128,
			'Flange collision decomposition stays compact: housing ${housingCollision.hulls.length}, plate ${plateCollision.hulls.length}');
		check(!hullsContainAt(housingCollision.hulls,0,0,flange.flange.housing.depth/2),
			"Flange collision pieces preserve the bearing bore");
		for (point in flange.flange.housing.boltPattern()) {
			check(!hullsContainAt(housingCollision.hulls,point.x,point.y,flange.flange.housing.depth/2),
				"Flange collision pieces preserve a mounting clearance");
			check(!hullsContainAt(plateCollision.hulls,point.x,point.y,flange.plate.thickness-1),
				"Plate collision pieces preserve each tapped entrance");
		}
		for (fastening in found) {
			near(fastening.engagementLength, flange.flange.screw.length - flange.flange.housing.depth);
			near(fastening.minimumEngagement, 1.5 * flange.flange.screw.diameter);
			check(fastening.threaded.member == "plate" && fastening.screw.member.indexOf("flange/screw") == 0, "occurrence identities survive flattening");
			check(fastening.angleWindow == null, "CAD does not invent qualified angle limits");
		}
		var shortened = new FlangeBearingAssembly(DeepGrooveBearing.metric("608"));
		shortened.replaceComponent("screw1", SocketHeadCapScrew.metric(shortened.screw.spec.size,
			shortened.housing.depth + 0.5 * shortened.screw.diameter));
		shortened.addMemberConnector("housing", "mount", Solids.axial(0, 0, 0));
		shortened.exposeConnector("mount", "housing", "mount");
		var shortMount = new FastenedAssembly();
		shortMount.addComponent("plate", new FlangeTappedPlate(shortened.housing, shortened.screw));
		shortMount.include("flange", shortened);
		var mount = shortMount.connector("flange/mount");
		shortMount.addMate("flange-seat", "fixed", "plate", "mount", mount.instanceId, mount.connectorName);
		error(shortMount.fastenings(), "fastening.under-engagement", 3);
		var name = found[0].name();
		flange.replaceComponent("plate", new FlangeTappedPlate(flange.flange.housing, flange.flange.screw, 22, 18));
		check(flange.fastenings().require()[0].name() == name, "plate depth edit keeps face-derived fastening name");
		flange.replaceComponent("plate", new FlangeTappedPlate(flange.flange.housing, flange.flange.screw, 20, 16, 2));
		var moved = flange.fastenings();
		check(moved.fastenings.length == 0, "offset holes never match by radius"); error(moved, "fastening.no-thread");

		var six = SocketHeadCapScrew.metric("M6", 12);
		var screwCollision = machinekit.component.CollisionHullFacet.of(six);
		if (screwCollision == null) throw "M6 screw has no authored collision envelope";
		var threadedHull = screwCollision.hulls[0];
		near(2 * Math.sqrt(threadedHull[0] * threadedHull[0] + threadedHull[1] * threadedHull[1]),
			six.thread.externalMinorDiameter());
		var basic = fixture(six);
		var one = basic.fastenings().require()[0];
		near(one.engagementLength, 7); near(one.minimumEngagement, 6);
		near(one.tightening.torqueNm, six.tightening().torqueNm);
		var rotated = new FastenedAssembly();
		rotated.include("work", basic, AssemblyFrames.fromRotationMatrix(40, 30, 20, [0.0, 0.0, 1.0, 0.0, 1.0, 0.0, -1.0, 0.0, 0.0]));
		var turned = rotated.fastenings().require()[0];
		near(turned.engagementLength, 7); near(turned.frame.normal.x, -1); near(turned.frame.origin.x, 35);
		var bolt = new FastenedAssembly();
		bolt.addComponent("clamp", new FasteningClamp());
		bolt.addComponent("screw", HexBolt.metric("M6", 12));
		bolt.addComponent("tap", new FasteningTap(six.thread.mating()));
		bolt.addMate("seat", "fixed", "screw", "head", "clamp", "head");
		near(bolt.fastenings().require()[0].engagementLength, 7);
		near(fixture(six, new FasteningConicalTap(six.thread.mating())).fastenings().require()[0].engagementLength, 7);
		error(fixture(SocketHeadCapScrew.metric("M6", 25), new FasteningConicalTap(six.thread.mating())).fastenings(), "fastening.blind-bottom");
		for (bored in [false, true]) {
			var blocked = new FastenedAssembly();
			blocked.addComponent("clamp", new FasteningClamp()); blocked.addComponent("screw", six);
			blocked.addComponent("tap", new FasteningTap(six.thread.mating()), AssemblyFrames.translation(0, 0, -2));
			blocked.addComponent("intervening", new FasteningObstacle(bored));
			blocked.addMate("seat", "fixed", "clamp", "head", "screw", "head");
			error(blocked.fastenings(), bored ? "fastening.no-thread" : "fastening.blocked-path");
		}
		basic.replaceComponent("screw", SocketHeadCapScrew.metric("M6", 8));
		error(basic.fastenings(), "fastening.under-engagement");
		basic.replaceComponent("screw", SocketHeadCapScrew.metric("M6", 25));
		error(basic.fastenings(), "fastening.blind-bottom");
		error(fixture(SocketHeadCapScrew.metric("M6", 40)).fastenings(), "fastening.plain-shank");
		var unspecified = fixture(six, new FasteningTap(six.thread.mating(), "bronze"));
		error(unspecified.fastenings(), "fastening.engagement-material");
		var specified = unspecified.join("screw", 1);
		near(specified.fastenings().require()[0].engagementLength, 7);
		var unseated = new FastenedAssembly();
		unseated.addComponent("missing", six);
		var missing = unseated.join("missing");
		error(missing.fastenings(), "fastening.missing-seat");
		error(fixture(six, null, new FasteningClamp(5.5)).fastenings(), "fastening.clearance-small");
		error(fixture(six, new FasteningTap(new ThreadSpec("M6x0.75", 6, 0.75, 1, RightHand, true))).fastenings(), "fastening.thread-mismatch");
		error(fixture(six, new FasteningTap(new ThreadSpec("M6", 6, 1, 1, LeftHand, true))).fastenings(), "fastening.thread-mismatch");
		error(fixture(six, new FasteningTap(new ThreadSpec("M6", 6, 1, 2, RightHand, true))).fastenings(), "fastening.thread-mismatch");
		error(fixture(six, new FasteningTap(new ThreadSpec("M5", 5, 0.8, 1, RightHand, true))).fastenings(), "fastening.thread-mismatch");
		error(fixture(six, new FasteningTap(new ThreadSpec("M6", 6, 1, 1, RightHand, true, "nominal", Trapezoidal))).fastenings(), "fastening.thread-mismatch");
		var aluminium = fixture(six, new FasteningTap(six.thread.mating(), "aluminium 6061"));
		error(aluminium.fastenings(), "fastening.under-engagement");
		var ambiguous = fixture(six);
		ambiguous.addComponent("other", new FasteningTap(six.thread.mating()));
		error(ambiguous.fastenings(), "fastening.ambiguous-thread");
		var fine = SocketHeadCapScrew.custom({size: "M6x0.75", diameter: 6, pitch: 0.75, headDiameter: 10,
			headHeight: 6, socketSize: 5, socketDepth: 3, threadLength: 18, tapDrill: 5.2,
			clearanceFine: 6.4, clearanceMedium: 6.6, clearanceCoarse: 7,
			counterboreDiameter: 11, counterboreDepth: 6.5}, 12);
		error(fixture(fine).fastenings(), "fastening.tightening-data");

		var nut = HexNut.metric("M6");
		// Put the nut's top bearing face on the clamp's underside; it is a through bore.
		var nutAssembly = new FastenedAssembly();
		nutAssembly.addComponent("clamp", new FasteningClamp());
		nutAssembly.addComponent("screw", six);
		nutAssembly.addComponent("nut", nut, AssemblyFrames.translation(0, 0, -5 - nut.height));
		nutAssembly.addMate("seat", "fixed", "clamp", "head", "screw", "head");
		var declared = nutAssembly.join("screw", nut.height / six.diameter, {minimum: 0.1, maximum: 0.3});
		var nutFastening = nutAssembly.fastenings().require()[0];
		near(nutFastening.engagementLength, nut.height);
		check(nutFastening.angleWindow != null, "authored angle limits are retained");
		var angle:TighteningAngleWindow = cast nutFastening.angleWindow;
		angle.maximum = 1;
		var rebuiltAngle:TighteningAngleWindow = cast nutAssembly.fastenings().require()[0].angleWindow;
		near(rebuiltAngle.maximum, 0.3);
		var nema = new NemaMotorMount();
		var motorFastenings = nema.fastenings().require();
		check(motorFastenings.length == 4, "NEMA has four M3 fastenings");
		for (fastening in motorFastenings) near(fastening.engagementLength, 4);
		Sys.println('Fastenings: $assertions assertions passed');
	}
}
