import cadkit.modeling.Location;
import cadkit.modeling.Part;
import cadkit.modeling.Plane;
import cadkit.modeling.Vector;
import machinekit.component.ComponentDetail;
import machinekit.component.MachineComponent;
import machinekit.component.Solids;

/** One machined feature. `approach` is the unit tool axis pointing out of the material, towards the spindle;
 * `entry` is where the tool axis crosses the surface the feature starts from. The cutter is a half-section
 * (r, depth) revolved about the tool axis, depth measured into the material from `entry`.
 */
class HousingFeature {
	public final id:String;
	public final kind:String;
	public final setup:String;
	public final entry:Vector;
	public final approach:Vector;
	public final section:Array<{r:Float, z:Float}>;
	public final tolerance:String;
	public final source:String;

	public function new(id:String, kind:String, setup:String, entry:Vector, approach:Vector, section:Array<{r:Float, z:Float}>,
			tolerance:String, source:String) {
		this.id = id;
		this.kind = kind;
		this.setup = setup;
		this.entry = entry;
		this.approach = approach.normalized();
		this.section = section;
		this.tolerance = tolerance;
		this.source = source;
	}

	public function diameter():Float {
		var r = 0.0;
		for (point in section) r = Math.max(r, point.r);
		return 2 * r;
	}

	public function depth():Float {
		var z = 0.0;
		for (point in section) z = Math.max(z, point.z);
		return z;
	}

	/** The removed material, extended 1 mm outside the entry surface so coincident faces cut cleanly. */
	public function cutter():Part {
		var profile = [for (point in section) {r: point.r, z: point.z == 0 ? -1.0 : point.z}];
		var local = Part.revolve(profile);
		var into = approach.scale(-1), reference = Math.abs(into.z) < 0.9 ? Vector.Z() : Vector.X();
		var xDirection = reference.subtract(into.scale(reference.dot(into))).normalized();
		var placed = local.placed(new Location(new Plane(entry, xDirection, into)));
		local.close();
		return Solids.named(placed, id);
	}
}

/** A 4th-axis setup: rotary axis `axis` (part frame, through `centre`), index 0 presents `zero` to the spindle and
 * positive index turns `axis × zero` towards it.
 */
class HousingSetup {
	public final id:String;
	public final label:String;
	public final axis:Vector;
	public final zero:Vector;
	public final centre:Vector;

	public function new(id:String, label:String, axis:Vector, zero:Vector, centre:Vector) {
		this.id = id;
		this.label = label;
		this.axis = axis.normalized();
		this.zero = zero.normalized();
		this.centre = centre;
	}

	/** Index angle in degrees [0, 360) that points `approach` at the spindle, or null if it lies off the rotary plane. */
	public function index(approach:Vector):Null<Float> {
		if (Math.abs(approach.dot(axis)) > 1e-9) return null;
		var angle = Math.atan2(approach.dot(axis.cross(zero)), approach.dot(zero)) * 180 / Math.PI;
		angle = Math.round(angle * 1e6) / 1e6;
		return angle < 0 ? angle + 360 : angle;
	}
}

/** Machinable reference for an eRob70T-class T housing (DRILL_TAP_PLAN.md DT0), in the supplier STEP frame:
 * drum axis +Z with the output at +Z, stem axis -Y with the link socket at the -Y end, millimetres.
 *
 * Exterior and link socket follow ZeroErr's 70T interface drawing and `eRob70H××T.STEP` (solid 8). The supplier
 * model fills the drum, so the inside is a representative size-14 strain-wave stack built from catalogue-class
 * sizes and labelled assumptions: a machining reference, not an actuator design.
 */
class ReferenceHousing extends MachineComponent {
	public static inline var RADIUS:Float = 36.5;
	public static inline var FRONT:Float = 40;
	public static inline var REAR:Float = -45;
	public static inline var SOCKET_FLOOR:Float = -39;
	public static inline var SOCKET_FACE:Float = -48.5;
	public static inline var BOSS_RADIUS:Float = 35.5;
	public static inline var SOCKET_RADIUS:Float = 29;
	public static inline var LINK_ROW:Float = -43;
	public static inline var PILOT_RADIUS:Float = 35;
	public static inline var PILOT_LENGTH:Float = 8;
	/** 6061-T6, g/mm³. */
	public static inline var DENSITY:Float = 2.70e-3;

	/** Square bar: X along the bar, section centred on the Op 1 rotary axis. */
	public static inline var BAR:Float = 90;
	public static inline var BAR_LENGTH:Float = 155;
	public static inline var GRIP:Float = 20;
	public static inline var TAB_RADIUS:Float = 15;
	public static final BAR_CENTRE = new Vector(0, -6, -2.5);

	static inline var STEP = "eRob70H××T.STEP solid 8";
	static inline var DRAWING = "ZeroErr eRob70T interface drawing";
	static inline var ASSUMED = "assumed, representative size-14 stack";

	public final features:Array<HousingFeature>;
	public final setups:Array<HousingSetup>;

	public function new() {
		super("REFERENCE-HOUSING-70T", "Machinable eRob70T-class T housing (DT0 reference)", "aluminium 6061", true);
		var c = BAR_CENTRE;
		setups = [
			new HousingSetup("op1", "Op 1: bar ends in chuck and tailstock, A across drum and stem", Vector.X(), Vector.Z(), c),
			new HousingSetup("op2", "Op 2: front register on an angle bracket, A along the stem", new Vector(0, -1, 0), Vector.X(),
				new Vector(0, 0, 0))
		];
		features = [];
		var front = Vector.Z(), rear = new Vector(0, 0, -1), socket = new Vector(0, -1, 0);

		// Op 1, index 0: the front bore stack, every precision diameter stepping down away from the spindle.
		add("front-face", "face", "op1", new Vector(0, 0, FRONT), front, [], "flat 0.01", STEP);
		add("output-recess", "bore", "op1", new Vector(0, 0, FRONT), front, bore(35.2, 3.5), "±0.05", STEP);
		add("gear-register", "precision-bore", "op1", new Vector(0, 0, 36.5), front, bore(33, 10), "Ø66 H7, datum A", ASSUMED);
		add("stator-bore", "precision-bore", "op1", new Vector(0, 0, 26.5), front, bore(25, 36.5), "Ø50 H7, ◎0.01 A", ASSUMED);
		add("rotor-bearing-seat", "precision-bore", "op1", new Vector(0, 0, -10), front, bore(21, 7), "Ø42 K6 (6806), ◎0.01 A",
			"ISO 15 bearing 6806, 30 × 42 × 7");
		add("bearing-shoulder", "bore", "op1", new Vector(0, 0, -17), front, bore(19, 3), "±0.1", ASSUMED);
		for (i in 0...12) {
			var a = i * Math.PI / 6;
			add('gear-screw-$i', "form-tap M3", "op1", new Vector(29 * Math.cos(a), 29 * Math.sin(a), 26.5), front, bore(1.4, 8),
				"M3 6H, 6 full threads", ASSUMED + "; axial bolt circle replaces rim screws");
		}
		add("drum-od-front", "contour", "op1", new Vector(RADIUS, 0, FRONT), front, [], "Ø73 ±0.05", DRAWING);

		// Op 1, index 90: the link socket, cut in the same clamping as the drum so 39 ± 0.05 and squareness hold.
		add("socket-face", "face", "op1", new Vector(0, SOCKET_FACE, 0), socket, [], "flat 0.01", DRAWING);
		add("link-socket", "precision-bore", "op1", new Vector(0, SOCKET_FACE, 0), socket, bore(SOCKET_RADIUS, 9.5),
			"Ø58 G7, datum B, ⟂0.02 A, floor 39 ± 0.05 from A", DRAWING);
		add("stem-pocket", "pocket", "op1", new Vector(0, SOCKET_FLOOR, 0), socket, bore(16, 8), "±0.1", ASSUMED + "; lightening");
		add("cable-hole", "hole", "op1", new Vector(0, SOCKET_FLOOR, -24), socket, bore(5, 12), "±0.1",
			ASSUMED + "; socket to rear cavity");
		add("stem-od", "contour", "op1", new Vector(BOSS_RADIUS, SOCKET_FACE, 0), socket, [], "Ø71 / Ø73 ±0.05", STEP);

		// Op 1, index 180: rear cavity, cover pilot and cover screws.
		add("rear-face", "face", "op1", new Vector(0, 0, REAR), rear, [], "flat 0.02", STEP);
		add("rear-cavity", "pocket", "op1", new Vector(0, 0, REAR), rear, bore(28, 25), "±0.05", ASSUMED + "; boards and brake");
		add("rear-pilot", "contour", "op1", new Vector(PILOT_RADIUS, 0, REAR), rear, [], "Ø70 h8", ASSUMED);
		for (i in 0...6) {
			var a = Math.PI / 6 + i * Math.PI / 3;
			add('cover-screw-$i', "form-tap M3", "op1", new Vector(31.75 * Math.cos(a), 31.75 * Math.sin(a), REAR), rear, bore(1.4, 8),
				"M3 6H, 5.0 engagement (manual)", ASSUMED);
		}
		add("drum-od-rear", "contour", "op1", new Vector(RADIUS, 0, REAR), rear, [], "Ø73 ±0.05", DRAWING);

		// Op 2: ZeroErr's radial link screws around the socket boss, and the bar-end tabs.
		for (degrees in [0, 30, 60, 120, 150, 180, 210, 240, 270, 300, 330]) {
			var a = degrees * Math.PI / 180, out = new Vector(Math.cos(a), 0, Math.sin(a));
			add('link-cbore-$degrees', "counterbored hole", "op2", new Vector(BOSS_RADIUS * out.x, LINK_ROW, BOSS_RADIUS * out.z), out,
				[{r: 0.0, z: 0.0}, {r: 2.9, z: 0.0}, {r: 2.9, z: 3.4}, {r: 1.65, z: 3.4}, {r: 1.65, z: 7}, {r: 0.0, z: 7}],
				"Ø3.3 thru, ⌴Ø5.8 × 3.4", DRAWING);
		}
		for (degrees in [90, 225, 315]) {
			var a = degrees * Math.PI / 180, out = new Vector(Math.cos(a), 0, Math.sin(a));
			add('link-csk-$degrees', "countersunk hole", "op2", new Vector(BOSS_RADIUS * out.x, LINK_ROW, BOSS_RADIUS * out.z), out,
				[{r: 0.0, z: 0.0}, {r: 3.3, z: 0.0}, {r: 1.7, z: 1.6}, {r: 1.7, z: 7}, {r: 0.0, z: 7}], "Ø3.4 thru, ⌵Ø6.6 × 90°",
				DRAWING);
		}
		for (side in [1, -1]) add(side > 0 ? "tab-px" : "tab-nx", "tab removal", "op2", new Vector(side * RADIUS, c.y, c.z),
			new Vector(side, 0, 0), [], "blend to Ø73, ±0.1", "stock holding");
	}

	static function bore(radius:Float, depth:Float):Array<{r:Float, z:Float}>
		return [{r: 0.0, z: 0.0}, {r: radius, z: 0.0}, {r: radius, z: depth}, {r: 0.0, z: depth}];

	function add(id:String, kind:String, setup:String, entry:Vector, approach:Vector, section:Array<{r:Float, z:Float}>, tolerance:String,
			source:String):Void
		features.push(new HousingFeature(id, kind, setup, entry, approach, section, tolerance, source));

	override public function hasGeometry():Bool return true;

	/** Finished exterior before any feature is cut. */
	public static function envelope():Part {
		var drum = Part.cylinderSpan(RADIUS, REAR, FRONT);
		var stem = Part.cylinderAlongY(RADIUS, SOCKET_FLOOR, 0);
		var boss = Part.cylinderAlongY(BOSS_RADIUS, SOCKET_FACE, SOCKET_FLOOR);
		var body = Solids.union([drum, stem, boss]);
		var pilotRing = Part.revolve([{r: PILOT_RADIUS, z: REAR - 1}, {r: RADIUS + 1, z: REAR - 1}, {r: RADIUS + 1, z: REAR + PILOT_LENGTH},
			{r: PILOT_RADIUS, z: REAR + PILOT_LENGTH}]);
		return Solids.cut(body, [pilotRing]);
	}

	override public function geometry(detail:ComponentDetail = Preview):Part {
		var body = envelope();
		if (detail == Envelope) return body;
		return Solids.cut(body, [for (feature in features) if (feature.section.length > 0) feature.cutter()]);
	}

	/** 90 mm square bar with the two holding tabs left by Op 1: the stock state between the operations is this part
	 * fused with `tabs()`. */
	public static function stock():Part {
		var bar = Part.box(BAR_LENGTH, BAR, BAR);
		var placed = bar.translated(new Vector(0, BAR_CENTRE.y, BAR_CENTRE.z - BAR / 2));
		bar.close();
		return placed;
	}

	public static function tabs():Part {
		var reach = BAR_LENGTH / 2 - GRIP, c = BAR_CENTRE;
		return Solids.union([for (side in [1, -1]) Part.cylinderAlong(TAB_RADIUS, new Vector(side * (RADIUS - 2), c.y, c.z),
			new Vector(side, 0, 0), reach - RADIUS + 2)]);
	}

	public function featuresIn(setup:String):Array<HousingFeature> return [for (feature in features) if (feature.setup == setup) feature];

	public function feature(id:String):HousingFeature {
		for (feature in features) if (feature.id == id) return feature;
		throw 'Unknown housing feature $id';
	}

	public function setup(id:String):HousingSetup {
		for (setup in setups) if (setup.id == id) return setup;
		throw 'Unknown housing setup $id';
	}
}
