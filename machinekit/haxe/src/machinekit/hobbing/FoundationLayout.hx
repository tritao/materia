package machinekit.hobbing;

import materia.assembly.AssemblyRecord.AssemblyFrame;
import materia.assembly.AssemblyFrames as F;
import materia.casting.CastingFeature;
import materia.casting.CastingPatterns as C;
import materia.casting.BoxAlgebra;
import materia.casting.InsertSpec;
import machinekit.component.ComponentValues;
import machinekit.structural.CastInsert;
import machinekit.structural.CastingFeaturePatterns as Patterns;
import machinekit.structural.MineralFoundation.CastInsertPlacement;
import machinekit.structural.InsertHoles;

typedef InsertLocation = {
	var id:String;
	var role:InsertRole;
	var label:String;
	var finish:InsertFinish;
	var width:Float;
	var depth:Float;
	var height:Float;
	var pose:AssemblyFrame;
	var openings:Array<CastingOpening>;
	@:optional var clampHeight:Float;
}

/** Machine design intent. Exterior openings are authored here, never inferred. */
class FoundationLayout {
	public static inline final TABLE_FACE = 228.0;
	public static inline final TABLE_SEAT = TABLE_FACE - 210.0;
	public static inline final TABLE_WELL_FLOOR = TABLE_SEAT - 10.0;
	public static inline final PLINTH_HEIGHT = 800.0;
	public static inline final UPPER_OFFSET = 590.0;
	public static inline final BED_RAIL_CENTRE = 220.0;
	public static inline final BED_RAIL_SEAT_WIDTH = 50.0;
	public static inline final BED_RAIL_BEAM_WIDTH = 110.0;
	public static inline final FOOT_FLOOR = 40.0;
	public static inline final FOOT_ROOF = 160.0;
	public static inline final FOOT_SUPPORT_TOP = 230.0;
	public static inline final FOOT_ACCESS_WIDTH = 140.0;
	public static inline final FOOT_BOSS_WIDTH = 190.0;
	public static inline final FOOT_STEM_CLEARANCE = 28.0;
	public static inline final FOOT_REGISTER_CLEARANCE = 1.0;
	public static inline final FOOT_PLATE_THICKNESS = 15.0;

	static function faces(names:Array<String>):Array<CastingOpening>
		return [for (name in names) C.face(name)];

	/** Casting-local bounds for the cap, twin webs and bearing cross-ribs. */
	public static function driveSupports():Array<Array<Float>> {
		var layout = HobberXLayout.front(), start = 140 - layout.bedCentre, end = layout.bedRear - layout.bedCentre;
		var supports = [[start, -150.0, 660.0, end, 150.0, 720.0]];
		for (side in [-1, 1])
			supports.push([start, side * 112.5 - 17.5, 0.0, end, side * 112.5 + 17.5, 720.0]);
		for (station in [layout.floatingCentre, layout.screwStart])
			supports.push([station - layout.bedCentre - 15, -275.0, 0.0, station - layout.bedCentre + 15, 275.0, 720.0]);
		return supports;
	}

	public static function enclosureSeats():Array<InsertLocation> {
		var out:Array<InsertLocation> = [];
		for (x in [-1, 1]) for (y in [-1, 1])
			out.push({
				id: 'enclosureSeat${x + 1}${y + 1}', role: EnclosureMount, label: "enclosure frame dry-side mounting pad",
				finish: GroundAfterCasting, width: 80, depth: 80, height: 20,
				openings: faces([y < 0 ? "body.y-min" : "body.y-max"]),
				pose: F.fromRotationMatrix(x * (HobberXLayout.front().bedLength / 2 - 100), y * 380, 720,
					[1.0, 0, 0, 0, 0, y, 0, -y, 0])
			});
		return out;
	}

	public static function trayLedges(depth:Float = machinekit.enclosure.TraySeat.DEPTH):Array<CastingFeature> {
		var d = dimensions("bed"), w = d[0], l = d[1], a = machinekit.enclosure.TraySeat.WIDTH;
		return [
			Patterns.box("tray-front-ledge", InterfaceRecess, faces(["body.x-min", "body.y-min", "body.y-max", "body.z-max"]),
				a, l, depth, -w / 2 + a / 2, 0, d[2] - depth, "tray"),
			Patterns.box("tray-rear-ledge", InterfaceRecess, faces(["body.x-max", "body.y-min", "body.y-max", "body.z-max"]),
				a, l, depth, w / 2 - a / 2, 0, d[2] - depth, "tray"),
			Patterns.box("tray-left-ledge", InterfaceRecess, faces(["body.y-min", "body.z-max"]),
				w - 2 * a, a, depth, 0, -l / 2 + a / 2, d[2] - depth, "tray"),
			Patterns.box("tray-right-ledge", InterfaceRecess, faces(["body.y-max", "body.z-max"]),
				w - 2 * a, a, depth, 0, l / 2 - a / 2, d[2] - depth, "tray")
		];
	}

	public static function trayClampSeats():Array<InsertLocation> {
		var out:Array<InsertLocation> = [], d = dimensions("bed");
		function add(x:Float, y:Float, height:Float = 11):Void
			out.push({id: "trayClampSeat" + out.length, role: ThreadedAnchor, label: "tray clamp seat", finish: GroundAfterCasting,
				width: 20, depth: 20, height: 20, openings: faces(["body.z-max"]),
				pose: F.translation(x, y, d[2] - 20), clampHeight: height});
		for (side in [-1, 1]) for (x in [-1, 0, 1]) add(x * (d[0] / 2 - 100), side * (d[1] / 2 - 35));
		for (side in [-1, 1]) for (y in [-150.0, 150.0]) add(side * (d[0] / 2 - 35), y);
		for (y in [-75.0, 75.0]) add(d[0] / 2 - 35, y, 17);
		return out;
	}

	public static function feet():Array<{x:Float, y:Float}>
		return [for (i in 0...3) {x: (i < 2 ? -1 : 1) * (HobberXLayout.front().bedLength / 2 - 120),
			y: i == 0 ? -330.0 : i == 1 ? 330.0 : 0.0}];

	public static function dimensions(kind:String):Array<Float>
		return switch kind {
			case "bed": [HobberXLayout.front().bedLength, 800, PLINTH_HEIGHT];
			case "column": [400.0, 200, 500];
			default: throw "Unknown hobber foundation kind";
		};

	public static function bodyValues(kind:String):ComponentValues {
		var d = dimensions(kind);
		var cores = kind == "bed" ? C.grid(d[0], d[1], d[2], 25, 85, 25, 35, 2, 1)
			: C.grid(d[0], d[1], d[2], 20, 20, 20, 20);
		if (kind == "bed") {
			var layout = HobberXLayout.front(), centre = layout.bedCentre;
			var supports = [for (side in [-1, 1]) BoxAlgebra.fromLimits([
				-d[0] / 2, side * BED_RAIL_CENTRE - BED_RAIL_BEAM_WIDTH / 2, 0,
				d[0] / 2, side * BED_RAIL_CENTRE + BED_RAIL_BEAM_WIDTH / 2, d[2]
			])];
			supports.push(BoxAlgebra.fromLimits([-140.0 - centre, -140, 0, 140 - centre, 140, d[2]]));
			for (bounds in driveSupports()) supports.push(BoxAlgebra.fromLimits(bounds));
			for (seat in enclosureSeats()) supports.push(BoxAlgebra.fromLimits([
				seat.pose.x - 60, seat.pose.y < 0 ? -400.0 : 330.0, 660.0,
				seat.pose.x + 60, seat.pose.y < 0 ? -330.0 : 400.0, 780.0
			]));
			for (seat in trayClampSeats()) supports.push(BoxAlgebra.fromLimits([
				seat.pose.x - 20, seat.pose.y - 20, 740.0, seat.pose.x + 20, seat.pose.y + 20, 800.0
			]));
			for (p in feet()) supports.push(BoxAlgebra.fromLimits([
				p.x - FOOT_BOSS_WIDTH / 2, p.y - FOOT_BOSS_WIDTH / 2, 0,
				p.x + FOOT_BOSS_WIDTH / 2, p.y + FOOT_BOSS_WIDTH / 2, FOOT_SUPPORT_TOP
			]));
			cores = Patterns.retain(cores, supports);
			var footCentres = feet();
			for (i in 0...footCentres.length) {
				var p = footCentres[i], alongX = i == 2;
				var opening = alongX ? "body.x-max" : p.y < 0 ? "body.y-min" : "body.y-max";
				var edge = alongX ? d[0] / 2 : p.y < 0 ? -d[1] / 2 : d[1] / 2;
				cores = cores.concat(Patterns.foot({access: 'foot$i-access', stem: 'foot$i-stem', register: 'foot$i-plate-register'},
					p.x, p.y, edge, alongX, FOOT_ACCESS_WIDTH, FOOT_ACCESS_WIDTH, FOOT_FLOOR, FOOT_ROOF,
					FOOT_STEM_CLEARANCE, FOOT_SUPPORT_TOP, faces([opening]),
					faces(["body.z-min"]).concat([C.feature('core-${i < 2 ? 0 : 1}-0')]), {
						owner: "footSeat" + i, openings: [C.feature('foot$i-access')],
						width: machinekit.structural.LevelingSeat.PLATE_WIDTH + FOOT_REGISTER_CLEARANCE,
						depth: machinekit.structural.LevelingSeat.PLATE_WIDTH + FOOT_REGISTER_CLEARANCE,
						thickness: FOOT_PLATE_THICKNESS
					}));
			}
			cores.push(Patterns.box("work-well", ServiceOpening, faces(["body.z-max"]),
				410, 350, 220 - TABLE_WELL_FLOOR, -45 - centre, 0, TABLE_WELL_FLOOR + UPPER_OFFSET));
			cores.push(Patterns.box("chip-drain", ServiceOpening, [], 280, 180, 80, -50 - centre, 0, TABLE_WELL_FLOOR - 80 + UPPER_OFFSET));
			var channelFront = layout.deckStart - 12.5;
			cores.push(Patterns.box("X-drive-channel", ServiceOpening, faces(["body.x-max", "body.z-max"]),
				layout.bedRear - channelFront, 110, 90, (layout.bedRear + channelFront) / 2 - centre, 0, 130 + UPPER_OFFSET));
			cores.push(Patterns.box("coolant-reserved-volume", Clearance, [], 120, 220, 300, -215 - centre, 0, 150));
			cores.push(Patterns.box("coolant-front-access", ServiceOpening, faces(["body.x-min"]), 40, 240, 320, -285 - centre, 0, 140));
			cores.push(Patterns.box("chip-downcomer", ServiceOpening, [], 80, 100, TABLE_WELL_FLOOR + 390, -200 - centre, 0, 200));
			cores.push(Patterns.box("rotary-union-access", Clearance, faces(["body.z-min"]).concat([C.feature("work-well")]),
				40, 40, TABLE_SEAT + UPPER_OFFSET, -centre, 0, 0));
			cores = cores.concat(trayLedges());
			cores.push(Patterns.box("tray-rear-bridge-seat", InterfaceRecess, faces(["body.x-max", "body.z-max"]),
				25, 150, 10, d[0] / 2 - 12.5, 0, d[2] - 10, "tray-rear-bridge"));
			cores.push(Patterns.box("cable-route", ServiceOpening, [], layout.bedLength - 200, 30, 40, 0, 300, 500));
		} else {
			cores.push(Patterns.box("head-recess", ServiceOpening, faces(["body.y-max", "body.z-min", "body.z-max"]),
				240, 100, d[2], 0, 60, 0));
			cores = Patterns.retain(cores, [BoxAlgebra.fromLimits([-60.0, -100, 0, 60, HobberZLayout.SPINE_FRONT_COLUMN_Y, 500])]);
		}
		return new ComponentValues().setNumber("width", d[0]).setNumber("depth", d[1]).setNumber("height", d[2])
			.setToken("features", C.encode(cores));
	}

	public static function placements(kind:String):Array<CastInsertPlacement>
		return [for (row in inserts(kind)) {id: row.id, part: insert(row), pose: row.pose, openings: row.openings}];

	public static function inserts(kind:String):Array<InsertLocation> {
		dimensions(kind);
		var out:Array<InsertLocation> = [];
		function add(id:String, role:InsertRole, label:String, w:Float, d:Float, h:Float,
			x:Float, y:Float, z:Float, openings:Array<String>):Void
			out.push({id: id, role: role, label: label, finish: GroundAfterCasting, width: w, depth: d, height: h,
				pose: F.translation(x, y, z), openings: faces(openings)});
		if (kind == "bed") {
			var layout = HobberXLayout.front(), centre = layout.bedCentre;
			out = enclosureSeats().concat(trayClampSeats());
			add("tablePlate", TableMount, "table seat", 280, 280, 20, -centre, 0, TABLE_SEAT - 20 + UPPER_OFFSET, []);
			for (side in [-1, 1]) out.push({id: side < 0 ? "railLeft" : "railRight", role: GuideSeat, label: "preloaded column guide seat",
				finish: GroundAfterCasting, width: BED_RAIL_SEAT_WIDTH, depth: layout.railLength, height: 20,
				pose: F.fromRotationMatrix(layout.railStart + layout.railLength / 2 - centre, side * BED_RAIL_CENTRE,
					200 + UPPER_OFFSET, [0.0, 1, 0, -1, 0, 0, 0, 0, 1]), openings: faces(["body.z-max"])});
			out.push({id: "driveSeat", role: DriveMount, label: "central X screw channel seat", finish: GroundAfterCasting,
				width: 100, depth: layout.motorFace - layout.deckStart, height: 20,
				pose: F.fromRotationMatrix((layout.motorFace + layout.deckStart) / 2 - centre, 0, 120 + UPPER_OFFSET,
					[0.0, 1, 0, -1, 0, 0, 0, 0, 1]), openings: []});
			for (side in [-1, 1]) add(side < 0 ? "liftLeft" : "liftRight", LiftingMount, "lifting point",
				30, 30, 30, -150 - centre, side * 310, 180 + UPPER_OFFSET, ["body.z-max"]);
		} else {
			add("basePlate", ColumnMount, "column carriage sole", 380, 100, 20, 0, -40, -10, ["body.z-min"]);
			for (side in [-1, 1]) out.push({id: side < 0 ? "railLeft" : "railRight", role: GuideSeat, label: "Z rail seat",
				finish: GroundAfterCasting, width: 35, depth: 480, height: 20,
				pose: F.fromRotationMatrix(side * 145, 90, 245, [1.0, 0, 0, 0, 0, 1, 0, -1, 0]), openings: faces(["body.y-max"])});
			out.push({id: "driveSeat", role: DriveMount, label: "central Z drive back seat", finish: GroundAfterCasting,
				width: 100, depth: 460, height: 20,
				pose: F.fromRotationMatrix(0, HobberZLayout.SEAT_COLUMN_Y, 250, [-1.0, 0, 0, 0, 0, 1, 0, 1, 0]), openings: []});
			for (side in [-1, 1]) add(side < 0 ? "liftLeft" : "liftRight", LiftingMount, "lifting point",
				25, 25, 20, side * 160, -40, 490, ["body.z-max"]);
		}
		return out;
	}

	public static function insert(row:InsertLocation):CastInsert {
		var holes = row.role == ThreadedAnchor ? InsertHoles.tappedBlind(InsertHoles.metricThread(machinekit.standard.FastenerThreads.coarse(6).mating()), row.height)
			: row.role == TableMount ? InsertHoles.through(40, row.height) : InsertHoles.fromNeutral([]);
		return new CastInsert(new ComponentValues().setNumber("width", row.width).setNumber("depth", row.depth)
			.setNumber("height", row.height).setToken("holes", holes).setToken("role", row.role)
			.setToken("label", row.label).setToken("finish", row.finish));
	}
}
