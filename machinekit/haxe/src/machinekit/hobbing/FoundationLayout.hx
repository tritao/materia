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
	var part:CastInsert;
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
	public static inline final TABLE_SEAT_HEIGHT = 608.0;
	public static inline final TABLE_WELL_FLOOR_HEIGHT = 598.0;
	public static inline final WORK_WELL_TOP = 810.0;
	public static inline final WORK_WELL_WIDTH = 410.0;
	public static inline final WORK_WELL_DEPTH = 350.0;
	public static inline final WORK_WELL_CENTRE_X = -45.0;
	public static inline final DRIVE_CHANNEL_FRONT_CLEARANCE = 12.5;
	public static inline final DRIVE_CHANNEL_FLOOR = 720.0;
	public static inline final DRIVE_CHANNEL_WIDTH = 110.0;
	public static inline final CHIP_DRAIN_DEPTH = 80.0;
	public static inline final CHIP_DRAIN_WIDTH = 180.0;
	public static inline final RAIL_SEAT_HEIGHT = 790.0;
	public static inline final DRIVE_SEAT_HEIGHT = 710.0;
	public static inline final LIFT_SEAT_HEIGHT = 770.0;
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
	public static inline final FOOT_PLATE_WIDTH = 110.0;
	public static inline final PLINTH_CORE_FLOOR = 85.0;
	public static inline final PLINTH_CORE_WEB = 35.0;
	public static inline final DRIVE_SUPPORT_FRONT = 140.0;
	public static inline final DRIVE_CAP_UNDERSIDE = 660.0;
	public static inline final DRIVE_RIB_HALF_WIDTH = 15.0;
	public static inline final PRECAST_STRUT_SECTION = 50.0;

	static function faces(names:Array<String>):Array<CastingOpening>
		return [for (name in names) C.face(name)];

	/** Three non-collinear contacts on each side of an enclosed core. The upper
	 * struts reach the open pour face and bear against a clamped tooling bridge.
	 * Fifty-mm square sections are source geometry, not a material capacity claim.
	 */
	public static function precastStruts(kind:String):Array<materia.casting.PrecastStrut> {
		dimensions(kind);
		if (kind != "bed") return [];
		var result:Array<materia.casting.PrecastStrut> = [], half = PRECAST_STRUT_SECTION / 2;
		for (core in [
			// The second pair clears the retained tray socket at X365/Y-365.
			{id:"front-band",featureId:"pocket-centreSocketRight-leftWall-floor",top:PLINTH_HEIGHT-25,sites:[{x:80.0,y:-350.0},{x:320.0,y:-350.0},{x:210.0,y:-300.0}]},
			// Keep the upper struts outside the source X-drive channel's +/-55 mm band.
			{id:"drive-pocket",featureId:"pocket-driveRibRear-leftWebInside-floor",top:DRIVE_CAP_UNDERSIDE,sites:[{x:100.0,y:-80.0},{x:220.0,y:-80.0},{x:160.0,y:80.0}]}
		]) for (index in 0...core.sites.length) {
			var site = core.sites[index];
			for (upper in [false,true]) result.push({
				id:"precast-"+core.id+"-"+(upper?"upper":"lower")+"-"+index,
				coreFeatureId:core.featureId,material:materia.casting.PrecastStrut.StrutMaterial.CastingCompound,
				contact:upper?materia.casting.PrecastStrut.StrutContact.Upper:materia.casting.PrecastStrut.StrutContact.Lower,
				box:{x:site.x-half,y:site.y-half,z:upper?core.top:0,width:PRECAST_STRUT_SECTION,
					depth:PRECAST_STRUT_SECTION,height:upper?PLINTH_HEIGHT-core.top:PLINTH_CORE_FLOOR}
			});
		}
		return materia.casting.PrecastStrut.PrecastStruts.copy(result);
	}

	static function boxInsert(id:String, role:InsertRole, label:String, w:Float, d:Float, h:Float,
			pose:AssemblyFrame, openings:Array<String>, holes:String):InsertLocation {
		return {id:id,part:new CastInsert(CastInsert.boxValues(w,d,h).setToken("role",role)
			.setToken("label",label).setToken("finish",GroundAfterCasting).setToken("holes",holes)),
			pose:pose,openings:faces(openings)};
	}

	public static function enclosureSeats():Array<InsertLocation> {
		var out:Array<InsertLocation> = [];
		for (x in [-1,1]) for (y in [-1,1])
			out.push(boxInsert('enclosureSeat${x+1}${y+1}',EnclosureMount,"enclosure frame dry-side mounting pad",80,80,20,
				F.fromRotationMatrix(x*(HobberXLayout.front().bedLength/2-100),y*380,720,[1.0,0,0,0,0,y,0,-y,0]),
				[y < 0 ? "body.y-min" : "body.y-max"],InsertHoles.fromNeutral([])));
		return out;
	}

	public static function trayLedges(depth:Float = machinekit.enclosure.TraySeat.DEPTH):Array<CastingFeature> {
		var d = dimensions("bed"), w = d[0], l = d[1], a = machinekit.enclosure.TraySeat.WIDTH;
		return [
			Patterns.box("tray-front-ledge", InterfaceRecess, faces(["body.x-min", "body.y-min", "body.y-max", "body.z-max"]),
				a, l, depth, -w / 2 + a / 2, 0, d[2] - depth, "tray"),
			// The removable bridge owns the middle span. The two ledges stop
			// at its edges rather than claiming the same cavity twice.
			Patterns.box("tray-rear-ledge-left", InterfaceRecess, faces(["body.x-max", "body.y-min", "body.z-max"]),
				a, (l - TRAY_REAR_BRIDGE_WIDTH) / 2, depth, w / 2 - a / 2,
				-(l + TRAY_REAR_BRIDGE_WIDTH) / 4, d[2] - depth, "tray"),
			Patterns.box("tray-rear-ledge-right", InterfaceRecess, faces(["body.x-max", "body.y-max", "body.z-max"]),
				a, (l - TRAY_REAR_BRIDGE_WIDTH) / 2, depth, w / 2 - a / 2,
				(l + TRAY_REAR_BRIDGE_WIDTH) / 4, d[2] - depth, "tray"),
			Patterns.box("tray-left-ledge", InterfaceRecess, faces(["body.y-min", "body.z-max"]),
				w - 2 * a, a, depth, 0, -l / 2 + a / 2, d[2] - depth, "tray"),
			Patterns.box("tray-right-ledge", InterfaceRecess, faces(["body.y-max", "body.z-max"]),
				w - 2 * a, a, depth, 0, l / 2 - a / 2, d[2] - depth, "tray")
		];
	}

	/** Unverified exterior study choices; published bore/thread dimensions live in the catalogue variant. */
	public static inline final TRAY_REAR_BRIDGE_WIDTH = 150.0;
	public static inline final TRAY_SOCKET_OUTSIDE_DIAMETER = 8.0;
	public static inline final TRAY_SOCKET_RELIEF_DIAMETER = 6.2;
	public static inline final TRAY_SOCKET_TAIL_WIDTH = 11.0;
	public static inline final TRAY_SOCKET_TAIL_THICKNESS = 2.2;
	public static inline final TRAY_SOCKET_TAIL_WAVE = 1.4;
	public static inline final TRAY_SOCKET_TAIL_BLEND = 0.5;
	public static inline final TRAY_SOCKET_POSITION_TOLERANCE = 0.5;

	public static function traySocket():CastInsert
		return machinekit.structural.FixingSockets.pfeiferStudy({outsideDiameter:TRAY_SOCKET_OUTSIDE_DIAMETER,
			reliefDiameter:TRAY_SOCKET_RELIEF_DIAMETER,tailWidth:TRAY_SOCKET_TAIL_WIDTH,
			tailThickness:TRAY_SOCKET_TAIL_THICKNESS,tailWave:TRAY_SOCKET_TAIL_WAVE,tailBlend:TRAY_SOCKET_TAIL_BLEND},
			TRAY_SOCKET_POSITION_TOLERANCE);

	public static function trayClampSeats():Array<InsertLocation> {
		var out:Array<InsertLocation> = [], d = dimensions("bed");
		function add(x:Float,y:Float,height:Float = 11):Void
			out.push({id:"trayClampSeat"+out.length,part:traySocket(),openings:faces(["body.z-max"]),
				pose:F.translation(x,y,d[2]-machinekit.structural.FixingSockets.LENGTH),clampHeight:height});
		for (side in [-1,1]) for (x in [-1,0,1]) add(x*(d[0]/2-100),side*(d[1]/2-35));
		for (side in [-1,1]) for (y in [-150.0,150.0]) add(side*(d[0]/2-35),y);
		for (y in [-75.0,75.0]) add(d[0]/2-35,y,17);
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

	static inline final COLUMN_WALL = 20.0;
	static inline final COLUMN_RIB = 20.0;
	static inline final COLUMN_SPINE_HALF_WIDTH = 60.0;
	static inline final COLUMN_HEAD_WIDTH = 240.0;
	static inline final COLUMN_HEAD_DEPTH = 100.0;
	static inline final COLUMN_HEAD_CENTRE_Y = 60.0;

	/** Four authored chambers. The front pair ends at the head recess, so
	 * neither the spine nor the recess is recovered by subtracting supports. */
	static function columnPockets(width:Float, depth:Float, height:Float):Array<CastingFeature> {
		var out:Array<CastingFeature> = [];
		for (side in [-1, 1]) {
			var outer = width / 2 - COLUMN_WALL;
			var rearInner = COLUMN_SPINE_HALF_WIDTH, frontInner = COLUMN_HEAD_WIDTH / 2;
			var rearLow = -depth / 2 + COLUMN_WALL, rearHigh = -COLUMN_RIB / 2;
			var frontLow = COLUMN_RIB / 2, frontHigh = depth / 2 - COLUMN_WALL;
			var suffix = side < 0 ? "left" : "right";
			out.push(Patterns.box("rear-chamber-" + suffix, Lightening, [], outer - rearInner,
				rearHigh - rearLow, height - 2 * COLUMN_WALL, side * (outer + rearInner) / 2,
				(rearLow + rearHigh) / 2, COLUMN_WALL));
			out.push(Patterns.box("head-side-chamber-" + suffix, Lightening, [C.feature("head-recess-" + suffix)],
				outer - frontInner, frontHigh - frontLow, height - 2 * COLUMN_WALL,
				side * (outer + frontInner) / 2, (frontLow + frontHigh) / 2, COLUMN_WALL));
		}
		return out;
	}

	public static function bodyValues(kind:String):ComponentValues {
		var d = dimensions(kind);
		var cores = kind == "bed" ? HobberCastingPockets.plinth(d[0], d[1], d[2])
			: columnPockets(d[0], d[1], d[2]);
		if (kind == "bed") {
			var layout = HobberXLayout.front(), centre = layout.bedCentre;
			var footCentres = feet();
			for (i in 0...footCentres.length) {
				var p = footCentres[i], alongX = i == 2;
				var opening = alongX ? "body.x-max" : p.y < 0 ? "body.y-min" : "body.y-max";
				var edge = alongX ? d[0] / 2 : p.y < 0 ? -d[1] / 2 : d[1] / 2;
				cores = cores.concat(Patterns.foot({access: 'foot$i-access', stem: 'foot$i-stem', register: 'foot$i-plate-register'},
					p.x, p.y, edge, alongX, FOOT_ACCESS_WIDTH, FOOT_ACCESS_WIDTH, FOOT_FLOOR, FOOT_ROOF,
					FOOT_STEM_CLEARANCE, FOOT_SUPPORT_TOP, faces([opening]),
					faces(["body.z-min"]).concat([C.feature(i == 2 ? "pocket-motorRibRear-leftWebInside-footRoof"
						: i == 0 ? "pocket-frontSocketStart-leftEnclosureInside-footRoof"
						: "pocket-frontSocketStart-rightEnclosureInside-footRoof")]), {
						owner: "footSeat" + i, openings: [C.feature('foot$i-access')],
						width: FOOT_PLATE_WIDTH + FOOT_REGISTER_CLEARANCE,
						depth: FOOT_PLATE_WIDTH + FOOT_REGISTER_CLEARANCE,
						thickness: FOOT_PLATE_THICKNESS
					}));
			}
			cores.push(Patterns.box("work-well", ServiceOpening, faces(["body.z-max"]),
				WORK_WELL_WIDTH, WORK_WELL_DEPTH, WORK_WELL_TOP - TABLE_WELL_FLOOR_HEIGHT, WORK_WELL_CENTRE_X - centre, 0, TABLE_WELL_FLOOR_HEIGHT));
			// This drain starts at the front chamber: the part of the former
			// rectangular cut already inside that chamber is not another feature.
			var drainFront = -140.0 - centre, drainRear = 90.0 - centre;
			cores.push(Patterns.box("chip-drain", ServiceOpening,
				[C.feature("work-well"), C.feature("pocket-frontSocketEnd-leftRailInside-floor")],
				drainRear - drainFront, CHIP_DRAIN_WIDTH, CHIP_DRAIN_DEPTH, (drainRear + drainFront) / 2, 0, TABLE_WELL_FLOOR_HEIGHT - CHIP_DRAIN_DEPTH));
			var channelFront = layout.deckStart - DRIVE_CHANNEL_FRONT_CLEARANCE;
			cores.push(Patterns.box("X-drive-channel", ServiceOpening, faces(["body.x-max", "body.z-max"]),
				layout.bedRear - channelFront, DRIVE_CHANNEL_WIDTH, 90, (layout.bedRear + channelFront) / 2 - centre, 0, DRIVE_CHANNEL_FLOOR));
			// The coolant reservoir and downcomer occupy the front chamber;
			// only the wall opening removes additional casting material.
			var accessFront = -305.0 - centre, accessRear = -d[0] / 2 + 25;
			cores.push(Patterns.box("coolant-front-access", ServiceOpening,
				faces(["body.x-min"]).concat([C.feature("pocket-frontWall-leftWebOutside-floor")]),
				accessRear - accessFront, 240, 320, (accessFront + accessRear) / 2, 0, 140));
			cores.push(Patterns.box("rotary-union-access", Clearance, faces(["body.z-min"]).concat([C.feature("chip-drain")]),
				40, 40, TABLE_WELL_FLOOR_HEIGHT - CHIP_DRAIN_DEPTH, -centre, 0, 0));
			cores = cores.concat(trayLedges());
			for (side in [-1, 1])
				cores.push(Patterns.box("tray-rear-bridge-seat-" + (side < 0 ? "left" : "right"), InterfaceRecess,
					faces(["body.x-max", "body.z-max"]).concat([C.feature("X-drive-channel")]),
					25, (TRAY_REAR_BRIDGE_WIDTH - DRIVE_CHANNEL_WIDTH) / 2, 10, d[0] / 2 - 12.5,
					side * (TRAY_REAR_BRIDGE_WIDTH + DRIVE_CHANNEL_WIDTH) / 4, d[2] - 10, "tray-rear-bridge"));
			// Side chambers already provide the rest of this cable path.
			cores.push(Patterns.box("cable-route", ServiceOpening, [
				C.feature("pocket-centreSocketLeft-rightRailOutside-floor"),
				C.feature("pocket-centreWebRight-rightRailOutside-floor")],
				PLINTH_CORE_WEB, 30, 40, 0, 300, 500));
		} else {
			// The spine continues into the head opening. Three direct spans
			// describe its U-shaped recess without cutting it and retaining a boss.
			var headRear = COLUMN_HEAD_CENTRE_Y + COLUMN_HEAD_DEPTH / 2;
			var sideWidth = COLUMN_HEAD_WIDTH / 2 - COLUMN_SPINE_HALF_WIDTH;
			for (side in [-1, 1])
				cores.push(Patterns.box("head-recess-" + (side < 0 ? "left" : "right"), ServiceOpening,
					faces(["body.y-max", "body.z-min", "body.z-max"]), sideWidth, COLUMN_HEAD_DEPTH, d[2],
					side * (COLUMN_HEAD_WIDTH / 2 + COLUMN_SPINE_HALF_WIDTH) / 2, COLUMN_HEAD_CENTRE_Y, 0));
			cores.push(Patterns.box("head-recess-centre", ServiceOpening, faces(["body.y-max", "body.z-min", "body.z-max"]),
				2 * COLUMN_SPINE_HALF_WIDTH, headRear - HobberZLayout.SPINE_FRONT_COLUMN_Y, d[2],
				0, (headRear + HobberZLayout.SPINE_FRONT_COLUMN_Y) / 2, 0));
		}
		var struts = precastStruts(kind);
		for (feature in cores) if (feature.kind != Lightening) for (strut in struts)
			if (BoxAlgebra.volume(BoxAlgebra.subtract([strut.box],[materia.casting.CastingBounds.feature(feature)]))
				< BoxAlgebra.volume([strut.box])-1e-7)
				throw "Precast strut "+strut.id+" intersects source opening "+feature.id;
		return new ComponentValues().setNumber("width", d[0]).setNumber("depth", d[1]).setNumber("height", d[2])
			.setToken("features", C.encode(cores)).setToken("precastStruts",materia.casting.PrecastStrut.PrecastStruts.encode(struts));
	}

	public static function placements(kind:String):Array<CastInsertPlacement>
		return [for (row in inserts(kind)) {id:row.id,part:row.part,pose:row.pose,openings:row.openings}];

	public static function inserts(kind:String):Array<InsertLocation> {
		dimensions(kind);
		var out:Array<InsertLocation> = [];
		function add(id:String,role:InsertRole,label:String,w:Float,d:Float,h:Float,x:Float,y:Float,z:Float,
				openings:Array<String>,holes:String):Void
			out.push(boxInsert(id,role,label,w,d,h,F.translation(x,y,z),openings,holes));
		var none = InsertHoles.fromNeutral([]);
		if (kind == "bed") {
			var layout = HobberXLayout.front(), centre = layout.bedCentre;
			out = enclosureSeats().concat(trayClampSeats());
			add("tablePlate",TableMount,"table seat",280,280,20,-centre,0,TABLE_SEAT_HEIGHT-20,[],InsertHoles.through(40,20));
			for (side in [-1,1]) out.push(boxInsert(side < 0 ? "railLeft" : "railRight",GuideSeat,"preloaded column guide seat",
				BED_RAIL_SEAT_WIDTH,layout.railLength,20,
				F.fromRotationMatrix(layout.railStart+layout.railLength/2-centre,side*BED_RAIL_CENTRE,RAIL_SEAT_HEIGHT,
					[0.0,1,0,-1,0,0,0,0,1]),["body.z-max"],none));
			out.push(boxInsert("driveSeat",DriveMount,"central X screw channel seat",100,layout.motorFace-layout.deckStart,20,
				F.fromRotationMatrix((layout.motorFace+layout.deckStart)/2-centre,0,DRIVE_SEAT_HEIGHT,[0.0,1,0,-1,0,0,0,0,1]),[],none));
			for (side in [-1,1]) add(side < 0 ? "liftLeft" : "liftRight",LiftingMount,"lifting point",30,30,30,
				-150-centre,side*310,LIFT_SEAT_HEIGHT,["body.z-max"],none);
		} else {
			add("basePlate",ColumnMount,"column carriage sole",380,100,20,0,-40,-10,["body.z-min"],none);
			for (side in [-1,1]) out.push(boxInsert(side < 0 ? "railLeft" : "railRight",GuideSeat,"Z rail seat",35,480,20,
				F.fromRotationMatrix(side*145,90,245,[1.0,0,0,0,0,1,0,-1,0]),["body.y-max"],none));
			out.push(boxInsert("driveSeat",DriveMount,"central Z drive back seat",100,460,20,
				F.fromRotationMatrix(0,HobberZLayout.SEAT_COLUMN_Y,250,[-1.0,0,0,0,0,1,0,1,0]),[],none));
			for (side in [-1,1]) add(side < 0 ? "liftLeft" : "liftRight",LiftingMount,"lifting point",25,25,20,
				side*160,-40,490,["body.z-max"],none);
		}
		return out;
	}
}
