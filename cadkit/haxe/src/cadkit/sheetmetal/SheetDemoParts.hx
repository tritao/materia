package cadkit.sheetmetal;

import materia.project.MaterialLibrary;

/** Dimensioned acceptance parts shared by sheet, tooling and planning tests. All dimensions are mm. */
class SheetDemoParts {
	/** Authored reference-part radius; process-driven parts derive theirs from selected tooling. */
	public static inline var RADIUS:Float = 1;
	public static function materialA():SheetMaterial {
		var grade = MaterialLibrary.require("mild-steel");
		return new SheetMaterial(grade.id, 1.5, BendRule.KFactor(new KFactorTable([0], [0.38])),
			grade.mechanical, RADIUS / 1.5);
	}

	/** 100 along the bend, 60 outside base, 40 outside height. */
	public static function p1():Sheet {
		var sheet = new Sheet(materialA());
		var setback = BendDimensions.outsideSetback(sheet.thickness(), RADIUS, Math.PI / 2);
		var base = sheet.addWall(WallOutline.rectangle(60 - setback, 100));
		flange(sheet, base, 1, 40 - setback, Math.PI / 2);
		return sheet;
	}
	/** 120 long, 60 outside width, equal 40 outside flanges. */
	public static function p2():Sheet {
		return channel(false);
	}
	/** 120 long, 60 outside web, 40 outside legs in opposite directions (78.5 overall height). */
	public static function p3():Sheet {
		return channel(true);
	}
	static function channel(opposite:Bool):Sheet {
		var sheet = new Sheet(materialA());
		var setback = BendDimensions.outsideSetback(sheet.thickness(), RADIUS, Math.PI / 2);
		var base = sheet.addWall(WallOutline.rectangle(60 - 2 * setback, 120));
		flange(sheet, base, 1, 40 - setback, Math.PI / 2);
		flange(sheet, base, 3, 40 - setback, opposite ? -Math.PI / 2 : Math.PI / 2);
		return sheet;
	}
	/** Hat: 120 long, 60 outside crown, 40 height and 20 outside lips; overall width 97. */
	public static function p4():Sheet {
		var sheet = new Sheet(materialA());
		var setback = BendDimensions.outsideSetback(sheet.thickness(), RADIUS, Math.PI / 2);
		var base = sheet.addWall(WallOutline.rectangle(60 - 2 * setback, 120));
		for (edge in [1, 3]) {
			var side = flange(sheet, base, edge, 40 - 2 * setback, Math.PI / 2);
			var lip = flange(sheet, side, 1, 20 - setback, -Math.PI / 2);
		}
		return sheet;
	}
	/** Open-corner tray: 150 by 100 by 25 outside, with 2 mm square corner reliefs. */
	public static function p5():Sheet {
		var sheet = new Sheet(materialA());
		var setback = BendDimensions.outsideSetback(sheet.thickness(), RADIUS, Math.PI / 2);
		var w = 150 - 2 * setback;
		var h = 100 - 2 * setback;
		var g = 2.0;
		var base = sheet.addWall(WallOutline.polygon([
			new Point2(g, 0), new Point2(w - g, 0), new Point2(w - g, g), new Point2(w, g),
			new Point2(w, h - g), new Point2(w - g, h - g), new Point2(w - g, h), new Point2(g, h),
			new Point2(g, h - g), new Point2(0, h - g), new Point2(0, g), new Point2(g, g)
		]));
		for (edge in [0, 3, 6, 9]) { var side = flange(sheet, base, edge, 25 - setback, Math.PI / 2); }
		return sheet;
	}
	/** Add an edge-to-edge rectangular wall; dimensions are straight leg and bend radius. */
	public static function flange(sheet:Sheet, parent:Wall, edge:Int, leg:Float, angle:Float, radius:Float = RADIUS):Wall {
		var width = parent.outline.segment(0, edge).length();
		var child = sheet.addWall(WallOutline.rectangle(leg, width));
		var bend = sheet.addBend(parent.id, new WallEdge(edge), child.id, new WallEdge(3), angle, radius);
		return child;
	}
}
