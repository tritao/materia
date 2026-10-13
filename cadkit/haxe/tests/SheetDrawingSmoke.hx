import cadkit.drawing.Drawing2D;
import cadkit.drawing.DrawingEntity;
import cadkit.drawing.DrawingLayer;
import cadkit.drawing.Dxf;
import cadkit.drawing.Svg;
import cadkit.sheetmetal.FlatPattern;
import cadkit.sheetmetal.Point2;
import cadkit.sheetmetal.Segment;
import cadkit.sheetmetal.SheetDemoParts;
import cadkit.sheetmetal.Sheet;
import cadkit.sheetmetal.SheetMaterial;
import cadkit.sheetmetal.WallOutline;

/** Independent R12 entity decoding checks exact curves, winding, layers and shared export data. */
class SheetDrawingSmoke {
	static var checks:Int = 0;
	static function check(value:Bool, label:String):Void { checks++; if (!value) throw label; }
	static function near(a:Float, b:Float, label:String):Void { check(Math.abs(a - b) < 1e-8, label + ": " + a + " != " + b); }
	static function layer(name:String):DrawingLayer {
		return switch (name) { case "OUTLINE": Outline; case "BEND_UP": BendUp; case "BEND_DOWN": BendDown;
			case "ETCH": Etch; case "TEXT": Text; case _: throw "Unknown DXF layer " + name; };
	}
	static function value(fields:Map<Int, String>, code:Int):String {
		var found = fields.get(code); if (found == null) throw "Missing DXF group " + code; return found;
	}
	static function number(fields:Map<Int, String>, code:Int):Float {
		var result = Std.parseFloat(value(fields, code)); if (!Math.isFinite(result)) throw "Invalid DXF number"; return result;
	}
	static function point(fields:Map<Int, String>, x:Int, y:Int):Point2 { return new Point2(number(fields, x), number(fields, y)); }
	static function read(source:String):Drawing2D {
		var lines = source.split("\n"); var cursor = 0; var inEntities = false; var entities:Array<DrawingEntity> = [];
		while (cursor + 1 < lines.length) {
			var code = Std.parseInt(lines[cursor]); var text = lines[cursor + 1]; cursor += 2;
			if (code == 9 && text == "$ACADVER") { check(lines[cursor] == "1" && lines[cursor + 1] == "AC1009", "R12 header"); }
			if (code == 2 && text == "ENTITIES") { inEntities = true; continue; }
			if (!inEntities || code != 0) continue;
			if (text == "ENDSEC") { inEntities = false; continue; }
			var fields:Map<Int, String> = [];
			while (cursor + 1 < lines.length && lines[cursor] != "0") {
				var group = Std.parseInt(lines[cursor]); if (group == null) throw "Invalid DXF group";
				fields.set(group, lines[cursor + 1]); cursor += 2;
			}
			var owner = layer(value(fields, 8));
			switch (text) {
				case "LINE": entities.push(Curve(owner, Segment.line(point(fields, 10, 20), point(fields, 11, 21))));
				case "ARC", "CIRCLE":
					var negative = fields.get(230) == "-1"; var center = point(fields, 10, 20);
					if (negative) center = new Point2(-center.x, center.y);
					var start = 0.0; var sweep = 2 * Math.PI;
					if (text == "ARC") {
						start = number(fields, 50) * Math.PI / 180;
						var end = number(fields, 51) * Math.PI / 180; sweep = end - start; if (sweep <= 0) sweep += 2 * Math.PI;
						if (negative) start = Math.PI - start;
					}
					if (negative) sweep = -sweep;
					entities.push(Curve(owner, Segment.arc(center, number(fields, 40), start, sweep)));
				case "TEXT": entities.push(Label(owner, point(fields, 10, 20), number(fields, 40), value(fields, 1)));
				case _: throw "Unexpected non-R12 entity " + text;
			}
		}
		return new Drawing2D(entities);
	}
	static function compare(original:Drawing2D, decoded:Drawing2D):Void {
		var before = original.entities(); var after = decoded.entities(); near(after.length, before.length, "entity count");
		for (i in 0...before.length) switch (before[i]) {
			case Curve(owner, expected): switch (after[i]) {
				case Curve(actualOwner, actual):
					check(actualOwner == owner, "curve layer"); near(actual.length(), expected.length(), "exact curve length");
					if (!expected.isLine() && Math.abs(expected.sweep) > 2 * Math.PI - 1e-10) {
						near(actual.arcCenter().distance(expected.arcCenter()), 0, "circle center"); near(actual.sweep, expected.sweep, "circle winding");
					} else { near(actual.start.distance(expected.start), 0, "curve start"); near(actual.end.distance(expected.end), 0, "curve end"); near(actual.point(0.5).distance(expected.point(0.5)), 0, "arc midpoint"); }
				case _: throw "Curve became label";
			}
			case Label(owner, position, height, text): switch (after[i]) {
				case Label(actualOwner, actualPosition, actualHeight, actualText):
					check(owner == actualOwner && text == actualText, "text content and layer"); near(position.distance(actualPosition), 0, "text placement"); near(height, actualHeight, "text height");
				case _: throw "Label became curve";
			}
		}
	}
	public static function run():Int {
		checks = 0;
		for (sheet in [SheetDemoParts.p1(), SheetDemoParts.p2(), SheetDemoParts.p3(), SheetDemoParts.p4(), SheetDemoParts.p5()]) {
			var flat = FlatPattern.of(sheet); var drawing = Drawing2D.of(flat); compare(drawing, read(Dxf.write(drawing)));
			var svg = Svg.write(drawing); for (owner in Drawing2D.layers()) check(svg.indexOf('id="' + Drawing2D.layerName(owner) + '"') >= 0, "SVG manufacturing layer");
			check(svg.indexOf("mm\"") >= 0, "SVG dimensions in millimetres");
		}
		var curves:Array<DrawingEntity> = [];
		for (sweep in [Math.PI / 3, -Math.PI / 3, 4 * Math.PI / 3, -4 * Math.PI / 3, 2 * Math.PI, -2 * Math.PI])
			curves.push(Curve(Outline, Segment.arc(new Point2(17, -23), 4, 5.8, sweep)));
		var curved = new Drawing2D(curves); compare(curved, read(Dxf.write(curved)));
		var annotated = new Drawing2D([Label(Text, new Point2(0, 0), 3, "A<&\"'>")]);
		check(Svg.write(annotated).indexOf("A&lt;&amp;&quot;&apos;&gt;") >= 0, "SVG escapes authored text");
		compare(annotated, read(Dxf.write(annotated)));
		var stock = SheetDemoParts.materialA();
		var grained = new Sheet(new SheetMaterial(stock.id, stock.thickness, stock.bendRule, stock.mechanical,
			stock.defaultRadiusRatio, stock.minRadiusRatio, Math.PI / 4));
		var wall = grained.addWall(WallOutline.rectangle(40, 20).withHole([Segment.arc(new Point2(10, 10), 2, 0, -2 * Math.PI)]));
		var pattern = FlatPattern.of(grained); var drawing = Drawing2D.of(pattern); var decoded = read(Dxf.write(drawing)); compare(drawing, decoded);
		var boundary:Array<Segment> = []; var etches = 0;
		for (entity in decoded.entities()) switch (entity) {
			case Curve(owner, segment):
				if (owner == Outline) boundary.push(segment);
				if (owner == Etch) {
					if (etches == 0) check(segment.start.y == segment.end.y && segment.end.x > segment.start.x, "grain arrow follows exported X");
					etches++;
				}
			case _: {}
		}
		near(etches, 3, "authored grain arrow uses etch layer");
		var outer = boundary.slice(0, 4); var holes = [boundary.slice(4)]; var restored = new WallOutline(outer, holes);
		near(restored.area(), pattern.area(), "decoded outer and hole loops preserve exact material area");
		check(Svg.write(drawing).indexOf("<circle") >= 0, "SVG preserves a circular hole");
		var refused = false;
		try { var invalid = Dxf.write(new Drawing2D([Label(Text, new Point2(0, 0), 3, "line\nbreak")])); }
		catch (error:Dynamic) { refused = Std.string(error).indexOf("ASCII") >= 0; }
		check(refused, "DXF refuses text that would break its group records");
		return checks;
	}
	static function main():Void { trace("Sheet drawing smoke passed (" + run() + " assertions)"); }
}
