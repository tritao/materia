package cadkit.drawing;

import cadkit.sheetmetal.FlatPattern;
import cadkit.sheetmetal.Point2;
import cadkit.sheetmetal.Segment;
import cadkit.sheetmetal.SheetBounds;
import cadkit.modeling.Vector;

/** Format-independent exact drawing data; exporting never changes the sheet. */
class Drawing2D {
	final entityData:Array<DrawingEntity>;
	public function new(entities:Array<DrawingEntity>) { entityData = entities.copy(); }
	public function entities():Array<DrawingEntity> { return entityData.copy(); }
	/** Geometry and explicitly sized annotation extents, with no native query. */
	public function bounds():SheetBounds {
		var result = new SheetBounds();
		for (entity in entityData) switch (entity) {
			case Curve(_, segment): for (point in segment.extrema()) result.include(new Vector(point.x, point.y));
			case Label(_, position, height, value):
				if (!Math.isFinite(height) || height <= 0) throw "Drawing text height must be positive millimetres";
				result.include(new Vector(position.x, position.y - height * 0.25));
				result.include(new Vector(position.x + textWidth(height, value), position.y + height));
		}
		return result;
	}
	public static function textWidth(height:Float, value:String):Float { return height * 0.6 * value.length; }
	public static function layerName(layer:DrawingLayer):String {
		return switch (layer) {
			case Outline: "OUTLINE";
			case BendUp: "BEND_UP";
			case BendDown: "BEND_DOWN";
			case Etch: "ETCH";
			case Text: "TEXT";
			case Process(name):
				if (name.length == 0 || name.length > 64) throw "Invalid drawing process layer";
				for (i in 0...name.length)
					if ("ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789_-".indexOf(name.charAt(i)) < 0)
						throw "Invalid drawing process layer";
				name;
		};
	}
	public static function layers():Array<DrawingLayer> { return [Outline, BendUp, BendDown, Etch, Text]; }
	/** Existing sheet layers plus named layers actually used by this drawing. */
	public function usedLayers():Array<DrawingLayer> {
		var result = layers(), names = [for (layer in result) layerName(layer)];
		for (entity in entityData) {
			var layer = switch entity { case Curve(owner, _): owner; case Label(owner, _, _, _): owner; };
			var name = layerName(layer);
			if (names.indexOf(name) < 0) { names.push(name); result.push(layer); }
		}
		return result;
	}
	public static function of(flat:FlatPattern):Drawing2D {
		var entities:Array<DrawingEntity> = [];
		for (region in flat.regions()) for (loop in region.loops()) for (segment in loop)
			entities.push(Curve(Outline, segment));
		for (bend in flat.bends()) {
			var layer = bend.up() ? DrawingLayer.BendUp : DrawingLayer.BendDown;
			for (line in bend.lines()) entities.push(Curve(layer, line));
			var lines = bend.lines();
			if (lines.length > 0) entities.push(Label(Text, lines[0].point(0.5), 2.5,
				"B" + bend.id + " " + (bend.up() ? "UP" : "DOWN") + " " + Math.abs(bend.angle) * 180 / Math.PI + " deg"));
		}
		if (flat.material.grainAngle != null) {
			var bounds = flat.bounds(); var length = Math.min(20, (bounds.maxX - bounds.minX) / 3);
			var x = bounds.minX + length / 2; var y = bounds.minY + length / 2; var tip = new Point2(x + length, y);
			entities.push(Curve(Etch, Segment.line(new Point2(x, y), tip)));
			entities.push(Curve(Etch, Segment.line(tip, new Point2(tip.x - length / 4, tip.y + length / 8))));
			entities.push(Curve(Etch, Segment.line(tip, new Point2(tip.x - length / 4, tip.y - length / 8))));
			entities.push(Label(Text, new Point2(x, y + length / 4), 2.5, "GRAIN"));
		}
		return new Drawing2D(entities);
	}
}
