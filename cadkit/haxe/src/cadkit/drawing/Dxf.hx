package cadkit.drawing;

import cadkit.sheetmetal.Segment;
import haxe.io.Bytes;

/** ASCII R12 drawing writer. Curves remain exact; clockwise arcs use the negative-Z OCS. */
class Dxf {
	static function pair(output:StringBuf, code:Int, value:String):Void {
		output.add(code); output.add("\n"); output.add(value); output.add("\n");
	}
	static function number(output:StringBuf, code:Int, value:Float):Void {
		if (!Math.isFinite(value)) throw "DXF coordinate must be finite";
		pair(output, code, Std.string(value == 0 ? 0.0 : value));
	}
	static function degrees(angle:Float):Float {
		var result = angle * 180 / Math.PI % 360;
		return result < 0 ? result + 360 : result;
	}
	static function curve(output:StringBuf, layer:DrawingLayer, segment:Segment):Void {
		if (segment.isLine()) {
			pair(output, 0, "LINE"); pair(output, 8, Drawing2D.layerName(layer));
			number(output, 10, segment.start.x); number(output, 20, segment.start.y); number(output, 30, 0);
			number(output, 11, segment.end.x); number(output, 21, segment.end.y); number(output, 31, 0);
			return;
		}
		var circle = Math.abs(Math.abs(segment.sweep) - 2 * Math.PI) < 1e-10;
		var clockwise = segment.sweep < 0; var center = segment.arcCenter();
		pair(output, 0, circle ? "CIRCLE" : "ARC"); pair(output, 8, Drawing2D.layerName(layer));
		// Autodesk's arbitrary-axis rule gives OCS X=-world X, Y=world Y for normal -Z.
		number(output, 10, clockwise ? -center.x : center.x); number(output, 20, center.y); number(output, 30, 0);
		number(output, 40, segment.radius);
		if (!circle) {
			var start = clockwise ? Math.PI - segment.startAngle : segment.startAngle;
			var end = clockwise ? Math.PI - segment.startAngle - segment.sweep : segment.startAngle + segment.sweep;
			number(output, 50, degrees(start)); number(output, 51, degrees(end));
		}
		if (clockwise) { number(output, 210, 0); number(output, 220, 0); number(output, 230, -1); }
	}
	public static function write(drawing:Drawing2D):String {
		var output = new StringBuf();
		pair(output, 999, "Materia exact drawing; coordinates in millimetres");
		pair(output, 0, "SECTION"); pair(output, 2, "HEADER"); pair(output, 9, "$ACADVER"); pair(output, 1, "AC1009"); pair(output, 0, "ENDSEC");
		pair(output, 0, "SECTION"); pair(output, 2, "TABLES"); pair(output, 0, "TABLE"); pair(output, 2, "LAYER"); pair(output, 70, Std.string(drawing.usedLayers().length));
		for (layer in drawing.usedLayers()) {
			pair(output, 0, "LAYER"); pair(output, 2, Drawing2D.layerName(layer)); pair(output, 70, "0"); pair(output, 62, "7"); pair(output, 6, "CONTINUOUS");
		}
		pair(output, 0, "ENDTAB"); pair(output, 0, "ENDSEC"); pair(output, 0, "SECTION"); pair(output, 2, "ENTITIES");
		for (entity in drawing.entities()) switch (entity) {
			case Curve(layer, segment): curve(output, layer, segment);
			case Label(layer, position, height, value):
				if (!Math.isFinite(height) || height <= 0) throw "DXF text height must be positive millimetres";
				var bytes = Bytes.ofString(value);
				for (i in 0...bytes.length) if (bytes.get(i) < 32 || bytes.get(i) > 126) throw "R12 ASCII text must contain printable ASCII characters";
				pair(output, 0, "TEXT"); pair(output, 8, Drawing2D.layerName(layer));
				number(output, 10, position.x); number(output, 20, position.y); number(output, 30, 0);
				number(output, 40, height); pair(output, 1, value);
		}
		pair(output, 0, "ENDSEC"); pair(output, 0, "EOF"); return output.toString();
	}
}
