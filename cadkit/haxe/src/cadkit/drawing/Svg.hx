package cadkit.drawing;

import cadkit.sheetmetal.Segment;

/** Millimetre SVG drawing with the same manufacturing layers and exact circular curves as DXF. */
class Svg {
	static function escape(value:String):String {
		return StringTools.replace(StringTools.replace(StringTools.replace(StringTools.replace(StringTools.replace(value,
			"&", "&amp;"), "<", "&lt;"), ">", "&gt;"), "\"", "&quot;"), "'", "&apos;");
	}
	static function curve(segment:Segment):String {
		if (segment.isLine()) return '<line x1="' + segment.start.x + '" y1="' + -segment.start.y
			+ '" x2="' + segment.end.x + '" y2="' + -segment.end.y + '"/>';
		if (Math.abs(Math.abs(segment.sweep) - 2 * Math.PI) < 1e-10) {
			var center = segment.arcCenter();
			return '<circle cx="' + center.x + '" cy="' + -center.y + '" r="' + segment.radius + '"/>';
		}
		return '<path d="M ' + segment.start.x + ' ' + -segment.start.y + ' A ' + segment.radius + ' ' + segment.radius
			+ ' 0 ' + (Math.abs(segment.sweep) > Math.PI ? '1' : '0') + ' ' + (segment.sweep > 0 ? '0' : '1')
			+ ' ' + segment.end.x + ' ' + -segment.end.y + '"/>';
	}
	public static function write(drawing:Drawing2D):String {
		var bounds = drawing.bounds(); if (!Math.isFinite(bounds.minX)) throw "Cannot export an empty SVG drawing";
		var margin = 2.5; var width = bounds.maxX - bounds.minX + 2 * margin; var height = bounds.maxY - bounds.minY + 2 * margin;
		var output = new StringBuf();
		output.add('<svg xmlns="http://www.w3.org/2000/svg" width="' + width + 'mm" height="' + height + 'mm" viewBox="'
			+ (bounds.minX - margin) + ' ' + (-bounds.maxY - margin) + ' ' + width + ' ' + height + '">\n');
		for (layer in Drawing2D.layers()) {
			var color = switch (layer) { case BendUp: "#2455ba"; case BendDown: "#b63434"; case Etch: "#777777"; case _: "#111111"; };
			output.add('<g id="' + Drawing2D.layerName(layer) + '" fill="none" stroke="' + color + '" stroke-width="0.2">\n');
			for (entity in drawing.entities()) switch (entity) {
				case Curve(owner, segment): if (owner == layer) { output.add(curve(segment)); output.add("\n"); }
				case Label(owner, position, textHeight, value): if (owner == layer) {
					output.add('<text x="' + position.x + '" y="' + -position.y + '" font-family="monospace" font-size="'
						+ textHeight + '" fill="' + color + '" stroke="none"');
					if (value.length > 0) output.add(' textLength="' + Drawing2D.textWidth(textHeight, value) + '" lengthAdjust="spacingAndGlyphs"');
					output.add('>' + escape(value) + '</text>\n');
				}
			}
			output.add('</g>\n');
		}
		output.add('</svg>\n'); return output.toString();
	}
}
