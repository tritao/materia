package materia.drawing;

/** Physical-size SVG export for an already projected engineering drawing view. */
class DrawingSvgExporter {
	public static function svg(drawing:ProjectedDrawingView, widthMm:Float = 210, heightMm:Float = 148):String {
		if (drawing == null || drawing.bounds == null || drawing.lines == null || drawing.lines.length == 0)
			throw "Drawing SVG export needs a projected view with linework";
		if (!Math.isFinite(widthMm) || !Math.isFinite(heightMm) || widthMm <= 0 || heightMm <= 0)
			throw "Drawing SVG page dimensions must be positive and finite";
		var margin = 10.0, titleHeight = 8.0;
		var availableWidth = widthMm - margin * 2, availableHeight = heightMm - margin * 2 - titleHeight;
		if (availableWidth <= 0 || availableHeight <= 0) throw "Drawing SVG page is too small for its margins";
		var bounds = drawing.bounds;
		var spanX = Math.max(1e-6, bounds.maxX - bounds.minX), spanY = Math.max(1e-6, bounds.maxY - bounds.minY);
		var scale = Math.min(availableWidth / spanX, availableHeight / spanY);
		var drawnWidth = spanX * scale, drawnHeight = spanY * scale;
		var originX = margin + (availableWidth - drawnWidth) / 2;
		var originY = margin + titleHeight + (availableHeight - drawnHeight) / 2;
		var out = new StringBuf();
		out.add('<?xml version="1.0" encoding="UTF-8"?>\n');
		out.add('<svg xmlns="http://www.w3.org/2000/svg" width="${fmt(widthMm)}mm" height="${fmt(heightMm)}mm" viewBox="0 0 ${fmt(widthMm)} ${fmt(heightMm)}">\n');
		out.add('<rect width="100%" height="100%" fill="#fff"/>\n');
		out.add('<text x="${fmt(margin)}" y="${fmt(margin + 4)}" font-family="sans-serif" font-size="3.5" font-weight="bold">${xml(drawing.partId)} · ${xml(drawing.viewId)}</text>\n');
		for (line in drawing.lines) {
			var x1 = originX + (line.x1 - bounds.minX) * scale;
			var y1 = originY + drawnHeight - (line.y1 - bounds.minY) * scale;
			var x2 = originX + (line.x2 - bounds.minX) * scale;
			var y2 = originY + drawnHeight - (line.y2 - bounds.minY) * scale;
			var width = switch line.style {
				case "section": 0.55;
				case "section-hatch": 0.12;
				case "thread-major": 0.35;
				case "thread-minor": 0.18;
				case "hidden": 0.18;
				default: 0.3;
			};
			var dash = line.style == "hidden" ? ' stroke-dasharray="1.5 1.0"' : "";
			var id = line.faceName == null ? "" : ' data-face-name="${xml(cast line.faceName)}"';
			out.add('<line x1="${fmt(x1)}" y1="${fmt(y1)}" x2="${fmt(x2)}" y2="${fmt(y2)}" stroke="#17202a" stroke-width="$width" stroke-linecap="round"$dash data-style="${xml(line.style)}"$id/>\n');
		}
		for (callout in drawing.callouts) {
			var x = originX + (callout.x - bounds.minX) * scale;
			var y = originY + drawnHeight - (callout.y - bounds.minY) * scale;
			var leaderX = originX + (callout.leaderX - bounds.minX) * scale;
			var leaderY = originY + drawnHeight - (callout.leaderY - bounds.minY) * scale;
			var elbowX = x - 2.0;
			out.add('<polyline points="${fmt(leaderX)},${fmt(leaderY)} ${fmt(elbowX)},${fmt(leaderY)} ${fmt(x - 0.8)},${fmt(y)}" fill="none" stroke="#59636e" stroke-width="0.18" data-style="callout-leader" data-face-name="${xml(callout.faceName)}"/>\n');
			out.add('<text x="${fmt(x)}" y="${fmt(y)}" font-family="sans-serif" font-size="2.6" fill="#17202a" data-face-name="${xml(callout.faceName)}">${xml(callout.text)}</text>\n');
		}
		var scaleLabel = Math.abs(scale - 1) < 0.0005 ? "1:1" : scale > 1 ? fmt(scale) + ":1" : "1:" + fmt(1 / scale);
		out.add('<text x="${fmt(margin)}" y="${fmt(heightMm - margin / 2)}" font-family="sans-serif" font-size="2.2" fill="#59636e">ISO 6410-1 conventional thread representation · ${xml(drawing.lengthUnit)} · Scale ${scaleLabel}</text>\n');
		out.add('</svg>\n');
		return out.toString();
	}

	static function xml(value:String):String {
		var escaped = StringTools.replace(StringTools.replace(StringTools.replace(value, "&", "&amp;"), "<", "&lt;"), ">", "&gt;");
		return StringTools.replace(escaped, '"', "&quot;");
	}
	static function fmt(value:Float):String return Std.string(Math.round(value * 1000) / 1000);
}
