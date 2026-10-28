package machinekit.hmi;

import cadkit.modeling.Footprint;
import cadkit.modeling.FootprintGeometry;
/** A mark to draw over the layout: a round or rectangular feature at a position, millimetres from the panel centre. */
typedef SvgMark = {
  var x:Float;
  var y:Float;
  /** Diameter for a round mark; width for a rectangle. */
  var size:Float;
  /** Height of a rectangle, 0 for a round mark. */
  var height:Float;
  var filled:Bool;
}

/**
 * A top-down drawing of a panel for review: the plate, the usable area inside the wall and edge margin,
 * each part's rear body (grey), keep-out (dashed), cutout (solid) and label. Parts whose numbers are still
 * assumed are drawn in orange. SVG y points down, so the panel is flipped. `marks` are drawn in green on top.
 */
class PanelLayoutSvg {
  public static function render(layout:PanelLayout, scale:Float = 4.0, ?marks:Array<SvgMark>):String {
    var w = layout.width * scale, h = layout.height * scale;
    function px(x:Float):String return Std.string(Math.round((x + layout.width / 2) * scale * 10) / 10);
    function py(y:Float):String return Std.string(Math.round((layout.height / 2 - y) * scale * 10) / 10);
    function len(v:Float):String return Std.string(Math.round(v * scale * 10) / 10);
    var out = ['<svg xmlns="http://www.w3.org/2000/svg" width="$w" height="$h" viewBox="0 0 $w $h" font-family="sans-serif" font-size="10">',
      '<rect width="$w" height="$h" fill="#f4f4f4" stroke="#222"/>',
      '<rect x="${px(-layout.usableHalfWidth())}" y="${py(layout.usableHalfHeight())}" width="${len(layout.usableHalfWidth() * 2)}" height="${len(layout.usableHalfHeight() * 2)}" fill="none" stroke="#888" stroke-dasharray="2 2"/>'];
    for (c in layout.controls) switch c.body() {
      case Circle(d): out.push('<circle cx="${px(c.x)}" cy="${py(c.y)}" r="${len(d / 2)}" fill="#e2e2e2" stroke="#bbb"/>');
      case Rect(rw, rh): out.push('<rect x="${px(c.x - rw / 2)}" y="${py(c.y + rh / 2)}" width="${len(rw)}" height="${len(rh)}" fill="#e2e2e2" stroke="#bbb"/>');
      case _:
    }
    for (c in layout.controls) {
      var colour = c.source() == "assumed" ? "#c60" : "#06a";
      switch c.keepOut() {
        case Circle(d): out.push('<circle cx="${px(c.x)}" cy="${py(c.y)}" r="${len(d / 2)}" fill="none" stroke="$colour" stroke-dasharray="4 3"/>');
        case Rect(rw, rh): out.push('<rect x="${px(c.x - rw / 2)}" y="${py(c.y + rh / 2)}" width="${len(rw)}" height="${len(rh)}" fill="none" stroke="$colour" stroke-dasharray="4 3"/>');
        case _:
      }
      switch c.cutout() {
        case Circle(d): out.push('<circle cx="${px(c.x)}" cy="${py(c.y)}" r="${len(d / 2)}" fill="#fff" stroke="$colour"/>');
        case Rect(rw, rh): out.push('<rect x="${px(c.x - rw / 2)}" y="${py(c.y + rh / 2)}" width="${len(rw)}" height="${len(rh)}" fill="#fff" stroke="$colour"/>');
        case _:
      }
      for (hole in c.holes())
        out.push('<circle cx="${px(c.x + hole.x)}" cy="${py(c.y + hole.y)}" r="${len(hole.diameter / 2)}" fill="#fff" stroke="$colour"/>');
      out.push('<text x="${px(c.x)}" y="${py(c.y)}" text-anchor="middle" dominant-baseline="middle" fill="$colour">${c.id}</text>');
    }
    if (marks != null) for (m in marks) {
      var fill = m.filled ? "#090" : "#fff";
      if (m.height > 0) out.push('<rect x="${px(m.x - m.size / 2)}" y="${py(m.y + m.height / 2)}" width="${len(m.size)}" height="${len(m.height)}" fill="$fill" stroke="#090"/>');
      else out.push('<circle cx="${px(m.x)}" cy="${py(m.y)}" r="${len(m.size / 2)}" fill="$fill" stroke="#090"/>');
    }
    out.push('</svg>');
    return out.join("\n") + "\n";
  }
}
