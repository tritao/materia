package machinekit.pendant;

/**
 * A top-down drawing of the face for review: the plate, the usable area inside the wall and edge
 * margin, each control's keep-out (dashed), cutout (solid) and label. Parts whose numbers are
 * still assumed are drawn in orange. SVG y points down, so the face is flipped.
 */
class PendantFaceSvg {
  public static function render(face:PendantFace, scale:Float = 4.0):String {
    var w = face.width * scale, h = face.height * scale;
    function px(x:Float):String return Std.string(Math.round((x + face.width / 2) * scale * 10) / 10);
    function py(y:Float):String return Std.string(Math.round((face.height / 2 - y) * scale * 10) / 10);
    function len(v:Float):String return Std.string(Math.round(v * scale * 10) / 10);
    var out = ['<svg xmlns="http://www.w3.org/2000/svg" width="$w" height="$h" viewBox="0 0 $w $h" font-family="sans-serif" font-size="10">',
      '<rect width="$w" height="$h" fill="#f4f4f4" stroke="#222"/>',
      '<rect x="${px(-face.usableHalfWidth())}" y="${py(face.usableHalfHeight())}" width="${len(face.usableHalfWidth() * 2)}" height="${len(face.usableHalfHeight() * 2)}" fill="none" stroke="#888" stroke-dasharray="2 2"/>'];
    for (c in face.controls) {
      var colour = c.source == "assumed" ? "#c60" : "#06a";
      switch c.keepOut {
        case Circle(d): out.push('<circle cx="${px(c.x)}" cy="${py(c.y)}" r="${len(d / 2)}" fill="none" stroke="$colour" stroke-dasharray="4 3"/>');
        case Rect(rw, rh): out.push('<rect x="${px(c.x - rw / 2)}" y="${py(c.y + rh / 2)}" width="${len(rw)}" height="${len(rh)}" fill="none" stroke="$colour" stroke-dasharray="4 3"/>');
        case _:
      }
      switch c.cutout {
        case Circle(d): out.push('<circle cx="${px(c.x)}" cy="${py(c.y)}" r="${len(d / 2)}" fill="#fff" stroke="$colour"/>');
        case Rect(rw, rh): out.push('<rect x="${px(c.x - rw / 2)}" y="${py(c.y + rh / 2)}" width="${len(rw)}" height="${len(rh)}" fill="#fff" stroke="$colour"/>');
        case _:
      }
      for (hole in c.extraHoles)
        out.push('<circle cx="${px(c.x + hole.x)}" cy="${py(c.y + hole.y)}" r="${len(hole.diameter / 2)}" fill="#fff" stroke="$colour"/>');
      out.push('<text x="${px(c.x)}" y="${py(c.y)}" text-anchor="middle" dominant-baseline="middle" fill="$colour">${c.id}</text>');
    }
    out.push('</svg>');
    return out.join("\n") + "\n";
  }
}
