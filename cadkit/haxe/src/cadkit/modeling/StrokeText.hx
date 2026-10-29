package cadkit.modeling;

/**
 * Raised or engraved lettering as a solid, from a plain stroke font: capital letters, digits and a few signs, each drawn
 * as polylines on a 4 x 6 grid and swept to a stroke width with round joins. Meant for labels on printed and moulded parts
 * (terminal names, key legends), where the text is read in a few millimetres.
 */
class StrokeText {
  public static inline var CELL_WIDTH:Float = 4.0;
  public static inline var CELL_HEIGHT:Float = 6.0;
  public static inline var GAP:Float = 1.5;
  static var font:Null<Map<String, Array<Array<Float>>>>;

  /** Each glyph as strokes of `x,y` points separated by spaces, strokes separated by a slash. */
  static var source:Map<String, String> = [
    "A" => "0,0 2,6 4,0 / 0.8,2.4 3.2,2.4",
    "B" => "0,0 0,6 / 0,6 2.6,6 3.6,5.2 3.6,3.9 2.6,3.1 0,3.1 / 2.6,3.1 3.8,2.3 3.8,0.9 2.8,0 0,0",
    "C" => "4,5 3,6 1,6 0,5 0,1 1,0 3,0 4,1",
    "D" => "0,0 0,6 / 0,6 2.5,6 4,4.5 4,1.5 2.5,0 0,0",
    "E" => "4,6 0,6 0,0 4,0 / 0,3 3,3",
    "F" => "4,6 0,6 0,0 / 0,3 3,3",
    "G" => "4,5 3,6 1,6 0,5 0,1 1,0 3,0 4,1 4,3 2.2,3",
    "H" => "0,0 0,6 / 4,0 4,6 / 0,3 4,3",
    "I" => "1,6 3,6 / 2,6 2,0 / 1,0 3,0",
    "J" => "4,6 4,1 3,0 1,0 0,1",
    "K" => "0,0 0,6 / 4,6 0,2.5 / 1.4,3.6 4,0",
    "L" => "0,6 0,0 4,0",
    "M" => "0,0 0,6 2,3 4,6 4,0",
    "N" => "0,0 0,6 4,0 4,6",
    "O" => "2,0 1,0 0,1 0,5 1,6 2,6 / 2,6 3,6 4,5 4,1 3,0 2,0",
    "P" => "0,0 0,6 / 0,6 3,6 4,5 4,4 3,3 0,3",
    "Q" => "2,0 1,0 0,1 0,5 1,6 2,6 / 2,6 3,6 4,5 4,1 3,0 2,0 / 2.5,1.5 4,0",
    "R" => "0,0 0,6 / 0,6 3,6 4,5 4,4 3,3 0,3 / 2,3 4,0",
    "S" => "4,5 3,6 1,6 0,5 0,4 1,3 3,3 4,2 4,1 3,0 1,0 0,1",
    "T" => "0,6 4,6 / 2,6 2,0",
    "U" => "0,6 0,1 1,0 3,0 4,1 4,6",
    "V" => "0,6 2,0 4,6",
    "W" => "0,6 1,0 2,3 3,0 4,6",
    "X" => "0,6 4,0 / 4,6 0,0",
    "Y" => "0,6 2,3 4,6 / 2,3 2,0",
    "Z" => "0,6 4,6 0,0 4,0",
    "0" => "2,0 1,0 0,1 0,5 1,6 2,6 / 2,6 3,6 4,5 4,1 3,0 2,0",
    "1" => "1,5 2,6 2,0 / 1,0 3,0",
    "2" => "0,5 1,6 3,6 4,5 4,4 0,0 4,0",
    "3" => "0,5 1,6 3,6 4,5 4,4 3,3 1.5,3 / 3,3 4,2 4,1 3,0 1,0 0,1",
    "4" => "3,0 3,6 / 3,6 0,2 4,2",
    "5" => "4,6 0,6 0,3.2 3,3.5 4,2.5 4,1 3,0 1,0 0,1",
    "6" => "3.5,6 1,6 0,5 0,1 1,0 3,0 4,1 4,2.5 3,3.5 / 3,3.5 0.4,3.2",
    "7" => "0,6 4,6 1.5,0",
    "8" => "2,3 1,3 0,4 0,5 1,6 2,6 / 2,6 3,6 4,5 4,4 3,3 2,3 / 2,3 1,3 0,2 0,1 1,0 2,0 / 2,0 3,0 4,1 4,2 3,3 2,3",
    "9" => "0.5,0 3,0 4,1 4,5 3,6 1,6 0,5 0,3.5 1,2.5 / 1,2.5 3.6,2.8",
    "-" => "0.5,3 3.5,3",
    "+" => "0.5,3 3.5,3 / 2,1.5 2,4.5",
    "." => "2,0 2,0.01",
    "/" => "0,0 4,6"
  ];

  static function glyphs():Map<String, Array<Array<Float>>> {
    if (font != null) return font;
    var parsed:Map<String, Array<Array<Float>>> = [];
    for (name => text in source) {
      var strokes:Array<Array<Float>> = [];
      for (stroke in text.split("/")) {
        var numbers:Array<Float> = [];
        for (point in StringTools.trim(stroke).split(" ")) {
          var pair = point.split(",");
          numbers.push(Std.parseFloat(pair[0]));
          numbers.push(Std.parseFloat(pair[1]));
        }
        strokes.push(numbers);
      }
      parsed.set(name, strokes);
    }
    font = parsed;
    return parsed;
  }

  /** The strokes of one character as flat x, y pairs on the 4 x 6 grid, or null when the font has none. */
  public static function strokes(character:String):Null<Array<Array<Float
    >>> return glyphs().get(character.toUpperCase());

  /** The text's width when its letters are `height` tall. */
  public static function width(text:String, height:Float):Float {
    var n = text.length;
    return n == 0 ? 0.0 : (n * CELL_WIDTH + (n - 1) * GAP) * height / CELL_HEIGHT;
  }

  /**
   * The text as a solid from z0 to z1: its baseline on y = 0, centred on x = 0, letters `height` tall drawn with strokes
   * `stroke` wide. `mirror` flips it left to right, for lettering read from the other side of a part. `rotation` (radians, counter-clockwise)
   * turns it about its own origin and `placeX`, `placeY` then move that origin. `rotation` (radians,
   * counter-clockwise) turns it about its own origin and `placeX`, `placeY` then move that origin. Each stroke is one
   * mitred polygon, so a few letters are a handful of solids to fuse.
   */
  public static function solid(
    text:String,
    height:Float,
    stroke:Float,
    z0:Float,
    z1:Float,
    mirror:Bool = false,
    placeX:Float = 0.0,
    placeY:Float = 0.0,
    rotation:Float = 0.0
  ):Part {
    if (!(height > 0)
      || !(stroke > 0) || !(z1 > z0) || text.length == 0) throw "Text needs a height, a stroke, a depth and characters";
    var scale = height / CELL_HEIGHT, half = stroke / 2, total = width(text, height);
    var pieces:Array<Part> = [];
    try {
      for (index in 0...text.length) {
        var strokes = glyphs().get(text.charAt(index).toUpperCase());
        if (strokes == null) {
          if (text.charAt(index) == " ") continue;
          throw 'No stroke glyph for "${text.charAt(index)}"';
        }
        var left = index * (CELL_WIDTH + GAP) * scale - total / 2;
        for (line in strokes) {
          var points = [for (i in 0...Std.int(line.length / 2)) new Vector(
            left + line[2 * i] * scale,
            line[2 * i + 1] * scale
          )];
          var outline = strokeOutline(points, half);
          if (mirror) {
            outline = [for (point in outline) new Vector(-point.x, point.y)];
            outline.reverse();
          }
          if (rotation != 0.0 || placeX != 0.0 || placeY != 0.0) {
            var c = Math.cos(rotation), d = Math.sin(rotation);
            outline = [for (point in outline) new Vector(
              placeX + point.x * c - point.y * d,
              placeY + point.x * d + point.y * c
            )];
          }
          pieces.push(Part.prism(outline, z0, z1));
        }
      }
      var result = pieces.length == 1 ? pieces[0] : Part.fuseAll(pieces);
      if (pieces.length > 1) for (piece in pieces) piece.close();
      return result;
    } catch (error:Dynamic) {
      for (piece in pieces) piece.close();
      throw error;
    }
  }

  /**
   * The outline of a polyline thickened to `half` each side, counter-clockwise: the left offsets forward and the right ones
   * back, with mitred corners (clamped at three half-widths so a sharp turn does not spike).
   */
  static function strokeOutline(points:Array<Vector>, half:Float):Array<Vector> {
    var count = points.length;
    var normals:Array<Vector> = [];
    for (i in 0...count - 1) {
      var dx = points[i + 1].x - points[i].x, dy = points[i + 1].y - points[i].y, length = Math.sqrt(dx * dx + dy * dy);
      normals.push(length < 1e-9 ? new Vector(0, 1) : new Vector(-dy / length, dx / length));
    }
    var offsets:Array<Vector> = [];
    for (i in 0...count) {
      if (i == 0) offsets.push(new Vector(normals[0].x * half, normals[0].y * half));
      else if (i == count - 1) offsets.push(new Vector(normals[count - 2].x * half, normals[count - 2].y * half));
      else {
        var a = normals[i - 1], b = normals[i], dot = a.x * b.x + a.y * b.y;
        var scaleMiter = half / Math.max(1.0 + dot, 1e-6);
        var mx = (a.x + b.x) * scaleMiter, my = (a.y + b.y) * scaleMiter, size = Math.sqrt(mx * mx + my * my);
        if (size > 3 * half) {
          mx *= 3 * half / size;
          my *= 3 * half / size;
        }
        offsets.push(new Vector(mx, my));
      }
    }
    var outline:Array<Vector> = [for (i in 0...count) new Vector(points[i].x + offsets[i].x,
      points[i].y + offsets[i].y)];
    for (i in 0...count) {
      var k = count - 1 - i;
      outline.push(new Vector(points[k].x - offsets[k].x, points[k].y - offsets[k].y));
    }
    return outline;
  }
}
