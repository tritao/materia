package app.editor;

/** A face picked on a part: its index among the part's faces, where the ray hit it, and its persistent name when the part has one. */
class PickedFace {
  /** Index of the face in the part's face list (the artifact's `faceRanges.faceIndex`). */
  public final index:Int;
  /** Hit point in the part's own frame, mm; null when it could not be worked out. */
  public final pointMm:Null<Array<Float>>;
  /** Persistent topological name (CAD-document parts). */
  public final name:Null<String>;

  public function new(index:Int, ?pointMm:Array<Float>, ?name:String) {
    this.index = index;
    this.pointMm = pointMm;
    this.name = name;
  }

  public function same(other:Null<PickedFace>):Bool
    return other != null && index == other.index && name == other.name && Std.string(pointMm) == Std.string(other.pointMm);
}

/** The text the status bar shows for the selection and the text clicking it copies. */
class SelectionReference {
  /** `label`, with the picked face appended when there is one. */
  public static function statusText(label:String, face:Null<PickedFace>):String
    return face == null ? label : label + " · face " + face.index;

  /** The object reference (`SceneTreeReference.text()`) followed by what is known about the picked face. */
  public static function copyText(reference:String, face:Null<PickedFace>):String {
    if (face == null) return reference;
    var lines = [reference, "Face: " + face.index];
    if (face.name != null && face.name.length > 0) lines.push("Face name: " + singleLine(face.name));
    var point = face.pointMm;
    if (point != null && point.length == 3) lines.push("Point: " + number(point[0]) + " " + number(point[1]) + " " + number(point[2])
      + " (part frame, mm)");
    return lines.join("\n");
  }

  /**
   * `world` (x, y, z, in the scene's metres) in the frame of a node whose column-major 4x4 world transform is `matrix`, in mm.
   * The inverse of a rotation plus translation (the transform of every part), so the rotation block is transposed.
   */
  public static function localPointMm(matrix:Array<Float>, x:Float, y:Float, z:Float):Null<Array<Float>> {
    if (matrix == null || matrix.length != 16) return null;
    var dx = x - matrix[12], dy = y - matrix[13], dz = z - matrix[14];
    var local = [
      matrix[0] * dx + matrix[1] * dy + matrix[2] * dz,
      matrix[4] * dx + matrix[5] * dy + matrix[6] * dz,
      matrix[8] * dx + matrix[9] * dy + matrix[10] * dz
    ];
    return [for (value in local) value * 1000.0];
  }

  /** Three decimals, trailing zeros dropped, no exponent, and no negative zero. */
  public static function number(value:Float):String {
    var rounded = Math.round(value * 1000.0) / 1000.0;
    if (rounded == 0.0) return "0";
    var text = Std.string(rounded);
    return text;
  }

  static function singleLine(value:String):String
    return StringTools.replace(StringTools.replace(value, "\r", "\\r"), "\n", "\\n");
}
