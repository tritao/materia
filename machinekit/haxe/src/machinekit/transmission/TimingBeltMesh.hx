package machinekit.transmission;

/** A display mesh along the posed pitch line; phase is belt travel in millimetres. */
class TimingBeltMesh {
  public static function build(belt:TimingBelt, phase:Float = 0.0, toothSide:Int = 1, ?thickness:Float):{
      positions:Array<Float>, normals:Array<Float>, indices:Array<Int>} {
    if (!Math.isFinite(phase) || (toothSide != 1 && toothSide != -1)) throw "Invalid belt mesh phase or tooth side";
    var band = thickness == null ? belt.thickness : thickness;
    if (!Math.isFinite(band) || band <= 0.0) throw "Invalid belt mesh thickness";
    if (belt.teeth > 200000) throw "Belt display mesh has too many teeth";
    var toothPitch = belt.length / belt.teeth;
    phase *= toothPitch / belt.pitch;
    var positions:Array<Float> = [], normals:Array<Float> = [], indices:Array<Int> = [];
    function face(points:Array<Array<Float>>):Void {
      if (toothSide == 1) points.reverse();
      var a = points[0], b = points[1], c = points[2];
      var u = [for (i in 0...3) b[i] - a[i]], v = [for (i in 0...3) c[i] - a[i]];
      var n = [u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0]];
      var length = Math.sqrt(n[0] * n[0] + n[1] * n[1] + n[2] * n[2]);
      if (length <= 1e-12) return;
      var base = Std.int(positions.length / 3);
      for (point in points) {
        for (value in point) positions.push(value);
        for (value in n) normals.push(value / length);
      }
      for (index in [0, 1, 2, 0, 2, 3]) indices.push(base + index);
    }
    function ring(distance:Float, depth:Float):Array<Array<Float>> {
      var p = belt.pointAt(distance), nx = -p.dy * toothSide, ny = p.dx * toothSide;
      var back = -band * 0.35, front = depth;
      return [[p.x + nx * back, p.y + ny * back, 0.0],
        [p.x + nx * front, p.y + ny * front, 0.0],
        [p.x + nx * front, p.y + ny * front, belt.width],
        [p.x + nx * back, p.y + ny * back, belt.width]];
    }
    // Material teeth travel with phase. Sample each tooth's flanks and land, plus the closing seam.
    var distances:Array<Float> = [0.0];
    var first = Math.floor(-phase / toothPitch) - 1;
    var last = Math.ceil((belt.length - phase) / toothPitch) + 1;
    for (tooth in Std.int(first)...Std.int(last) + 1)
      for (fraction in [0.0, 0.2, 0.35, 0.65, 0.8]) {
        var distance = phase + (tooth + fraction) * toothPitch;
        if (distance > 1e-8 && distance < belt.length - 1e-8) distances.push(distance);
      }
    distances.push(belt.length);
    distances.sort(Reflect.compare);
    function depth(distance:Float):Float {
      var fraction = (distance - phase) / toothPitch;
      fraction -= Math.floor(fraction);
      var height = fraction < 0.2 || fraction > 0.8 ? 0.0 :
        fraction < 0.35 ? (fraction - 0.2) / 0.15 : fraction <= 0.65 ? 1.0 : (0.8 - fraction) / 0.15;
      return band * (0.1 + 0.55 * height);
    }
    for (i in 1...distances.length) {
      var a = ring(distances[i - 1], depth(distances[i - 1])), b = ring(distances[i], depth(distances[i]));
      for (side in 0...4) { var next = (side + 1) % 4; face([a[side], b[side], b[next], a[next]]); }
    }
    return {positions: positions, normals: normals, indices: indices};
  }
}
