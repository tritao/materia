package materia.project;

import haxe.io.Bytes;

typedef MeshMassData = {
  var volume:Float;
  var centerOfMass:Array<Float>;
  /** Unit-density inertia tensor about the centre, row major, in model-unit^5. */
  var inertia:Array<Float>;
}

/** Integrates a closed, consistently wound triangle mesh as signed tetrahedra. */
class MeshMassProperties {
  public static function compute(vertices:Bytes, indices:Bytes):MeshMassData {
    if (vertices == null || indices == null || vertices.length % 24 != 0 || indices.length % 12 != 0)
      throw "Invalid mesh for mass properties";
    var volume = 0.0;
    var first = [0.0, 0.0, 0.0];
    var second = [for (_ in 0...9) 0.0];
    for (triangle in 0...Std.int(indices.length / 12)) {
      var points:Array<Array<Float>> = [];
      for (corner in 0...3) {
        var index = indices.getInt32(triangle * 12 + corner * 4);
        if (index < 0 || index * 24 + 24 > vertices.length)
          throw "Invalid mesh index for mass properties";
        points.push([for (axis in 0...3) vertices.getDouble(index * 24 + axis * 8)]);
      }
      var a = points[0], b = points[1], c = points[2];
      var tetra = (a[0] * (b[1] * c[2] - b[2] * c[1]) +
        a[1] * (b[2] * c[0] - b[0] * c[2]) +
        a[2] * (b[0] * c[1] - b[1] * c[0])) / 6.0;
      volume += tetra;
      for (axis in 0...3) first[axis] += tetra * (a[axis] + b[axis] + c[axis]) / 4.0;
      for (row in 0...3) for (column in 0...3) {
        var sumRow = a[row] + b[row] + c[row];
        var sumColumn = a[column] + b[column] + c[column];
        var paired = a[row] * a[column] + b[row] * b[column] + c[row] * c[column];
        second[row * 3 + column] += tetra * (sumRow * sumColumn + paired) / 20.0;
      }
    }
    if (!Math.isFinite(volume) || Math.abs(volume) < 1e-12)
      throw "Mesh has no closed positive volume";
    var sign = volume < 0 ? -1.0 : 1.0;
    volume *= sign;
    var center = [for (axis in 0...3) first[axis] * sign / volume];
    var trace = sign * (second[0] + second[4] + second[8]);
    var distanceSquared = center[0] * center[0] + center[1] * center[1] + center[2] * center[2];
    var inertia:Array<Float> = [];
    for (row in 0...3) for (column in 0...3) {
      var identity = row == column ? 1.0 : 0.0;
      inertia.push(identity * trace - sign * second[row * 3 + column] -
        volume * (identity * distanceSquared - center[row] * center[column]));
    }
    return {volume: volume, centerOfMass: center, inertia: inertia};
  }
}
