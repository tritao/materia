package robotkit.manipulation;

import robotkit.spatial.Transform3;
import robotkit.spatial.Vec3;
import robotkit.tool.ToolCollisionShape;

private typedef ClearancePiece = {
  vertices:Array<Vec3>,
  normals:Array<Vec3>,
  edges:Array<Vec3>
};

/** Conservative convex-tool clearance against oriented boxes. Tool poses and
 * obstacles use the manipulator base frame. Joint motion is sampled; callers
 * choose a joint step small enough for their required path resolution. */
class ToolClearanceChecker {
  final pieces:Array<ClearancePiece> = [];
  final obstacles:Array<ToolBoxObstacle>;
  final clearance:Float;

  public function new(shape:ToolCollisionShape, obstacles:Array<ToolBoxObstacle>, ?clearance:Float = 0.0) {
    if (shape == null || obstacles == null || !Math.isFinite(clearance) || clearance < 0)
      throw "Tool clearance requires a shape, obstacles, and non-negative clearance";
    this.obstacles = obstacles.copy();
    var padding = 0.0;
    switch (shape) {
      case NoCollision:
      case Box(half, centre):
        if (half.x <= 0 || half.y <= 0 || half.z <= 0) throw "Tool box half-extents must be positive";
        var c = centre == null ? Vec3.zero() : centre;
        var vertices:Array<Vec3> = [];
        for (x in [-1.0, 1.0]) for (y in [-1.0, 1.0]) for (z in [-1.0, 1.0])
          vertices.push(c.add(new Vec3(x * half.x, y * half.y, z * half.z)));
        pieces.push(prepare(vertices));
      case Cylinder(radius, height):
        if (radius <= 0 || height <= 0) throw "Tool cylinder dimensions must be positive";
        // Enclosing box is conservative for this less common shape.
        var vertices:Array<Vec3> = [];
        for (x in [-1.0, 1.0]) for (y in [-1.0, 1.0]) for (z in [-1.0, 1.0])
          vertices.push(new Vec3(x * radius, y * radius, z * height * 0.5));
        pieces.push(prepare(vertices));
      case Hulls(hulls, hullPadding):
        if (!Math.isFinite(hullPadding) || hullPadding < 0) throw "Tool hull padding must be non-negative";
        padding = hullPadding;
        for (hull in hulls) {
          if (hull == null || hull.length < 12 || hull.length % 3 != 0)
            throw "Tool hull requires at least four vertices";
          var vertices:Array<Vec3> = [];
          for (i in 0...Std.int(hull.length / 3))
            vertices.push(new Vec3(hull[3 * i], hull[3 * i + 1], hull[3 * i + 2]));
          pieces.push(prepare(vertices));
        }
    }
    this.clearance = clearance + padding;
  }

  public function isClear(base_T_flange:Transform3):Bool {
    if (base_T_flange == null) throw "Tool clearance requires a flange pose";
    for (obstacle in obstacles) {
      var box_T_flange = obstacle.pose.inverse().compose(base_T_flange);
      for (piece in pieces) if (!separated(piece, box_T_flange, obstacle.halfExtents)) return false;
    }
    return true;
  }

  public function clearJointSegment(manipulator:Manipulator, from:Array<Float>, to:Array<Float>,
      ?maxJointStep:Float = 0.02):Bool {
    if (manipulator == null || from == null || to == null || from.length != to.length ||
        !Math.isFinite(maxJointStep) || maxJointStep <= 0)
      throw "Tool segment requires matching joints and a positive sampling step";
    var steps = 1;
    for (i in 0...from.length) {
      if (!Math.isFinite(from[i]) || !Math.isFinite(to[i])) throw "Tool segment joints must be finite";
      steps = Std.int(Math.max(steps, Math.ceil(Math.abs(to[i] - from[i]) / maxJointStep)));
    }
    for (step in 0...(steps + 1)) {
      var t = step / steps;
      var q = [for (i in 0...from.length) from[i] + (to[i] - from[i]) * t];
      if (!isClear(manipulator.forwardKinematics(q))) return false;
    }
    return true;
  }

  static function prepare(vertices:Array<Vec3>):ClearancePiece {
    var normals:Array<Vec3> = [];
    var n = vertices.length;
    for (i in 0...n) for (j in i + 1...n) for (k in j + 1...n) {
      var cross = vertices[j].sub(vertices[i]).cross(vertices[k].sub(vertices[i]));
      if (cross.norm() < 1e-10) continue;
      var normal = cross.normalized();
      var low = 0.0, high = 0.0;
      for (vertex in vertices) {
        var d = normal.dot(vertex.sub(vertices[i]));
        low = Math.min(low, d); high = Math.max(high, d);
      }
      if (low < -1e-8 && high > 1e-8) continue;
      if (low < -1e-8) normal = normal.negate();
      var duplicate = false;
      for (existing in normals) if (Math.abs(existing.dot(normal)) > 1.0 - 1e-8) {
        duplicate = true; break;
      }
      if (!duplicate) normals.push(normal);
    }
    if (normals.length < 3) throw "Tool hull must enclose a volume";
    var edges:Array<Vec3> = [];
    for (i in 0...n) for (j in i + 1...n) {
      var shared:Array<Vec3> = [];
      for (normal in normals) {
        var min = Math.POSITIVE_INFINITY, max = Math.NEGATIVE_INFINITY;
        for (vertex in vertices) {
          var value = normal.dot(vertex);
          min = Math.min(min, value); max = Math.max(max, value);
        }
        var a = normal.dot(vertices[i]), b = normal.dot(vertices[j]);
        if ((Math.abs(a - min) < 1e-7 && Math.abs(b - min) < 1e-7) ||
            (Math.abs(a - max) < 1e-7 && Math.abs(b - max) < 1e-7)) shared.push(normal);
      }
      var edge = false;
      for (a in 0...shared.length) for (b in a + 1...shared.length)
        if (shared[a].cross(shared[b]).norm() > 1e-7) edge = true;
      if (!edge) continue;
      var direction = vertices[j].sub(vertices[i]).normalized();
      var duplicate = false;
      for (existing in edges) if (Math.abs(existing.dot(direction)) > 1.0 - 1e-8) {
        duplicate = true; break;
      }
      if (!duplicate) edges.push(direction);
    }
    return {vertices: vertices, normals: normals, edges: edges};
  }

  function separated(piece:ClearancePiece, box_T_flange:Transform3, half:Vec3):Bool {
    var vertices = [for (vertex in piece.vertices) box_T_flange.transformPoint(vertex)];
    var axes = [new Vec3(1, 0, 0), new Vec3(0, 1, 0), new Vec3(0, 0, 1)];
    for (normal in piece.normals) axes.push(box_T_flange.transformVector(normal));
    for (edge in piece.edges) {
      var direction = box_T_flange.transformVector(edge);
      for (axis in axes.slice(0, 3)) {
        var cross = direction.cross(axis);
        if (cross.norm() > 1e-10) axes.push(cross.normalized());
      }
    }
    for (axis in axes) {
      var radius = Math.abs(axis.x) * half.x + Math.abs(axis.y) * half.y + Math.abs(axis.z) * half.z;
      var low = Math.POSITIVE_INFINITY, high = Math.NEGATIVE_INFINITY;
      for (vertex in vertices) {
        var projection = axis.dot(vertex);
        low = Math.min(low, projection); high = Math.max(high, projection);
      }
      if (low > radius + clearance || high < -radius - clearance) return true;
    }
    return false;
  }
}
