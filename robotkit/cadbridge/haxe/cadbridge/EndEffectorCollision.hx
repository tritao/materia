package cadbridge;

import cadkit.ConvexHullVertices;
import cadkit.modeling.AssemblyState;
import haxe.io.Bytes;
import machinekit.component.ComponentDetail;
import machinekit.robotics.EndEffector;
import machinekit.robotics.EndEffector.EndEffectorSolvedContext;
import machinekit.robotics.EndEffectorFrames;
import materia.assembly.AssemblyFrames;

typedef CollisionPiece = {memberIds:Array<String>, vertices:Array<Float>};
typedef CollisionDiagnostic = {instanceId:String, warning:Null<String>, errorRatio:Float};
typedef EndEffectorCollisionResult = {
  pieces:Array<CollisionPiece>, merged:Array<String>, excluded:Array<String>,
  diagnostics:Array<CollisionDiagnostic>, padding:Float
};

/** Geometry policy, in millimetres except for the piece limit. */
class EndEffectorCollisionOptions {
  public final minFeature:Float;
  public final maxPieces:Int;
  public final padding:Float;

  public function new(minFeature:Float = 10, maxPieces:Int = 16, padding:Float = 0) {
    if (!Math.isFinite(minFeature) || minFeature < 0 || maxPieces < 1 ||
        !Math.isFinite(padding) || padding < 0)
      throw "Invalid end effector collision options";
    this.minFeature = minFeature;
    this.maxPieces = maxPieces;
    this.padding = padding;
  }
}

/** Conservative per-member convex pieces in the robot flange frame. */
class EndEffectorCollision {
  public static function pieces(effector:EndEffector, ?state:AssemblyState,
      ?options:EndEffectorCollisionOptions,
      ?solved:EndEffectorSolvedContext):EndEffectorCollisionResult {
    if (effector == null) throw "End effector is required";
    var settings = options == null ? new EndEffectorCollisionOptions() : options;
    var context = solved == null ? effector.solve(state) : solved;
    var inverse = AssemblyFrames.inverse(context.mountWorld);
    var result:Array<CollisionPiece> = [];
    var excluded:Array<String> = [];
    var diagnostics:Array<CollisionDiagnostic> = [];
    for (member in effector.components()) {
      if (effector.collisionExcluded(member.id)) {
        excluded.push(member.id);
        continue;
      }
      var pose = context.poses.get(member.id);
      if (pose == null) throw 'Missing solved pose for "${member.id}"';
      var relative = AssemblyFrames.compose(inverse, pose);
      var part = member.component.geometry(Envelope);
      var mesh:cadkit.Mesh;
      try mesh = part.shape.tessellateRelative() catch (error:Dynamic) {
        part.close();
        throw error;
      }
      part.close();
      var hull = ConvexHullVertices.enclosingFromMesh(mesh.vertices, mesh.vertexCount, 0.1);
      var vertices:Array<Float> = [];
      for (index in 0...Std.int(hull.vertices.length / 3)) {
        var at = index * 3;
        var point = AssemblyFrames.transformPoint(relative,
          hull.vertices[at], hull.vertices[at + 1], hull.vertices[at + 2]);
        var robot = EndEffectorFrames.pointYToZ(point.x, point.y, point.z);
        vertices.push(robot.x);
        vertices.push(robot.y);
        vertices.push(robot.z);
      }
      diagnostics.push({instanceId: member.id, warning: hull.warning, errorRatio: hull.errorRatio});
      result.push({memberIds: [member.id], vertices: vertices});
    }
    if (result.length == 0) throw "End effector has no collision members";
    var merged:Array<String> = [];
    while (result.length > 1) {
      var smallest = -1, size = Math.POSITIVE_INFINITY;
      for (index in 0...result.length) {
        var diameter = diagonal(result[index].vertices);
        if ((diameter < settings.minFeature || result.length > settings.maxPieces) && diameter < size) {
          smallest = index;
          size = diameter;
        }
      }
      if (smallest < 0) break;
      var neighbour = -1, distance = Math.POSITIVE_INFINITY;
      var centre = centroid(result[smallest].vertices);
      for (index in 0...result.length) if (index != smallest) {
        var other = centroid(result[index].vertices);
        var squared = Math.pow(other[0] - centre[0], 2) + Math.pow(other[1] - centre[1], 2) +
          Math.pow(other[2] - centre[2], 2);
        if (squared < distance) { distance = squared; neighbour = index; }
      }
      var absorbed = result[smallest], retained = result[neighbour];
      var combined = retained.vertices.concat(absorbed.vertices);
      var mesh = Bytes.alloc(Std.int(combined.length / 3) * 24);
      for (index in 0...combined.length) mesh.setDouble(Std.int(index / 3) * 24 + (index % 3) * 8,
        combined[index]);
      retained.vertices = ConvexHullVertices.enclosingFromMesh(mesh, Std.int(combined.length / 3), 0.1).vertices;
      for (id in absorbed.memberIds) {
        retained.memberIds.push(id);
        merged.push(id);
      }
      result.remove(absorbed);
    }
    for (piece in result) for (index in 0...piece.vertices.length) piece.vertices[index] /= 1000;
    return {pieces: result, merged: merged, excluded: excluded,
      diagnostics: diagnostics, padding: settings.padding / 1000};
  }

  static function centroid(points:Array<Float>):Array<Float> {
    var centre = [0.0, 0.0, 0.0];
    for (index in 0...points.length) centre[index % 3] += points[index];
    for (axis in 0...3) centre[axis] /= points.length / 3;
    return centre;
  }

  static function diagonal(points:Array<Float>):Float {
    var lo = [Math.POSITIVE_INFINITY, Math.POSITIVE_INFINITY, Math.POSITIVE_INFINITY];
    var hi = [Math.NEGATIVE_INFINITY, Math.NEGATIVE_INFINITY, Math.NEGATIVE_INFINITY];
    for (index in 0...points.length) {
      var axis = index % 3;
      lo[axis] = Math.min(lo[axis], points[index]);
      hi[axis] = Math.max(hi[axis], points[index]);
    }
    return Math.sqrt(Math.pow(hi[0] - lo[0], 2) + Math.pow(hi[1] - lo[1], 2) +
      Math.pow(hi[2] - lo[2], 2));
  }
}
