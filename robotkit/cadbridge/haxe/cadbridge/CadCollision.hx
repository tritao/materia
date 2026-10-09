package cadbridge;

import cadkit.Mesh;
import cadkit.Shape;
import collisionkit.CollisionDescription;
import collisionkit.CollisionGeometry;
import collisionkit.CollisionPose;
import collisionkit.ConvexDecomposer;
import collisionkit.ConvexDecomposition;
import haxe.io.Bytes;
import materia.assembly.AssemblyFrames.AssemblyFrame;
import robotkit.collision.RobotCollision;

/** How a CAD part enters a collision description (CL-D8: solids are convex, meshes are surfaces). */
enum CadPartCollision {
  /** A convex hull (x, y, z triples in CAD units, e.g. `AssemblyPhysicalPart.collisionHull`, an enclosing hull). */
  Hull(vertices:Array<Float>);
  /** The part's tessellation as a surface mesh: floors, walls, sheet. */
  Surface(shape:Shape, linearDeflection:Float);
  /** A non-convex solid as convex pieces (CL-D11), inflated to enclose its tessellation. */
  Decomposed(shape:Shape, linearDeflection:Float, maxPieces:Int);
}

/** Decompositions per part, kept with the parts so a part is decomposed again only when it changes (CL-D11). */
class CadDecompositionCache {
  final entries = new Map<String, ConvexDecomposition>();
  public var decompositions(default, null) = 0;

  public function new() {}

  /** The cached decomposition of `mesh` for `part`, or a new one; the key covers the mesh's contents. */
  public function get(part:String, mesh:CadCollisionMesh, maxPieces:Int, decomposer:ConvexDecomposer):ConvexDecomposition {
    var key = '$part|$maxPieces|${mesh.fingerprint()}';
    var found = entries.get(key);
    if (found != null) return found;
    var made = decomposer.decompose(mesh.vertices, mesh.indices, maxPieces, 0.0);
    decompositions++;
    entries.set(key, made);
    return made;
  }
}

/** A tessellation as flat arrays, in metres. */
class CadCollisionMesh {
  public final vertices:Array<Float>;
  public final indices:Array<Int>;
  public final linearDeflection:Float;

  public function new(vertices:Array<Float>, indices:Array<Int>, linearDeflection:Float) {
    this.vertices = vertices;
    this.indices = indices;
    this.linearDeflection = linearDeflection;
  }

  public static function of(mesh:Mesh, metresPerUnit:Float):CadCollisionMesh {
    var vertices = [for (i in 0...mesh.vertexCount * 3) mesh.vertices.getDouble(i * 8) * metresPerUnit];
    var indices = [for (i in 0...mesh.indexCount) mesh.indices.getInt32(i * 4)];
    return new CadCollisionMesh(vertices, indices, mesh.linearDeflection * metresPerUnit);
  }

  /** A content key: counts and a hash of every coordinate and index. */
  public function fingerprint():String {
    var hash = 17;
    var bytes = Bytes.alloc(8);
    for (value in vertices) {
      bytes.setDouble(0, value);
      hash = (hash * 31 + bytes.getInt32(0)) | 0;
      hash = (hash * 31 + bytes.getInt32(4)) | 0;
    }
    for (index in indices) hash = (hash * 31 + index) | 0;
    return '${vertices.length}/${indices.length}/$hash';
  }
}

/**
 * CAD geometry in a collision description (COLLISION.md CL3a): mechanisms
 * as articulations posed from their own model (through RobotKit, from
 * `AssemblySimulationBridge.toRobotModel`), and parts and fixtures at their
 * world pose as hulls, surface meshes or decomposed solids.
 */
class CadCollision {
  /** A CAD mechanism, converted to a RobotKit model, with its link hulls (CAD-sourced). */
  public static function describeMechanism(description:CollisionDescription, mechanism:AssemblySimulationModel,
      options:RobotCollisionOptions):RobotCollisionBodies {
    options.hulls = options.hulls.concat([for (hull in mechanism.linkHulls)
      new RobotLinkHull(hull.link, hull.part, hull.vertices)]);
    description.note('Mechanism "${options.name}": CAD hulls are enclosing hulls of the parts\' tessellations');
    return RobotCollision.describe(description, mechanism.model, options);
  }

  /**
   * A fixed part at `pose` (CAD units, scaled by `metresPerUnit`) in
   * `group`; returns its described objects. Decomposition needs a
   * `decomposer` and caches through `cache`.
   */
  public static function describePart(description:CollisionDescription, name:String, pose:AssemblyFrame,
      metresPerUnit:Float, collision:CadPartCollision, group:Int, ?decomposer:ConvexDecomposer,
      ?cache:CadDecompositionCache):Array<Int> {
    var body = description.addBody(name, group, true, new CollisionPose(pose.x * metresPerUnit,
      pose.y * metresPerUnit, pose.z * metresPerUnit, pose.qx, pose.qy, pose.qz, pose.qw));
    var at = CollisionPose.identity();
    return switch collision {
      case Hull(vertices):
        description.note('Part "$name" is its enclosing convex hull');
        [description.addObject(name, body, at, CollisionGeometry.Convex([for (v in vertices) v * metresPerUnit]), 0.0,
          "enclosing hull")];
      case Surface(shape, deflection):
        var mesh = CadCollisionMesh.of(shape.tessellate(deflection), metresPerUnit);
        description.note('Part "$name" is a surface mesh (a shape inside it reads as clear), tessellated to ${mesh.linearDeflection} m');
        [description.addObject(name, body, at, CollisionGeometry.Mesh(mesh.vertices, mesh.indices), 0.0, "surface mesh")];
      case Decomposed(shape, deflection, maxPieces):
        if (decomposer == null) throw 'Part "$name" needs a convex decomposer';
        var mesh = CadCollisionMesh.of(shape.tessellate(deflection), metresPerUnit);
        var pieces = cache == null ? decomposer.decompose(mesh.vertices, mesh.indices, maxPieces, 0.0)
          : cache.get(name, mesh, maxPieces, decomposer);
        description.note('Part "$name" is ${pieces.pieces.length} convex pieces inflated by ${pieces.inflation} m to enclose its tessellation (measured ${pieces.measured} m, sampled every ${pieces.spacing} m); the tessellation lies within ${mesh.linearDeflection} m of the part');
        [for (i in 0...pieces.pieces.length)
          description.addObject('$name/piece$i', body, at, CollisionGeometry.Convex(pieces.pieces[i]), pieces.inflation,
            "decomposed piece")];
    }
  }
}
