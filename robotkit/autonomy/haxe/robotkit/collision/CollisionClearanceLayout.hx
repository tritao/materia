package robotkit.collision;

import collisionkit.CollisionDescription;
import collisionkit.CollisionGeometry;
import collisionkit.kinematics.ModelBodies;
import robotkit.spatial.Transform3;

/** The description plus its mechanical carrier mapping; policy stays in the description. */
class CollisionClearanceLayout {
  public final description:CollisionDescription;
  public final bodies:ModelBodies;

  public function new(description:CollisionDescription, bodies:ModelBodies) {
    if (description == null || bodies == null) throw "Described clearance needs geometry and kinematics";
    this.description = description; this.bodies = bodies;
  }

  /** Enclosing corners for motion bounds only. Native queries keep the exact described geometry. */
  public function corners(body:Int):Array<Float> {
    var result:Array<Float> = [];
    for (object in description.objects) if (object.body == body) {
      var points:Array<Float> = switch object.geometry {
        case Convex(vertices) | Mesh(vertices, _): vertices;
        case Box(x, y, z): CollisionSceneBuilder.box(x, y, z);
        case Sphere(r): CollisionSceneBuilder.box(r, r, r);
        case Cylinder(r, h): CollisionSceneBuilder.box(r, r, h);
        case Capsule(r, h): CollisionSceneBuilder.box(r, r, h + r);
        case HalfSpace(_, _, _, _) | HeightField(_, _, _, _, _):
          throw 'Clearance motion bounds need finite geometry for "${object.name}"';
      };
      var at = object.offset;
      var pose = Transform3.fromArrays([at.x, at.y, at.z], [at.qx, at.qy, at.qz, at.qw]);
      for (i in 0...Std.int(points.length / 3)) {
        var p = pose.transformPoint(new robotkit.spatial.Vec3(points[3 * i], points[3 * i + 1], points[3 * i + 2]));
        // Inflation is a geometric radius, not a change to the collision mesh.
        var r = object.inflation;
        if (r == 0) { result.push(p.x); result.push(p.y); result.push(p.z); }
        else for (corner in 0...8) {
          result.push(p.x + ((corner & 1) == 0 ? -r : r));
          result.push(p.y + ((corner & 2) == 0 ? -r : r));
          result.push(p.z + ((corner & 4) == 0 ? -r : r));
        }
      }
    }
    return result;
  }

}
