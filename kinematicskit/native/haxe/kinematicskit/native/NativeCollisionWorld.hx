package kinematicskit.native;

import KinematicsKitNative;
import kinematicskit.CollisionDistance;
import kinematicskit.CollisionGeometry;
import kinematicskit.CollisionPair;
import kinematicskit.CollisionPairRule;
import kinematicskit.CollisionPairStatus;
import kinematicskit.CollisionWorld;
import kinematicskit.KinematicModel;
import kinematicskit.KinematicState;
import kinematicskit.Transform;
import kinematicskit.Vector3;

/**
 * A `CollisionWorld` on coal, through the kinematicskit-native C ABI
 * (`kk_collision_*`, see `kinematicskit.h`). It keeps its own copy of the
 * model. Owns the native handle; call `dispose` when done.
 */
class NativeCollisionWorld implements CollisionWorld {
  public final model:KinematicModel;
  final owner:Ownedkk_collision_world_handle;
  var disposed = false;
  /** Room for this many pairs in the first query call; grown to the largest count seen. */
  var capacity = 32;

  public function new(model:KinematicModel) {
    if (model == null) throw "A collision world requires a model";
    this.model = model;
    var kinematics = new NativeKinematics(model);
    var created = KinematicsKitNative.kk_collision_world_create(kinematics.handle());
    kinematics.dispose();
    check(created.status, "world.create");
    owner = created.out_world;
  }

  public function add(body:Int, offset:Transform, geometry:CollisionGeometry):Int {
    requireLive();
    var at = flat(offset);
    var shape = (kind:Int, params:Array<Float>) -> {
      var result = KinematicsKitNative.kk_collision_add_shape(owner.borrow(), body, at, kind, params);
      check(result.status, "add");
      return result.out_object;
    };
    return switch geometry {
      case Box(halfX, halfY, halfZ): shape(KinematicsKitNativeConstants.KK_SHAPE_BOX, [halfX, halfY, halfZ]);
      case Sphere(radius): shape(KinematicsKitNativeConstants.KK_SHAPE_SPHERE, [radius]);
      case Capsule(radius, halfLength): shape(KinematicsKitNativeConstants.KK_SHAPE_CAPSULE, [radius, halfLength]);
      case Cylinder(radius, halfLength): shape(KinematicsKitNativeConstants.KK_SHAPE_CYLINDER, [radius, halfLength]);
      case HalfSpace(nx, ny, nz, offsetAlong):
        shape(KinematicsKitNativeConstants.KK_SHAPE_HALFSPACE, [nx, ny, nz, offsetAlong]);
      case Convex(points):
        var result = KinematicsKitNative.kk_collision_add_convex(owner.borrow(), body, at, points);
        check(result.status, "add convex");
        result.out_object;
      case Mesh(vertices, indices):
        var result = KinematicsKitNative.kk_collision_add_mesh(owner.borrow(), body, at, vertices, indices);
        check(result.status, "add mesh");
        result.out_object;
      case HeightField(xSize, ySize, heights, rows, minHeight):
        var result = KinematicsKitNative.kk_collision_add_height_field(owner.borrow(), body, at, xSize, ySize, heights,
          rows, minHeight);
        check(result.status, "add height field");
        result.out_object;
    }
  }

  public function setHeights(object:Int, heights:Array<Float>):Void {
    requireLive();
    check(KinematicsKitNative.kk_collision_set_heights(owner.borrow(), object, heights), "set_heights");
  }

  public function attach(object:Int, body:Int, offset:Transform):Void {
    requireLive();
    check(KinematicsKitNative.kk_collision_attach(owner.borrow(), object, body, flat(offset)), "attach");
  }

  public function remove(object:Int):Void {
    requireLive();
    check(KinematicsKitNative.kk_collision_remove(owner.borrow(), object), "remove");
  }

  public function setPairRule(a:Int, b:Int, rule:CollisionPairRule):Void {
    requireLive();
    check(KinematicsKitNative.kk_collision_set_pair_rule(owner.borrow(), a, b, rule), "set_pair_rule");
  }

  public function pairStatus(a:Int, b:Int):CollisionPairStatus {
    requireLive();
    var result = KinematicsKitNative.kk_collision_pair_status(owner.borrow(), a, b);
    check(result.status, "pair_status");
    return switch result.out_status {
      case 0: Checked;
      case 1: Static;
      case 2: Rigid;
      case 3: Adjacent;
      case 4: OverlapsAtReference;
      case 5: Allowed;
      case 6: Unsupported;
      case other: throw 'Unknown collision pair status $other';
    }
  }

  public function allowOverlapping(state:KinematicState):Int {
    requireLive();
    var result = KinematicsKitNative.kk_collision_allow_overlapping(owner.borrow(), state.q, rootPoses(state));
    check(result.status, "allow_overlapping");
    return result.out_count;
  }

  public function update(state:KinematicState):Void {
    requireLive();
    check(KinematicsKitNative.kk_collision_update(owner.borrow(), state.q, rootPoses(state)), "update");
  }

  public function colliding(margin:Float):Array<CollisionPair> {
    requireLive();
    while (true) {
      var result = KinematicsKitNative.kk_collision_check(owner.borrow(), margin, 2 * capacity);
      check(result.status, "check");
      var count:Int = result.out_count;
      if (count > capacity) {
        capacity = count;
        continue;
      }
      return [for (i in 0...count) new CollisionPair(result.out_pairs[2 * i], result.out_pairs[2 * i + 1])];
    }
  }

  public function distances(queryDistance:Float):Array<CollisionDistance> {
    requireLive();
    while (true) {
      var result = KinematicsKitNative.kk_collision_distances(owner.borrow(), queryDistance, 2 * capacity,
        10 * capacity);
      check(result.status, "distances");
      var count:Int = result.out_count;
      if (count > capacity) {
        capacity = count;
        continue;
      }
      var rows = result.out_results;
      return [
        for (i in 0...count) {
          var r = 10 * i;
          new CollisionDistance(result.out_pairs[2 * i], result.out_pairs[2 * i + 1], rows[r],
            new Vector3(rows[r + 1], rows[r + 2], rows[r + 3]), new Vector3(rows[r + 4], rows[r + 5], rows[r + 6]),
            new Vector3(rows[r + 7], rows[r + 8], rows[r + 9]));
        }
      ];
    }
  }

  public function dispose():Void {
    if (disposed) return;
    disposed = true;
    owner.close();
  }

  /** Every body's root pose in `state` (seven floats each; non-roots are ignored natively). */
  function rootPoses(state:KinematicState):Array<Float> {
    if (state.model != model) throw "Collision world state must be of its model";
    var poses:Array<Float> = [];
    for (body in 0...model.bodyCount()) {
      var pose = model.bodyParentJoint[body] < 0 ? state.rootPose(body) : Transform.identity();
      poses.push(pose.x); poses.push(pose.y); poses.push(pose.z);
      poses.push(pose.qx); poses.push(pose.qy); poses.push(pose.qz); poses.push(pose.qw);
    }
    return poses;
  }

  static function flat(pose:Transform):Array<Float> {
    var checked = Transform.checked(pose, "Collision offset");
    return [checked.x, checked.y, checked.z, checked.qx, checked.qy, checked.qz, checked.qw];
  }

  function requireLive():Void {
    if (disposed) throw "Collision world has been disposed";
  }

  static function check(status:Int, operation:String):Void {
    if (status == KinematicsKitNativeConstants.KK_ERROR_UNSUPPORTED)
      throw 'Collision $operation failed: a checked pair has shapes that cannot be compared (allow it or remove one)';
    if (status != KinematicsKitNativeConstants.KK_OK) throw 'Collision $operation failed with error $status';
  }
}
