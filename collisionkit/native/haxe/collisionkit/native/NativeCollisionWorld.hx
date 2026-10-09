package collisionkit.native;

import CollisionKitNative;
import collisionkit.CollisionDistance;
import collisionkit.CollisionGeometry;
import collisionkit.CollisionMargins;
import collisionkit.CollisionPair;
import collisionkit.CollisionPairRule;
import collisionkit.CollisionPairStatus;
import collisionkit.CollisionPose;
import collisionkit.CollisionViolation;
import collisionkit.CollisionWorld;

/**
 * A `CollisionWorld` on coal, through collisionkit's C ABI (`ck_*`, see
 * `collisionkit.h`). Owns the native handle; call `dispose` when done.
 */
class NativeCollisionWorld implements CollisionWorld {
  final owner:Ownedck_world_handle;
  var disposed = false;
  var bodies = 0;
  /** Room for this many pairs in the first query call; grown to the largest count seen. */
  var capacity = 32;
  static final none:Array<Float> = [];

  public function new() {
    var created = CollisionKitNative.ck_world_create();
    check(created.status, "world.create");
    owner = created.out_world;
  }

  public function addBody(group:Int):Int {
    requireLive();
    var result = CollisionKitNative.ck_add_body(owner.borrow(), group);
    check(result.status, "add_body");
    bodies++;
    return result.out_body;
  }

  public function bodyCount():Int return bodies;

  public function setBodyGroup(body:Int, group:Int):Void {
    requireLive();
    check(CollisionKitNative.ck_set_body_group(owner.borrow(), body, group), "set_body_group");
  }

  public function setBodyStatic(body:Int, fixed:Bool):Void {
    requireLive();
    check(CollisionKitNative.ck_set_body_static(owner.borrow(), body, fixed ? 1 : 0), "set_body_static");
  }

  public function setBodyPose(body:Int, pose:CollisionPose):Void {
    setBodyPoses(body, flat(pose));
  }

  public function setBodyPoses(first:Int, poses:Array<Float>):Void {
    requireLive();
    check(CollisionKitNative.ck_set_body_poses(owner.borrow(), first, poses), "set_body_poses");
  }

  public function add(body:Int, offset:CollisionPose, geometry:CollisionGeometry):Int {
    requireLive();
    var at = flat(offset);
    var shape = (kind:Int, params:Array<Float>) -> {
      var result = CollisionKitNative.ck_add_shape(owner.borrow(), body, at, kind, params);
      check(result.status, "add");
      return result.out_object;
    };
    return switch geometry {
      case Box(halfX, halfY, halfZ): shape(CollisionKitNativeConstants.CK_SHAPE_BOX, [halfX, halfY, halfZ]);
      case Sphere(radius): shape(CollisionKitNativeConstants.CK_SHAPE_SPHERE, [radius]);
      case Capsule(radius, halfLength): shape(CollisionKitNativeConstants.CK_SHAPE_CAPSULE, [radius, halfLength]);
      case Cylinder(radius, halfLength): shape(CollisionKitNativeConstants.CK_SHAPE_CYLINDER, [radius, halfLength]);
      case HalfSpace(nx, ny, nz, offsetAlong):
        shape(CollisionKitNativeConstants.CK_SHAPE_HALFSPACE, [nx, ny, nz, offsetAlong]);
      case Convex(points):
        var result = CollisionKitNative.ck_add_convex(owner.borrow(), body, at, points);
        check(result.status, "add convex");
        result.out_object;
      case Mesh(vertices, indices):
        var result = CollisionKitNative.ck_add_mesh(owner.borrow(), body, at, vertices, indices);
        check(result.status, "add mesh");
        result.out_object;
      case HeightField(xSize, ySize, heights, rows, minHeight):
        var result = CollisionKitNative.ck_add_height_field(owner.borrow(), body, at, xSize, ySize, heights, rows,
          minHeight);
        check(result.status, "add height field");
        result.out_object;
    }
  }

  public function setInflation(object:Int, radius:Float):Void {
    requireLive();
    var status = CollisionKitNative.ck_set_inflation(owner.borrow(), object, radius);
    if (status == CollisionKitNativeConstants.CK_ERROR_UNSUPPORTED)
      throw 'Collision object $object cannot be inflated (only primitives and convex sets can)';
    check(status, "set_inflation");
  }

  public function setHeights(object:Int, heights:Array<Float>):Void {
    requireLive();
    check(CollisionKitNative.ck_set_heights(owner.borrow(), object, heights), "set_heights");
  }

  public function attach(object:Int, body:Int, offset:CollisionPose):Void {
    requireLive();
    check(CollisionKitNative.ck_attach(owner.borrow(), object, body, flat(offset)), "attach");
  }

  public function remove(object:Int):Void {
    requireLive();
    check(CollisionKitNative.ck_remove(owner.borrow(), object), "remove");
  }

  public function setBodyRule(a:Int, b:Int, rule:CollisionPairRule, reason:CollisionPairStatus):Void {
    requireLive();
    check(CollisionKitNative.ck_set_body_rule(owner.borrow(), a, b, rule, reason), "set_body_rule");
  }

  public function setObjectRule(a:Int, b:Int, rule:CollisionPairRule, reason:CollisionPairStatus):Void {
    requireLive();
    check(CollisionKitNative.ck_set_object_rule(owner.borrow(), a, b, rule, reason), "set_object_rule");
  }

  public function pairStatus(a:Int, b:Int):CollisionPairStatus {
    requireLive();
    var result = CollisionKitNative.ck_pair_status(owner.borrow(), a, b);
    check(result.status, "pair_status");
    return switch result.out_status {
      case 0: Checked;
      case 1: Static;
      case 2: Rigid;
      case 3: Adjacent;
      case 4: OverlapsAtReference;
      case 5: Allowed;
      case 6: Unsupported;
      case 7: Closure;
      case 8: ProcessContact;
      case 9: DeclaredContact;
      case other: throw 'Unknown collision pair status $other';
    }
  }

  public function allowOverlapping(bodies:Array<Int>):Int {
    requireLive();
    var result = CollisionKitNative.ck_allow_overlapping(owner.borrow(), bodies);
    check(result.status, "allow_overlapping");
    return result.out_count;
  }

  public function colliding(margin:Float):Array<CollisionPair> {
    requireLive();
    while (true) {
      var result = CollisionKitNative.ck_check(owner.borrow(), margin, 4 * capacity);
      check(result.status, "check");
      var count:Int = result.out_count;
      if (count > capacity) {
        capacity = count;
        continue;
      }
      var ids = result.out_pairs;
      return [for (i in 0...count) new CollisionPair(ids[4 * i], ids[4 * i + 1], ids[4 * i + 2], ids[4 * i + 3])];
    }
  }

  public function distances(queryDistance:Float):Array<CollisionDistance> {
    requireLive();
    while (true) {
      var result = CollisionKitNative.ck_distances(owner.borrow(), queryDistance, 4 * capacity, 10 * capacity);
      check(result.status, "distances");
      var count:Int = result.out_count;
      if (count > capacity) {
        capacity = count;
        continue;
      }
      var ids = result.out_pairs, rows = result.out_results;
      return [
        for (i in 0...count)
          row(ids[4 * i], ids[4 * i + 1], ids[4 * i + 2], ids[4 * i + 3], rows, 10 * i)
      ];
    }
  }

  public function pairDistances(pairs:Array<CollisionPair>):Array<CollisionDistance> {
    requireLive();
    var ids:Array<Int> = [];
    for (pair in pairs) {
      ids.push(pair.a);
      ids.push(pair.b);
    }
    var result = CollisionKitNative.ck_pair_distances(owner.borrow(), ids, 10 * pairs.length);
    check(result.status, "pair_distances");
    var rows = result.out_results;
    return [for (i in 0...pairs.length) row(pairs[i].a, pairs[i].b, pairs[i].bodyA, pairs[i].bodyB, rows, 10 * i)];
  }

  public function violation(margins:CollisionMargins, ?inflation:Array<Float>):Null<CollisionViolation> {
    requireLive();
    var result = CollisionKitNative.ck_violation(owner.borrow(), margins.flat(), inflation == null ? none : inflation, 4,
      2);
    check(result.status, "violation");
    return result.out_found == 0 ? null : violationOf(result.out_pair, result.out_result, 0);
  }

  public function closest(margins:CollisionMargins):Null<CollisionViolation> {
    requireLive();
    var result = CollisionKitNative.ck_closest(owner.borrow(), margins.flat(), 4, 2);
    check(result.status, "closest");
    return result.out_found == 0 ? null : violationOf(result.out_pair, result.out_result, 0);
  }

  public function firstViolation(poses:Array<Float>, margins:CollisionMargins, ?inflation:Array<Float>):Null<CollisionViolation> {
    requireLive();
    var result = CollisionKitNative.ck_violation_batch(owner.borrow(), poses, inflation == null ? none : inflation,
      margins.flat(), 4, 2);
    check(result.status, "violation_batch");
    return result.out_set < 0 ? null : violationOf(result.out_pair, result.out_result, result.out_set);
  }

  public function dispose():Void {
    if (disposed) return;
    disposed = true;
    owner.close();
  }

  static function violationOf(ids:Array<Int>, values:Array<Float>, set:Int):CollisionViolation
    return new CollisionViolation(ids[0], ids[1], ids[2], ids[3], values[0], values[1], set);

  static function row(a:Int, b:Int, bodyA:Int, bodyB:Int, rows:Array<Float>, r:Int):CollisionDistance
    return new CollisionDistance(a, b, bodyA, bodyB, rows[r], [rows[r + 1], rows[r + 2], rows[r + 3]],
      [rows[r + 4], rows[r + 5], rows[r + 6]], [rows[r + 7], rows[r + 8], rows[r + 9]]);

  static function flat(pose:CollisionPose):Array<Float> {
    var out:Array<Float> = [];
    pose.writeTo(out);
    return out;
  }

  function requireLive():Void {
    if (disposed) throw "Collision world has been disposed";
  }

  static function check(status:Int, operation:String):Void {
    if (status == CollisionKitNativeConstants.CK_ERROR_UNSUPPORTED)
      throw 'Collision $operation failed: a checked pair has shapes that cannot be compared (allow it or remove one)';
    if (status != CollisionKitNativeConstants.CK_OK) throw 'Collision $operation failed with error $status';
  }
}
