package collisionkit.kinematics;

import collisionkit.CollisionBuild;
import collisionkit.CollisionDescription;
import collisionkit.CollisionPairRule;
import collisionkit.CollisionPairStatus;
import collisionkit.CollisionPose;
import collisionkit.CollisionWorld;
import collisionkit.CollisionDistance;
import kinematicskit.AvoidancePair;
import kinematicskit.BodyPairRelation;
import kinematicskit.Vector3;
import kinematicskit.KinematicBodyPairs;
import kinematicskit.KinematicModel;
import kinematicskit.KinematicSnapshot;
import kinematicskit.KinematicState;
import kinematicskit.Transform;

/**
 * A `KinematicModel`'s bodies in a `CollisionDescription` (CL-D1, CL-D3):
 * one described body per model body, the kit's rigid, adjacent and closure
 * pairs as rules, posed at a reference configuration and declared as one
 * articulation. Extra bodies can follow a model body rigidly (a tool in its
 * own group); they take that body's rules. `place` poses them all from a
 * snapshot of the model.
 */
class ModelBodies {
  public final model:KinematicModel;
  public final name:String;
  /** Described body of each model body. */
  public final bodies:Array<Int>;
  /** Described bodies that follow a model body, and the model body each follows. */
  public final followers:Array<Int> = [];
  public final followed:Array<Int> = [];
  public final reference:KinematicState;
  public final referenceName:String;
  final description:CollisionDescription;
  final snapshot:KinematicSnapshot;
  var articulation = -1;

  function new(description:CollisionDescription, model:KinematicModel, name:String, bodies:Array<Int>,
      reference:KinematicState, referenceName:String) {
    this.description = description;
    this.model = model;
    this.name = name;
    this.bodies = bodies;
    this.reference = reference;
    this.referenceName = referenceName;
    snapshot = new KinematicSnapshot(model);
  }

  /**
   * Adds `model`'s bodies to `description` in `group`, named
   * `name/bodyId`, posed at `reference` (called `referenceName`). With
   * `fixedRoots`, bodies rigid to a root of the model are fixed in the cell
   * (a robot bolted down); otherwise only the model's structure decides.
   */
  public static function describe(description:CollisionDescription, model:KinematicModel, name:String, group:Int,
      reference:KinematicState, referenceName:String, fixedRoots:Bool):ModelBodies {
    if (reference.model != model) throw 'Reference of "$name" must be a state of its model';
    var groups = KinematicBodyPairs.rigidGroups(model);
    var bodies = [for (body in 0...model.bodyCount())
      description.addBody('$name/${model.bodyIds[body]}', group, fixedRoots && model.bodyParentJoint[groups[body]] < 0)];
    var result = new ModelBodies(description, model, name, bodies, reference, referenceName);
    for (pair in KinematicBodyPairs.of(model))
      description.setBodyRule(bodies[pair.a], bodies[pair.b], CollisionPairRule.Allow, reasonOf(pair.relation));
    result.poseDescription();
    return result;
  }

  /**
   * Adds a body in `group` that moves rigidly with model body `body` (a tool
   * on its flange link): it is rigid with that body and takes its rules.
   */
  public function follow(body:Int, followerName:String, group:Int):Int {
    if (articulation >= 0) throw 'Bodies of "$name" are already declared as an articulation';
    var target = bodies[body];
    var follower = description.addBody('$name/$followerName', group, description.bodies[target].fixed,
      description.bodies[target].pose);
    description.setBodyRule(target, follower, CollisionPairRule.Allow, CollisionPairStatus.Rigid);
    for (rule in description.bodyRules.copy()) {
      if (rule.rule != CollisionPairRule.Allow) continue;
      if (rule.a == target && rule.b != follower) description.setBodyRule(follower, rule.b, rule.rule, rule.reason);
      else if (rule.b == target && rule.a != follower) description.setBodyRule(rule.a, follower, rule.rule, rule.reason);
    }
    followers.push(follower);
    followed.push(body);
    return follower;
  }

  /** Declares the bodies (and followers) as one articulation at the reference; call once all followers exist. */
  public function declare():Int {
    if (articulation < 0) articulation = description.addArticulation(name, bodies.concat(followers), referenceName);
    return articulation;
  }

  /** Every described body of this model, followers last. */
  public function all():Array<Int> return bodies.concat(followers);

  /** A state of the model at DOF values `q`, with the root poses of the reference (where the model sits in the cell). */
  public function state(q:Array<Float>):KinematicState {
    var state = new KinematicState(model, q);
    for (body in 0...model.bodyCount()) if (model.bodyParentJoint[body] < 0) state.setRootPose(body, reference.rootPose(body));
    return state;
  }

  /** Poses the built world's bodies of this model for `state` (its root poses included: see `state`). */
  public function place(build:CollisionBuild, state:KinematicState):Void {
    if (state.model != model) throw 'State must be of the model of "$name"';
    snapshot.evaluate(state);
    var poses:Array<Float> = [for (_ in 0...7) 0.0];
    for (body in 0...bodies.length) {
      snapshot.bodyPoseInto(body, poses, 0);
      build.world.setBodyPoses(build.bodies[bodies[body]], poses);
    }
    for (i in 0...followers.length) {
      snapshot.bodyPoseInto(followed[i], poses, 0);
      build.world.setBodyPoses(build.bodies[followers[i]], poses);
    }
  }

  /** Every body's pose for `state` (seven numbers each, in `all()` order): a pose set for batched checks. */
  public function poses(state:KinematicState):Array<Float> {
    snapshot.evaluate(state);
    var out:Array<Float> = [for (_ in 0...7 * (bodies.length + followers.length)) 0.0];
    for (body in 0...bodies.length) snapshot.bodyPoseInto(body, out, 7 * body);
    for (i in 0...followers.length) snapshot.bodyPoseInto(followed[i], out, 7 * (bodies.length + i));
    return out;
  }

  /**
   * Avoidance pairs for this model (COLLISION.md CL5) from a built world's
   * distances: each pair with a body of this model, as model bodies (a side
   * outside it, or a follower such as the tool, maps to its model body or
   * -1), with the pair's safety margin and influence distance.
   */
  public function avoidancePairs(build:CollisionBuild, found:Array<CollisionDistance>, safety:Float,
      influence:Float):Array<AvoidancePair> {
    function modelBody(worldBody:Int):Int {
      var described = build.bodyOf(worldBody);
      if (described < 0) return -1;
      var direct = bodies.indexOf(described);
      if (direct >= 0) return direct;
      var follower = followers.indexOf(described);
      return follower >= 0 ? followed[follower] : -1;
    }
    var pairs:Array<AvoidancePair> = [];
    for (d in found) {
      var a = modelBody(d.bodyA), b = modelBody(d.bodyB);
      if (a < 0 && b < 0) continue;
      pairs.push(new AvoidancePair(a, b, new Vector3(d.pointA[0], d.pointA[1], d.pointA[2]),
        new Vector3(d.pointB[0], d.pointB[1], d.pointB[2]), new Vector3(d.normal[0], d.normal[1], d.normal[2]), d.distance,
        safety, influence));
    }
    return pairs;
  }

  function poseDescription():Void {
    snapshot.evaluate(reference);
    for (body in 0...bodies.length) description.bodies[bodies[body]].pose = pose(snapshot.bodyPose(body));
  }

  public static function pose(t:Transform):CollisionPose return new CollisionPose(t.x, t.y, t.z, t.qx, t.qy, t.qz, t.qw);

  static function reasonOf(relation:BodyPairRelation):CollisionPairStatus {
    return switch relation {
      case BodyPairRelation.Rigid: CollisionPairStatus.Rigid;
      case BodyPairRelation.Adjacent: CollisionPairStatus.Adjacent;
      case _: CollisionPairStatus.Closure;
    }
  }
}
