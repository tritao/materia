package robotkit.manipulation;

import robotkit.model.RobotModel;
import robotkit.model.Link;
import robotkit.model.LinkId;
import robotkit.model.Frame;
import robotkit.model.FrameId;
import robotkit.model.Joint;
import robotkit.model.JointId;
import robotkit.model.JointType;
import robotkit.spatial.Vec3;
import robotkit.spatial.Quat;
import robotkit.spatial.Transform3;

/**
 * A serial kinematic chain from a base link to a tip (a link's own origin,
 * or a mounted Frame), built by walking `Joint`s exactly as
 * `RobotFrameTree2`/the native runtime interpret them (see
 * ARCHITECTURE.md, "Joint-frame convention"). Revolute, continuous, and
 * prismatic joints become degrees of freedom; fixed joints fold into
 * constant transforms; any other joint type is a construction error.
 */
class KinematicChain {
  public final baseLink:LinkId;
  public final tipLink:LinkId;
  public final tipFrame:Null<FrameId>;

  final steps:Array<ChainStep> = [];
  final dofJoints:Array<Joint> = [];
  final tipOffset:Transform3;

  public function new(model:RobotModel, baseLinkId:LinkId, tip:ChainTip) {
    if (model == null) throw "Kinematic chain requires a robot model";
    var base = findLink(model, baseLinkId);
    if (base == null) throw 'Kinematic chain base link "$baseLinkId" is not part of the model';
    this.baseLink = baseLinkId;

    var tipLinkResolved:Link;
    switch tip {
      case ChainTip.Link(id):
        var link = findLink(model, id);
        if (link == null) throw 'Kinematic chain tip link "$id" is not part of the model';
        tipLinkResolved = link;
        this.tipFrame = null;
        this.tipOffset = Transform3.identity();
      case ChainTip.Frame(id):
        var frame:Null<Frame> = null;
        for (candidate in model.frames) if (candidate != null && candidate.id == id) { frame = candidate; break; }
        if (frame == null || frame.link == null)
          throw 'Kinematic chain tip frame "$id" is not part of the model';
        tipLinkResolved = frame.link;
        this.tipFrame = id;
        this.tipOffset = Transform3.fromArrays(frame.position, frame.rotation);
    }
    this.tipLink = tipLinkResolved.id;

    var chainJoints:Array<Joint> = [];
    var current = tipLinkResolved;
    var guard = 0;
    while (current.id != base.id) {
      if (guard++ > model.links.length)
        throw "Kinematic chain walk did not terminate; check the model topology";
      var incoming:Null<Joint> = null;
      for (joint in model.joints) if (joint != null && joint.child != null && joint.child.id == current.id) {
        incoming = joint;
        break;
      }
      if (incoming == null)
        throw 'Kinematic chain has no path from base link "$baseLinkId" to link "${current.id}"';
      chainJoints.push(incoming);
      current = incoming.parent;
    }
    chainJoints.reverse();

    for (joint in chainJoints) {
      var parentTJointFrame = Transform3.fromArrays(joint.parentFramePosition, joint.parentFrameRotation);
      var jointFrameTChild = Transform3.fromArrays(joint.childFramePosition, joint.childFrameRotation).inverse();
      switch joint.type {
        case JointType.Fixed:
          steps.push(new ChainStep(joint, false, -1, parentTJointFrame, jointFrameTChild, Vec3.zero()));
        case JointType.Revolute, JointType.Continuous, JointType.Prismatic:
          var axis = unitAxis(joint);
          steps.push(new ChainStep(joint, true, dofJoints.length, parentTJointFrame, jointFrameTChild, axis));
          dofJoints.push(joint);
        case other:
          throw 'Joint "${joint.id}" has unsupported kinematic chain type "$other"';
      }
    }
  }

  public function dofCount():Int return dofJoints.length;

  public function dofJointIds():Array<JointId> return [for (joint in dofJoints) joint.id];

  public function dofJointsCopy():Array<Joint> return dofJoints.copy();

  public function forwardKinematics(q:Array<Float>):Transform3 return evaluate(q).tip;

  /** `base_T_link` for the base link and every link reached while walking to the tip (base link first). */
  public function allLinkTransforms(q:Array<Float>):Array<Transform3> return evaluate(q).links;

  /** Geometric Jacobian, 6xn (rows 0..2 linear, rows 3..5 angular), expressed in the base frame. */
  public function jacobian(q:Array<Float>):Array<Array<Float>> {
    var evaluated = evaluate(q);
    var tipPosition = evaluated.tip.translation;
    var rows:Array<Array<Float>> = [];
    for (_ in 0...6) rows.push([]);
    for (index in 0...dofJoints.length) {
      var origin = evaluated.dofOrigins[index];
      var axis = evaluated.dofAxes[index];
      var isPrismatic = dofJoints[index].type == JointType.Prismatic;
      var linear = isPrismatic ? axis : axis.cross(tipPosition.sub(origin));
      var angular = isPrismatic ? Vec3.zero() : axis;
      rows[0].push(linear.x); rows[1].push(linear.y); rows[2].push(linear.z);
      rows[3].push(angular.x); rows[4].push(angular.y); rows[5].push(angular.z);
    }
    return rows;
  }

  function evaluate(q:Array<Float>):ChainEvaluation {
    if (q == null || q.length != dofJoints.length)
      throw 'Kinematic chain requires ${dofJoints.length} joint values, got ${q == null ? 0 : q.length}';
    var current = Transform3.identity();
    var links:Array<Transform3> = [current];
    var dofOrigins:Array<Vec3> = [];
    var dofAxes:Array<Vec3> = [];
    for (step in steps) {
      var jointFrameInBase = current.compose(step.parentTJointFrame);
      if (step.isMovable) {
        dofOrigins.push(jointFrameInBase.translation);
        dofAxes.push(jointFrameInBase.transformVector(step.axis));
        var motion = motionTransform(step.joint.type, step.axis, q[step.dofIndex]);
        current = jointFrameInBase.compose(motion).compose(step.jointFrameTChild);
      } else {
        current = jointFrameInBase.compose(step.jointFrameTChild);
      }
      links.push(current);
    }
    return new ChainEvaluation(current.compose(tipOffset), links, dofOrigins, dofAxes);
  }

  static function motionTransform(type:JointType, axis:Vec3, value:Float):Transform3 return switch type {
    case JointType.Revolute, JointType.Continuous:
      new Transform3(Vec3.zero(), Quat.fromAxisAngle(axis, value));
    case JointType.Prismatic:
      new Transform3(axis.scale(value), Quat.identity());
    case other:
      throw 'Unsupported joint motion type "$other"';
  };

  static function unitAxis(joint:Joint):Vec3 {
    if (joint.axis == null || joint.axis.length != 3)
      throw 'Joint "${joint.id}" has an invalid motion axis';
    var axis = Vec3.fromArray(joint.axis);
    var norm = axis.norm();
    if (!Math.isFinite(norm) || Math.abs(norm - 1.0) > 1e-6)
      throw 'Joint "${joint.id}" motion axis must be unit length';
    return axis;
  }

  static function findLink(model:RobotModel, id:LinkId):Null<Link> {
    for (link in model.links) if (link != null && link.id == id) return link;
    return null;
  }
}

private class ChainStep {
  public final joint:Joint;
  public final isMovable:Bool;
  public final dofIndex:Int;
  public final parentTJointFrame:Transform3;
  public final jointFrameTChild:Transform3;
  public final axis:Vec3;

  public function new(joint:Joint, isMovable:Bool, dofIndex:Int,
      parentTJointFrame:Transform3, jointFrameTChild:Transform3, axis:Vec3) {
    this.joint = joint;
    this.isMovable = isMovable;
    this.dofIndex = dofIndex;
    this.parentTJointFrame = parentTJointFrame;
    this.jointFrameTChild = jointFrameTChild;
    this.axis = axis;
  }
}

private class ChainEvaluation {
  public final tip:Transform3;
  public final links:Array<Transform3>;
  public final dofOrigins:Array<Vec3>;
  public final dofAxes:Array<Vec3>;

  public function new(tip:Transform3, links:Array<Transform3>, dofOrigins:Array<Vec3>, dofAxes:Array<Vec3>) {
    this.tip = tip;
    this.links = links;
    this.dofOrigins = dofOrigins;
    this.dofAxes = dofAxes;
  }
}
