package kinematicskit;

/**
 * Collects bodies, joints, couplings, frames and closures, then compiles
 * them into an immutable `KinematicModel`. Indices returned by the `add*`
 * methods are the compiled indices.
 */
class KinematicModelBuilder {
  final bodyIds:Array<String> = [];
  final bodyRootPoses:Array<Null<Transform>> = [];
  final joints:Array<JointSpec> = [];
  final couplings:Array<CouplingSpec> = [];
  final frames:Array<FrameSpec> = [];
  final closures:Array<ClosureSpec> = [];
  final bodyIndex = new Map<String, Int>();
  final jointIndex = new Map<String, Int>();
  final frameIndex = new Map<String, Int>();
  final closureIndex = new Map<String, Int>();

  public function new() {}

  public function addBody(id:String):Int {
    requireId(id, "Kinematic body");
    if (bodyIndex.exists(id)) throw 'Kinematic body "$id" is declared more than once';
    bodyIndex.set(id, bodyIds.length);
    bodyIds.push(id);
    bodyRootPoses.push(null);
    return bodyIds.length - 1;
  }

  /** The pose a root body takes when the state does not override it (identity by default). */
  public function setRootPose(body:Int, pose:Transform):Void {
    requireBody(body);
    bodyRootPoses[body] = Transform.checked(pose, 'Root pose of body "${bodyIds[body]}"');
  }

  /**
   * A tree joint: `child = parent · parentTJoint · motion(axis, value) · jointTChild`.
   * `axis` is expressed in the joint frame and normalized here. Limits default
   * to unbounded; `continuous` marks a revolute joint whose value may wrap.
   */
  public function addJoint(id:String, kind:JointKind, parent:Int, child:Int, parentTJoint:Transform,
      jointTChild:Transform, ?axis:Vector3, ?lower:Float, ?upper:Float, ?defaultValue:Float = 0.0,
      ?continuous:Bool = false):Int {
    requireId(id, "Kinematic joint");
    if (jointIndex.exists(id)) throw 'Kinematic joint "$id" is declared more than once';
    requireBody(parent);
    requireBody(child);
    if (parent == child) throw 'Kinematic joint "$id" connects body "${bodyIds[child]}" to itself';
    var ax = 0.0, ay = 0.0, az = 1.0;
    if (kind != JointKind.Fixed) {
      if (axis == null) throw 'Kinematic joint "$id" needs a motion axis';
      var norm = Math.sqrt(axis.x * axis.x + axis.y * axis.y + axis.z * axis.z);
      if (!Math.isFinite(norm) || norm < 1e-9) throw 'Kinematic joint "$id" has a zero or non-finite axis';
      ax = axis.x; ay = axis.y; az = axis.z;
      if (norm != 1.0) { ax /= norm; ay /= norm; az /= norm; }
    }
    var lo = lower == null ? Math.NEGATIVE_INFINITY : lower;
    var hi = upper == null ? Math.POSITIVE_INFINITY : upper;
    if (Math.isNaN(lo) || Math.isNaN(hi) || lo > hi) throw 'Kinematic joint "$id" has invalid limits';
    if (!Math.isFinite(defaultValue)) throw 'Kinematic joint "$id" default value must be finite';
    jointIndex.set(id, joints.length);
    joints.push(new JointSpec(id, kind, parent, child,
      Transform.checked(parentTJoint, 'Kinematic joint "$id" parent frame'),
      Transform.checked(jointTChild, 'Kinematic joint "$id" child frame'),
      ax, ay, az, lo, hi, defaultValue, continuous && kind == JointKind.Revolute));
    return joints.length - 1;
  }

  /** Drives `target`'s value as `ratio * value(source) + offset`; the target gets no DOF of its own. */
  public function couple(target:String, source:String, ratio:Float, offset:Float):Void {
    if (!Math.isFinite(ratio) || ratio == 0.0 || !Math.isFinite(offset))
      throw 'Kinematic coupling of "$target" needs a finite non-zero ratio and finite offset';
    couplings.push(new CouplingSpec(target, source, ratio, offset));
  }

  public function addFrame(id:String, body:Int, bodyTFrame:Transform):Int {
    requireId(id, "Kinematic frame");
    if (frameIndex.exists(id)) throw 'Kinematic frame "$id" is declared more than once';
    requireBody(body);
    frameIndex.set(id, frames.length);
    frames.push(new FrameSpec(id, body, Transform.checked(bodyTFrame, 'Kinematic frame "$id"')));
    return frames.length - 1;
  }

  /**
   * A loop closure between two frames. Closures are never tree edges: forward
   * kinematics ignores them, and solvers treat them as equality tasks. `axis`
   * is expressed in both frames (Revolute/Prismatic); `tolerance` is the
   * authored position tolerance, if any.
   */
  public function addClosure(id:String, kind:ClosureKind, frameA:Int, frameB:Int, ?axis:Vector3,
      ?tolerance:Float):Int {
    requireId(id, "Kinematic closure");
    if (closureIndex.exists(id)) throw 'Kinematic closure "$id" is declared more than once';
    if (frameA < 0 || frameA >= frames.length || frameB < 0 || frameB >= frames.length)
      throw 'Kinematic closure "$id" references an unknown frame';
    var ax = 0.0, ay = 0.0, az = 1.0;
    if (axis != null) {
      var norm = Math.sqrt(axis.x * axis.x + axis.y * axis.y + axis.z * axis.z);
      if (!Math.isFinite(norm) || norm < 1e-9) throw 'Kinematic closure "$id" has a zero or non-finite axis';
      ax = axis.x / norm; ay = axis.y / norm; az = axis.z / norm;
    } else if (kind != ClosureKind.Fixed) throw 'Kinematic closure "$id" needs an axis';
    if (tolerance != null && (!Math.isFinite(tolerance) || tolerance <= 0.0))
      throw 'Kinematic closure "$id" tolerance must be finite and positive';
    closureIndex.set(id, closures.length);
    closures.push(new ClosureSpec(id, kind, frameA, frameB, ax, ay, az, tolerance));
    return closures.length - 1;
  }

  public function build():KinematicModel {
    var bodyCount = bodyIds.length;
    var jointCount = joints.length;

    // Tree structure: at most one incoming joint per body, no cycles.
    var parentJoint = [for (_ in 0...bodyCount) -1];
    for (index in 0...jointCount) {
      var joint = joints[index];
      if (parentJoint[joint.child] >= 0)
        throw 'Kinematic body "${bodyIds[joint.child]}" has two parent joints ' +
          '("${joints[parentJoint[joint.child]].id}" and "${joint.id}"); make one of them a closure';
      parentJoint[joint.child] = index;
    }
    var jointOrder:Array<Int> = [];
    var bodyOrder:Array<Int> = [];
    var children:Array<Array<Int>> = [for (_ in 0...bodyCount) []];
    for (index in 0...jointCount) children[joints[index].parent].push(index);
    for (body in 0...bodyCount) if (parentJoint[body] < 0) {
      var stack = [body];
      while (stack.length > 0) {
        var current = stack.shift();
        bodyOrder.push(current);
        for (joint in children[current]) {
          jointOrder.push(joint);
          stack.push(joints[joint].child);
        }
      }
    }
    if (bodyOrder.length != bodyCount) {
      for (body in 0...bodyCount) if (bodyOrder.indexOf(body) < 0)
        throw 'Kinematic tree contains a cycle through body "${bodyIds[body]}"';
    }

    // Couplings: resolve each coupled joint to the DOF that ultimately drives it.
    var couplingOf = new Map<Int, CouplingSpec>();
    for (coupling in couplings) {
      var target = jointIndex.get(coupling.target);
      var source = jointIndex.get(coupling.source);
      if (target == null || source == null)
        throw 'Kinematic coupling "${coupling.target}" <- "${coupling.source}" references an unknown joint';
      if (target == source) throw 'Kinematic joint "${coupling.target}" cannot drive itself';
      if (joints[target].kind == JointKind.Fixed || joints[source].kind == JointKind.Fixed)
        throw 'Kinematic coupling "${coupling.target}" <- "${coupling.source}" needs two movable joints';
      if (couplingOf.exists(target)) throw 'Kinematic joint "${coupling.target}" is driven by two couplings';
      couplingOf.set(target, coupling);
    }

    var jointDof = [for (_ in 0...jointCount) -1];
    var jointSource = [for (_ in 0...jointCount) -1];
    var jointRatio = [for (_ in 0...jointCount) 1.0];
    var jointOffset = [for (_ in 0...jointCount) 0.0];
    var jointScale = [for (_ in 0...jointCount) 0.0];
    var dofJoint:Array<Int> = [];
    for (index in 0...jointCount) {
      if (joints[index].kind == JointKind.Fixed || couplingOf.exists(index)) continue;
      jointDof[index] = dofJoint.length;
      jointScale[index] = 1.0;
      dofJoint.push(index);
    }
    // Values are computed in `valueOrder`: every source before its targets.
    var valueOrder:Array<Int> = [];
    var resolved = [for (_ in 0...jointCount) false];
    for (index in 0...jointCount) if (jointDof[index] >= 0 || joints[index].kind == JointKind.Fixed) {
      resolved[index] = true;
      valueOrder.push(index);
    }
    // Composite affine map from the driving DOF value, used for limits and Jacobian scale.
    var jointA = [for (index in 0...jointCount) jointDof[index] >= 0 ? 1.0 : 0.0];
    var jointB = [for (_ in 0...jointCount) 0.0];
    for (index in 0...jointCount) if (!resolved[index]) {
      var path:Array<Int> = [];
      var current = index;
      while (!resolved[current]) {
        if (path.indexOf(current) >= 0)
          throw 'Kinematic couplings form a cycle through joint "${joints[current].id}"';
        path.push(current);
        var link:CouplingSpec = couplingOf.get(current);
        var next = jointIndex.get(link.source);
        if (next == null) throw 'Kinematic coupling source "${link.source}" is not a joint';
        current = next;
      }
      var i = path.length - 1;
      while (i >= 0) {
        var target = path[i];
        var coupling:CouplingSpec = couplingOf.get(target);
        var source = jointIndex.get(coupling.source);
        if (source == null) throw 'Kinematic coupling source "${coupling.source}" is not a joint';
        jointSource[target] = source;
        jointRatio[target] = coupling.ratio;
        jointOffset[target] = coupling.offset;
        jointDof[target] = jointDof[source];
        jointA[target] = jointA[source] * coupling.ratio;
        jointB[target] = jointB[source] * coupling.ratio + coupling.offset;
        jointScale[target] = jointA[target];
        resolved[target] = true;
        valueOrder.push(target);
        i--;
      }
    }

    // DOF ranges: each driving joint's own limits, narrowed by every coupled joint's limits.
    var dofCount = dofJoint.length;
    var dofLower = [for (dof in 0...dofCount) joints[dofJoint[dof]].lower];
    var dofUpper = [for (dof in 0...dofCount) joints[dofJoint[dof]].upper];
    for (index in 0...jointCount) if (jointSource[index] >= 0) {
      var dof = jointDof[index];
      var a = jointA[index], b = jointB[index];
      var lo = (joints[index].lower - b) / a, hi = (joints[index].upper - b) / a;
      if (a < 0.0) { var swap = lo; lo = hi; hi = swap; }
      if (!Math.isNaN(lo) && lo > dofLower[dof]) dofLower[dof] = lo;
      if (!Math.isNaN(hi) && hi < dofUpper[dof]) dofUpper[dof] = hi;
      if (dofLower[dof] > dofUpper[dof])
        throw 'Coupled joint "${joints[index].id}" leaves joint "${joints[dofJoint[dof]].id}" no valid range';
    }

    // Movable joints on the path from each body's root, for Jacobians.
    var bodyChain:Array<Array<Int>> = [for (_ in 0...bodyCount) []];
    for (body in bodyOrder) {
      var joint = parentJoint[body];
      if (joint < 0) continue;
      var chain = bodyChain[joints[joint].parent].copy();
      if (joints[joint].kind != JointKind.Fixed) chain.push(joint);
      bodyChain[body] = chain;
    }

    var rootPoses:Array<Transform> = [];
    for (body in 0...bodyCount) rootPoses.push(bodyRootPoses[body] == null ? Transform.identity() : bodyRootPoses[body]);

    var parts = new KinematicModelParts();
    parts.bodyIds = bodyIds.copy();
    parts.bodyParentJoint = parentJoint;
    parts.bodyRootPoses = rootPoses;
    parts.bodyOrder = bodyOrder;
    parts.bodyChain = bodyChain;
    parts.jointIds = [for (joint in joints) joint.id];
    parts.jointKind = [for (joint in joints) joint.kind];
    parts.jointParent = [for (joint in joints) joint.parent];
    parts.jointChild = [for (joint in joints) joint.child];
    parts.jointParentTJoint = flatten([for (joint in joints) joint.parentTJoint]);
    parts.jointJointTChild = flatten([for (joint in joints) joint.jointTChild]);
    parts.jointAxis = axes();
    parts.jointOrder = jointOrder;
    parts.jointValueOrder = valueOrder;
    parts.jointDof = jointDof;
    parts.jointSource = jointSource;
    parts.jointRatio = jointRatio;
    parts.jointOffset = jointOffset;
    parts.jointScale = jointScale;
    parts.jointLower = [for (joint in joints) joint.lower];
    parts.jointUpper = [for (joint in joints) joint.upper];
    parts.jointDefault = [for (joint in joints) joint.defaultValue];
    parts.dofJoint = dofJoint;
    parts.dofLower = dofLower;
    parts.dofUpper = dofUpper;
    parts.frameIds = [for (frame in frames) frame.id];
    parts.frameBody = [for (frame in frames) frame.body];
    parts.frameOffset = flatten([for (frame in frames) frame.bodyTFrame]);
    parts.closureIds = [for (closure in closures) closure.id];
    parts.closureKind = [for (closure in closures) closure.kind];
    parts.closureFrameA = [for (closure in closures) closure.frameA];
    parts.closureFrameB = [for (closure in closures) closure.frameB];
    parts.closureAxis = closureAxes();
    parts.closureTolerance = [for (closure in closures) closure.tolerance];
    return new KinematicModel(parts);
  }

  function axes():Array<Float> {
    var result:Array<Float> = [];
    for (joint in joints) { result.push(joint.ax); result.push(joint.ay); result.push(joint.az); }
    return result;
  }

  function closureAxes():Array<Float> {
    var result:Array<Float> = [];
    for (closure in closures) { result.push(closure.ax); result.push(closure.ay); result.push(closure.az); }
    return result;
  }

  static function flatten(values:Array<Transform>):Array<Float> {
    var result:Array<Float> = [];
    for (value in values) {
      result.push(value.x); result.push(value.y); result.push(value.z);
      result.push(value.qx); result.push(value.qy); result.push(value.qz); result.push(value.qw);
    }
    return result;
  }

  function requireBody(body:Int):Void {
    if (body < 0 || body >= bodyIds.length) throw 'Kinematic body index $body is not part of the model';
  }

  static function requireId(id:String, what:String):Void {
    if (id == null || id.length == 0) throw '$what needs a non-empty ID';
  }
}

private class JointSpec {
  public final id:String;
  public final kind:JointKind;
  public final parent:Int;
  public final child:Int;
  public final parentTJoint:Transform;
  public final jointTChild:Transform;
  public final ax:Float;
  public final ay:Float;
  public final az:Float;
  public final lower:Float;
  public final upper:Float;
  public final defaultValue:Float;
  public final continuous:Bool;

  public function new(id:String, kind:JointKind, parent:Int, child:Int, parentTJoint:Transform,
      jointTChild:Transform, ax:Float, ay:Float, az:Float, lower:Float, upper:Float,
      defaultValue:Float, continuous:Bool) {
    this.id = id;
    this.kind = kind;
    this.parent = parent;
    this.child = child;
    this.parentTJoint = parentTJoint;
    this.jointTChild = jointTChild;
    this.ax = ax;
    this.ay = ay;
    this.az = az;
    this.lower = lower;
    this.upper = upper;
    this.defaultValue = defaultValue;
    this.continuous = continuous;
  }
}

private class CouplingSpec {
  public final target:String;
  public final source:String;
  public final ratio:Float;
  public final offset:Float;

  public function new(target:String, source:String, ratio:Float, offset:Float) {
    this.target = target;
    this.source = source;
    this.ratio = ratio;
    this.offset = offset;
  }
}

private class FrameSpec {
  public final id:String;
  public final body:Int;
  public final bodyTFrame:Transform;

  public function new(id:String, body:Int, bodyTFrame:Transform) {
    this.id = id;
    this.body = body;
    this.bodyTFrame = bodyTFrame;
  }
}

private class ClosureSpec {
  public final id:String;
  public final kind:ClosureKind;
  public final frameA:Int;
  public final frameB:Int;
  public final ax:Float;
  public final ay:Float;
  public final az:Float;
  public final tolerance:Null<Float>;

  public function new(id:String, kind:ClosureKind, frameA:Int, frameB:Int, ax:Float, ay:Float, az:Float,
      tolerance:Null<Float>) {
    this.id = id;
    this.kind = kind;
    this.frameA = frameA;
    this.frameB = frameB;
    this.ax = ax;
    this.ay = ay;
    this.az = az;
    this.tolerance = tolerance;
  }
}
