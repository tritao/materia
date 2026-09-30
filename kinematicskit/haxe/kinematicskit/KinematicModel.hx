package kinematicskit;

/**
 * An immutable compiled kinematic forest. Bodies are joined by tree joints;
 * coupled joints share their source's degree of freedom (DOF); frames are
 * fixed offsets on bodies; closures are extra equality constraints that
 * forward kinematics never walks. Build one with `KinematicModelBuilder`.
 *
 * Transforms are stored flat, seven floats each (x, y, z, qx, qy, qz, qw).
 * The arrays are shared with snapshots and solvers and must not be modified.
 */
class KinematicModel {
  public final bodyIds:Array<String>;
  /** Incoming tree joint per body, or -1 for a root. */
  public final bodyParentJoint:Array<Int>;
  public final bodyRootPoses:Array<Transform>;
  /** Bodies with every parent before its children. */
  public final bodyOrder:Array<Int>;
  /** Movable joints from each body's root to the body, root first. */
  public final bodyChain:Array<Array<Int>>;

  public final jointIds:Array<String>;
  public final jointKind:Array<JointKind>;
  public final jointParent:Array<Int>;
  public final jointChild:Array<Int>;
  public final jointParentTJoint:Array<Float>;
  public final jointJointTChild:Array<Float>;
  /** Unit motion axis per joint in the joint frame, three floats each. */
  public final jointAxis:Array<Float>;
  /** Joints with every parent joint before its children. */
  public final jointOrder:Array<Int>;
  /** Joints with every coupling source before its targets. */
  public final jointValueOrder:Array<Int>;
  /** The DOF that drives each movable joint (directly or through couplings); -1 when fixed. */
  public final jointDof:Array<Int>;
  /** Coupling source joint, or -1 when the joint is fixed or drives its own DOF. */
  public final jointSource:Array<Int>;
  public final jointRatio:Array<Float>;
  public final jointOffset:Array<Float>;
  /** d(joint value) / d(DOF value): 1 for a driving joint, the ratio product for a coupled one. */
  public final jointScale:Array<Float>;
  public final jointLower:Array<Float>;
  public final jointUpper:Array<Float>;
  public final jointDefault:Array<Float>;

  /** The driving joint of each DOF, in authored joint order. */
  public final dofJoint:Array<Int>;
  /** DOF ranges after intersecting coupled joints' limits; infinite when unbounded. */
  public final dofLower:Array<Float>;
  public final dofUpper:Array<Float>;

  public final frameIds:Array<String>;
  public final frameBody:Array<Int>;
  public final frameOffset:Array<Float>;

  public final closureIds:Array<String>;
  public final closureKind:Array<ClosureKind>;
  public final closureFrameA:Array<Int>;
  public final closureFrameB:Array<Int>;
  public final closureAxis:Array<Float>;
  public final closureTolerance:Array<Null<Float>>;

  final bodyIndexById = new Map<String, Int>();
  final jointIndexById = new Map<String, Int>();
  final frameIndexById = new Map<String, Int>();
  final closureIndexById = new Map<String, Int>();

  @:allow(kinematicskit.KinematicModelBuilder)
  function new(parts:KinematicModelParts) {
    this.bodyIds = parts.bodyIds;
    this.bodyParentJoint = parts.bodyParentJoint;
    this.bodyRootPoses = parts.bodyRootPoses;
    this.bodyOrder = parts.bodyOrder;
    this.bodyChain = parts.bodyChain;
    this.jointIds = parts.jointIds;
    this.jointKind = parts.jointKind;
    this.jointParent = parts.jointParent;
    this.jointChild = parts.jointChild;
    this.jointParentTJoint = parts.jointParentTJoint;
    this.jointJointTChild = parts.jointJointTChild;
    this.jointAxis = parts.jointAxis;
    this.jointOrder = parts.jointOrder;
    this.jointValueOrder = parts.jointValueOrder;
    this.jointDof = parts.jointDof;
    this.jointSource = parts.jointSource;
    this.jointRatio = parts.jointRatio;
    this.jointOffset = parts.jointOffset;
    this.jointScale = parts.jointScale;
    this.jointLower = parts.jointLower;
    this.jointUpper = parts.jointUpper;
    this.jointDefault = parts.jointDefault;
    this.dofJoint = parts.dofJoint;
    this.dofLower = parts.dofLower;
    this.dofUpper = parts.dofUpper;
    this.frameIds = parts.frameIds;
    this.frameBody = parts.frameBody;
    this.frameOffset = parts.frameOffset;
    this.closureIds = parts.closureIds;
    this.closureKind = parts.closureKind;
    this.closureFrameA = parts.closureFrameA;
    this.closureFrameB = parts.closureFrameB;
    this.closureAxis = parts.closureAxis;
    this.closureTolerance = parts.closureTolerance;
    for (i in 0...bodyIds.length) bodyIndexById.set(bodyIds[i], i);
    for (i in 0...jointIds.length) jointIndexById.set(jointIds[i], i);
    for (i in 0...frameIds.length) frameIndexById.set(frameIds[i], i);
    for (i in 0...closureIds.length) closureIndexById.set(closureIds[i], i);
  }

  public function bodyCount():Int return bodyIds.length;
  public function jointCount():Int return jointIds.length;
  public function dofCount():Int return dofJoint.length;
  public function frameCount():Int return frameIds.length;
  public function closureCount():Int return closureIds.length;

  /** Compiled index of a body, joint, frame or closure ID; -1 when absent. */
  public function bodyIndex(id:String):Int return indexIn(bodyIndexById, id);
  public function jointIndex(id:String):Int return indexIn(jointIndexById, id);
  public function frameIndex(id:String):Int return indexIn(frameIndexById, id);
  public function closureIndex(id:String):Int return indexIn(closureIndexById, id);

  /** The DOF a joint ID drives directly; -1 when the joint is fixed, coupled, or absent. */
  public function dofIndex(jointId:String):Int {
    var joint = jointIndex(jointId);
    return joint >= 0 && jointSource[joint] < 0 ? jointDof[joint] : -1;
  }

  public function dofId(dof:Int):String return jointIds[dofJoint[dof]];

  /** True when the DOF is revolute (its value is an angle). */
  public function dofIsAngular(dof:Int):Bool return jointKind[dofJoint[dof]] == JointKind.Revolute;

  public function frameTransform(frame:Int):Transform return transformAt(frameOffset, frame);

  static function transformAt(values:Array<Float>, index:Int):Transform {
    var o = index * 7;
    return new Transform(values[o], values[o + 1], values[o + 2], values[o + 3], values[o + 4],
      values[o + 5], values[o + 6]);
  }

  static function indexIn(map:Map<String, Int>, id:String):Int {
    if (id == null) return -1;
    var index = map.get(id);
    return index == null ? -1 : index;
  }
}
