package kinematicskit;

/** The arrays `KinematicModelBuilder` compiles, handed to `KinematicModel` in one piece. */
@:allow(kinematicskit.KinematicModelBuilder)
@:allow(kinematicskit.KinematicModel)
class KinematicModelParts {
  var bodyIds:Array<String>;
  var bodyParentJoint:Array<Int>;
  var bodyRootPoses:Array<Transform>;
  var bodyOrder:Array<Int>;
  var bodyChain:Array<Array<Int>>;
  var bodyRoot:Array<Int>;
  var jointIds:Array<String>;
  var jointKind:Array<JointKind>;
  var jointParent:Array<Int>;
  var jointChild:Array<Int>;
  var jointParentTJoint:Array<Float>;
  var jointJointTChild:Array<Float>;
  var jointAxis:Array<Float>;
  var jointOrder:Array<Int>;
  var jointValueOrder:Array<Int>;
  var jointDof:Array<Int>;
  var jointSource:Array<Int>;
  var jointRatio:Array<Float>;
  var jointOffset:Array<Float>;
  var jointScale:Array<Float>;
  var jointLower:Array<Float>;
  var jointUpper:Array<Float>;
  var jointDefault:Array<Float>;
  var dofJoint:Array<Int>;
  var dofLower:Array<Float>;
  var dofUpper:Array<Float>;
  var frameIds:Array<String>;
  var frameBody:Array<Int>;
  var frameOffset:Array<Float>;
  var closureIds:Array<String>;
  var closureKind:Array<ClosureKind>;
  var closureFrameA:Array<Int>;
  var closureFrameB:Array<Int>;
  var closureAxis:Array<Float>;
  var closureTolerance:Array<Null<Float>>;

  function new() {}
}
