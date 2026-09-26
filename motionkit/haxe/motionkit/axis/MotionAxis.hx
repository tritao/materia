package motionkit.axis;

/** Runtime-resolved logical axis over one or more RobotKit joints. */
class MotionAxis {
  public final id:String;
  public final jointIds:Array<String>;
  public final jointIndices:Array<Int>;
  public final lowerLimit:Float;
  public final upperLimit:Float;
  public final maxVelocity:Float;
  public final maxAcceleration:Float;
  public final homePosition:Float;
  final jointScales:Array<Float>;
  final jointOffsets:Array<Float>;

  public function new(blueprint:MotionAxisBlueprint, robotJointNames:Array<String>) {
    if (blueprint == null) throw "Motion axis blueprint is required";
    if (robotJointNames == null) throw "Robot joint names are required";
    var indices:Array<Int> = [];
    for (jointId in blueprint.jointIds) {
      var index = robotJointNames.indexOf(jointId);
      if (index < 0) throw 'Robot does not expose motion axis joint "$jointId"';
      if (indices.indexOf(index) >= 0)
        throw 'Motion axis "${blueprint.id}" maps one joint more than once';
      indices.push(index);
    }
    id = blueprint.id;
    jointIds = blueprint.jointIds.copy();
    jointIndices = indices;
    lowerLimit = blueprint.lowerLimit;
    upperLimit = blueprint.upperLimit;
    maxVelocity = blueprint.maxVelocity;
    maxAcceleration = blueprint.maxAcceleration;
    homePosition = blueprint.homePosition;
    jointScales = blueprint.jointScales.copy();
    jointOffsets = blueprint.jointOffsets.copy();
  }

  public function containsJoint(index:Int):Bool return jointIndices.indexOf(index) >= 0;

  public function logicalPosition(jointPositions:Array<Float>):Float {
    if (jointPositions == null) throw "Joint positions are required";
    var joint = jointIndices[0];
    return (jointPositions[joint] - jointOffsets[0]) / jointScales[0];
  }

  public function writeLogicalPosition(jointPositions:Array<Float>, logicalPosition:Float):Void {
    if (jointPositions == null) throw "Joint positions are required";
    for (i in 0...jointIndices.length)
      jointPositions[jointIndices[i]] = jointOffsets[i] + jointScales[i] * logicalPosition;
  }

  /** Writes a logical velocity or acceleration into all physical joints. */
  public function writeLogicalDelta(jointValues:Array<Float>, logicalValue:Float):Void {
    if (jointValues == null) throw "Joint values are required";
    for (i in 0...jointIndices.length)
      jointValues[jointIndices[i]] = jointScales[i] * logicalValue;
  }
}
