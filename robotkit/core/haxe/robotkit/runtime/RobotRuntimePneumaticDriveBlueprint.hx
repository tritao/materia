package robotkit.runtime;

import RobotKitRuntime;

/** Compiled process binding for one double-acting cylinder. */
class RobotRuntimePneumaticDriveBlueprint {
  public final joint:Int;
  public final channelA:String;
  public final channelB:Null<String>;
  public final normallyToA:Bool;
  public final extensionForce:Float;
  public final retractionForce:Float;
  public final extendSign:Float;
  public final ratedSpeed:Float;

  public function new(joint:Int, channelA:String, channelB:Null<String>, normallyToA:Bool,
      extensionForce:Float, retractionForce:Float, extendSign:Float, ratedSpeed:Float) {
    this.joint = joint; this.channelA = channelA; this.channelB = channelB;
    this.normallyToA = normallyToA; this.extensionForce = extensionForce;
    this.retractionForce = retractionForce; this.extendSign = extendSign;
    this.ratedSpeed = ratedSpeed;
  }
}
