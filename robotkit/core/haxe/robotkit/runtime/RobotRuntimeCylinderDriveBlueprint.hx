package robotkit.runtime;

import robotkit.model.ActuatorDrive.CylinderDrive;

/** Rebuilt process binding for either pressure-effort or flow-limited fluid cylinders. */
class RobotRuntimeCylinderDriveBlueprint {
  public final joint:Int;
  public final control:CylinderControl;
  public final channelA:String;
  public final channelB:Null<String>;
  public final normallyToA:Bool;
  public final extensionForce:Float;
  public final retractionForce:Float;
  public final extendSign:Float;
  public final ratedSpeed:Float;
  public final stroke:Float;
  public final approachSpeed:Float;
  public final workingSpeed:Float;
  public final returnSpeed:Float;
  public final changePoint:Float;
  public final retractedPosition:Float;

  public function new(joint:Int, cylinder:CylinderDrive) {
    if (cylinder == null) throw "A cylinder process binding needs a cylinder drive";
    this.joint = joint; this.channelA = cylinder.channelA; this.channelB = cylinder.channelB;
    this.normallyToA = cylinder.normallyToA; this.extensionForce = cylinder.extensionForce();
    this.retractionForce = cylinder.retractionForce(); this.extendSign = cylinder.extendSign;
    this.ratedSpeed = cylinder.ratedSpeed; this.stroke = cylinder.stroke;
    var hydraulic = cylinder.hydraulic();
    this.control = hydraulic == null ? PressureEffort : FlowLimited;
    this.approachSpeed = hydraulic == null ? cylinder.ratedSpeed : hydraulic.approachSpeed;
    this.workingSpeed = hydraulic == null ? cylinder.ratedSpeed : hydraulic.workingSpeed;
    this.returnSpeed = cylinder.returningSpeed();
    this.changePoint = hydraulic == null ? cylinder.stroke : hydraulic.changePoint;
    this.retractedPosition = hydraulic == null ? 0.0 : hydraulic.retractedPosition;
  }
}
