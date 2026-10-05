package robotkit.model;

/** Current RobotModel hydraulic drive payload, in metres, seconds, Pa and N. */
@:wire typedef HydraulicDriveRecord = {
  @:id(1) var bore:Float;
  @:id(2) var rod:Float;
  @:id(3) var stroke:Float;
  @:id(4) var pressurePa:Float;
  @:id(5) var approachSpeed:Float;
  @:id(6) var workingSpeed:Float;
  @:id(7) var returnSpeed:Float;
  @:id(8) var changePoint:Float;
  @:id(9) var channelA:String;
  @:id(10) var channelB:String;
  @:id(11) var extendSign:Float;
  @:id(12) var retractedPosition:Float;
}
