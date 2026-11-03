package robotkit.execution;



/** Runtime output value, independent of the MotionKit authoring package. */
enum ProcessEventValue : Equatable {
  Digital(value:Bool);
  Analog(value:Float);
  Process(command:String, argument:Float);
}
