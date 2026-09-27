package motionkit.event;

/** Typed process output carried by a path or timed event. */
enum EventValue {
  Digital(value:Bool);
  Analog(value:Float);
  Process(command:String, argument:Float);
}
