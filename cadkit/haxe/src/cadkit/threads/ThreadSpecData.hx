package cadkit.threads;

import cadkit.threads.ThreadSpec.ThreadHand;
import cadkit.threads.ThreadSpec.ThreadProfile;

/** Current saved nominal dimensions; face intervals are rebuilt from their feature. */
@:wire
typedef ThreadSpecData = {
	@:id(1) var designation:String;
	@:id(2) var majorDiameter:Float;
	@:id(3) var pitch:Float;
	@:id(4) var starts:Int;
	@:id(5) var hand:ThreadHand;
	@:id(6) var internal:Bool;
	@:id(7) var toleranceClass:String;
	@:id(8) var profile:ThreadProfile;
}
