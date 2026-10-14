package robotkit.tool;

import haxe.Int64;

/** One timestamped observation of the changer's physical lock and mounted tool. */
typedef ChangerLockObservation = {
	var locked:Bool;
	var toolId:Null<String>;
	var sourceClockId:String;
	var sourceTimestampNs:Int64;
}
