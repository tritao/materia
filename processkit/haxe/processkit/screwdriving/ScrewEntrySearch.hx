package processkit.screwdriving;

import robotkit.spatial.Transform3;

/** One bounded, measured entry attempt between the safe approach and tightening engage. */
interface ScrewEntrySearch {
	function start():Void;
	function update(screwHeld:Bool):Void;
	function running():Bool;
	function isComplete():Bool;
	function failureReason():Null<String>;
	function entryPose():Null<Transform3>;
	function cancel():Void;
}
