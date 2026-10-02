package app;

import nativekit.sim.SimObject;
import robotkit.runtime.SimulatedSuctionTool;

/** A scene object the simulation owns, by scene id. */
typedef GripObject = {id:String, object:SimObject};

/**
 * The simulated tools of a session's robots, as a member of the session: they act after each step
 * on their own, so this only resets them with the session and reports what they hold.
 */
class SimulatedTools implements SessionMember {
	public final tools:Array<SimulatedSuctionTool> = [];
	final objects:Array<GripObject>;

	public function new(objects:Array<GripObject>) this.objects = objects;

	public function add(tool:SimulatedSuctionTool):SimulatedSuctionTool {
		tools.push(tool);
		return tool;
	}

	/** Scene ids of the objects the tools hold right now. */
	public function heldIds():Array<String> {
		var result:Array<String> = [];
		for (tool in tools) {
			var held = tool.held;
			if (held != null) for (entry in objects) if (entry.object == held) result.push(entry.id);
		}
		return result;
	}

	/** The session's free objects, by scene id. */
	public function objectsByScene():Array<GripObject> return objects;

	public function feed():Void {}

	public function reset():Void for (tool in tools) tool.reset();

	public function present():Void {}
}
