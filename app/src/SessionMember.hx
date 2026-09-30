package app;

/**
 * Something that lives in the application simulation's session and follows its
 * lifecycle: it is fed before every tick, presented once per frame, and reset
 * with the session. The simulation drives every member the same way, so a new
 * kind of member needs no special case there.
 */
interface SessionMember {
	/** Runs before each tick, to supply what that tick will consume. */
	function feed():Void;

	/** The session has just been reset to its start; return to it too. */
	function reset():Void;

	/** Publishes the current state to the scene: once per presented frame, and after a reset. */
	function present():Void;
}
