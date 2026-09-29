package app;

import nativekit.sim.SimSession;

/**
 * Something that takes part in the application simulation beyond its robots
 * and editable scene, such as a person. Every rebuild builds a new session.
 */
interface SessionParticipant {
	/** Joins a newly built session while it is still stopped. */
	function join(session:SimSession):Void;

	/** Leaves the current session, which is about to be released. */
	function leave():Void;
}
