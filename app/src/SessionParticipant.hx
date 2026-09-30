package app;

import nativekit.sim.SimSession;

/**
 * A member that outlives any one session and joins each new one, such as a
 * person placed from outside the document. Every rebuild builds a new session.
 * Members built with a session, like the document's workers, need no join.
 */
interface SessionParticipant extends SessionMember {
	/** Joins a newly built session while it is still stopped. */
	function join(session:SimSession):Void;

	/** Leaves the current session, which is about to be released. */
	function leave():Void;
}
