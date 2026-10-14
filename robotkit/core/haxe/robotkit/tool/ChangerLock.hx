package robotkit.tool;

import haxe.Int64;

/** Lock actuation for an automatic tool changer. */
interface ChangerLock {
	public function lock(timestampNs:Int64):Void;
	public function unlock(timestampNs:Int64):Void;
	/**
	 * Return fresh measured lock and tool-identity feedback. A physical adapter
	 * must throw when either observation is unavailable or stale; it must not
	 * report the last output command or requested tool as physical confirmation.
	 */
	public function observation():ChangerLockObservation;
}
