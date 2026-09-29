package robotkit.tool;

import haxe.Int64;

/** Lock actuation for an automatic tool changer. */
interface ChangerLock {
	public function lock(timestampNs:Int64):Void;
	public function unlock(timestampNs:Int64):Void;
	public function isLocked():Bool;
}
