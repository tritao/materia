package robotkit.time;

import haxe.Int64;
import nativekit.ffi.NativeKit;

/** Endpoint-owned clock identity. A restarted time line is a new epoch, never an implicit mapping. */
class SourceClock {
  static var nextInstance:Int = 0;
  static final instanceMutex = new sys.thread.Mutex();
  final name:String;
  var epoch:Int = 0;
  var previous:Null<Int64> = null;

  public function new(domain:String) {
    if (domain == null || domain.length == 0 || domain == "unspecified")
      throw "Source clock needs an authoritative domain";
    // Clock IDs cross robotd/recording boundaries. Haxeon's Std.random uses an unseeded
    // process PRNG, so it cannot supply an owner token. Include owner time, PID and
    // a local ordinal; nanoseconds distinguish creations even at coarse wall-clock resolution.
    instanceMutex.acquire();
    var ordinal = ++nextInstance;
    instanceMutex.release();
    var owner = Sys.time() + "|" + Sys.getPid() + "|" +
      Int64.toStr(NativeKit.nk_time_now_ns()) + "|" + ordinal;
    name = domain + "." + haxe.crypto.Sha256.encode(owner);
  }

  public function id():String return name + "." + epoch;

  /** The owner explicitly reports a successful reset, even when subsequent timestamps repeat. */
  public function beginEpoch():Void { epoch++; previous = null; }

  /** Native observations are ordered by the runtime. A backward source timestamp starts a new epoch. */
  public function observe(timestampNs:Int64):Void {
    if (timestampNs < Int64.ofInt(0)) throw "Source clock timestamp must be nonnegative";
    if (previous != null && timestampNs < cast(previous, Int64)) beginEpoch();
    previous = timestampNs;
  }
}
