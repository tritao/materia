package machinekit.cache;

import sys.thread.Mutex;

/**
 * A value built on first use, exactly once, for any number of threads: the lazily built recipes, catalogues and axes that
 * classes keep in statics. The first caller builds it under the lock (later callers wait for it); after that it is read
 * without locking. A build that throws keeps nothing, and the next caller tries again. The value is shared by every caller,
 * so it must not be changed after it is built: finish it inside `make`.
 *
 * Declare it `static final cache = new Once<T>();` and read it with `cache.get(() -> build())`.
 */
class Once<T> {
  final lock:Mutex;
  var value:Null<T>;
  var ready:Bool;

  public function new() {
    lock = new Mutex();
    value = null;
    ready = false;
  }

  public function get(make:() -> T):T {
    // The value is stored before `ready`, so a thread that sees `ready` sees the value.
    if (ready) return value;
    lock.acquire();
    try {
      if (!ready) {
        value = make();
        ready = true;
      }
      lock.release();
    } catch (error:Dynamic) {
      lock.release();
      throw error;
    }
    return value;
  }
}
