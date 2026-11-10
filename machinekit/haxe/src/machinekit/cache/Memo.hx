package machinekit.cache;

import sys.thread.Mutex;

/**
 * One build-time memo: a bounded, thread-safe map from a stable text key to a value computed from nothing but what the key
 * names. Every MachineKit cache of derived results (sizing layouts, anchor derivations, realized bodies, imports) is one of
 * these, so they share one set of rules:
 *
 * - **Keys** are stable text built from the inputs (a record's codec text, a recipe's key, a pose's JSON), never from object
 *   identity or anything that varies between runs; a null key means "not cacheable" and the value is computed every time.
 * - **Values** are either plain immutable data handed to every caller as is (the default), or are copied out for each caller by
 *   the memo's `copyOut` (data a caller may change, native shapes). What a memo keeps is never handed out itself.
 * - **Native shapes** are always copied out as independent topology (`Memos.parts`, `Memos.shapes`): OCCT writes into a shape's
 *   topology after it is built (tessellation stores triangulations on its faces and edges, some algorithms raise
 *   tolerances), and a handle clone shares that topology. A deep copy shares only the geometry, which nothing writes, so a
 *   caller can mesh, measure or cut its copy on any thread while another thread does the same to its own. It also keeps
 *   results independent of history: a later caller never sees a triangulation an earlier one left behind.
 * - **Bounded**: at most `capacity` entries; the least recently used one is dropped (and released, for native values) past it.
 * - **Thread-safe**: the table is guarded by a mutex held only for lookups and inserts, never while a value is computed, so
 *   computations may use other memos (or this one) and run concurrently. Two threads missing on one key both compute it; the
 *   first stored value is kept and the other released, which is the same value since it depends only on the key.
 * - **Static**: memos are `static final` fields, built during static initialisation (single-threaded, at start-up), which is
 *   when they join the registry (`Memos`) that reports and clears them all.
 *
 * Lazily built single values (a recipe, a catalogue, an axis) use `Once` instead.
 */
class Memo<V> implements MemoTable {
  public final name:String;
  final capacity:Int;
  final copyOut:V -> V;
  final release:Null<V -> Void>;
  final lock:Mutex;
  final index:Map<String, MemoEntry<V>>;
  /** Most recently used first; `oldest` is evicted first. */
  var newest:Null<MemoEntry<V>>;
  var oldest:Null<MemoEntry<V>>;
  var size:Int;
  var hitCount:Int;
  var missCount:Int;
  var evictionCount:Int;
  var computeSeconds:Float;

  /**
   * `copyOut` gives each caller its own value (identity when omitted: the value must then never be changed by anyone);
   * `release` frees a value the memo drops (native handles).
   */
  public function new(name:String, capacity:Int, ?copyOut:V -> V, ?release:V -> Void) {
    if (name == null || name.length == 0 || capacity < 1) throw "Memo requires a name and a positive capacity";
    this.name = name;
    this.capacity = capacity;
    this.copyOut = copyOut == null ? value -> value : copyOut;
    this.release = release;
    lock = new Mutex();
    index = new Map();
    newest = null;
    oldest = null;
    size = 0;
    hitCount = 0;
    missCount = 0;
    evictionCount = 0;
    computeSeconds = 0;
    Memos.register(this);
  }

  /** The value for `key`, computing it with `compute` when it is not kept (and keeping it). */
  public function get(key:Null<String>, compute:() -> V):V {
    if (key == null) return compute();
    var found = lookup(key);
    if (found != null) return found.value;
    var began = Sys.time();
    var value = compute();
    var seconds = Sys.time() - began;
    return keep(key, value, seconds);
  }

  /** Whether `key` is kept (a hit is not counted). */
  public function contains(key:String):Bool return lookup(key, false) != null;

  function lookup(key:String, count:Bool = true):Null<MemoFound<V>> {
    lock.acquire();
    try {
      var entry = index.get(key);
      var found:Null<MemoFound<V>> = null;
      if (entry != null) {
        if (count) hitCount++;
        touch(entry);
        found = new MemoFound<V>(count ? copyOut(entry.value) : entry.value);
      } else if (count) missCount++;
      lock.release();
      return found;
    } catch (error:Dynamic) {
      lock.release();
      throw error;
    }
  }

  function keep(key:String, value:V, seconds:Float):V {
    var dropped:Array<V> = [];
    var result:V;
    lock.acquire();
    try {
      computeSeconds += seconds;
      var raced = index.get(key);
      if (raced != null) {
        // Another thread kept this key while this one computed it: keep the first, release this one.
        touch(raced);
        result = copyOut(raced.value);
        dropped.push(value);
      } else {
        var entry = new MemoEntry<V>(key, value);
        index.set(key, entry);
        link(entry);
        size++;
        while (size > capacity) {
          var last = oldest;
          if (last == null) break;
          unlink(last);
          index.remove(last.key);
          size--;
          evictionCount++;
          dropped.push(last.value);
        }
        result = copyOut(value);
      }
      lock.release();
    } catch (error:Dynamic) {
      lock.release();
      throw error;
    }
    if (release != null) for (value in dropped) release(value);
    return result;
  }

  /** Drops (and releases) every kept value and zeroes the counters: for tests. */
  public function clear():Void {
    var dropped:Array<V> = [];
    lock.acquire();
    for (entry in index) dropped.push(entry.value);
    index.clear();
    newest = null;
    oldest = null;
    size = 0;
    hitCount = 0;
    missCount = 0;
    evictionCount = 0;
    computeSeconds = 0;
    lock.release();
    if (release != null) for (value in dropped) release(value);
  }

  public function stats():MemoStats {
    lock.acquire();
    var result:MemoStats = {
      name: name,
      size: size,
      capacity: capacity,
      hits: hitCount,
      misses: missCount,
      evictions: evictionCount,
      computeSeconds: computeSeconds
    };
    lock.release();
    return result;
  }

  public function tableName():String return name;

  function touch(entry:MemoEntry<V>):Void {
    if (newest == entry) return;
    unlink(entry);
    link(entry);
  }

  function link(entry:MemoEntry<V>):Void {
    entry.older = newest;
    entry.newer = null;
    var head = newest;
    if (head != null) head.newer = entry;
    newest = entry;
    if (oldest == null) oldest = entry;
  }

  function unlink(entry:MemoEntry<V>):Void {
    var newer = entry.newer, older = entry.older;
    if (newer != null) newer.older = older else newest = older;
    if (older != null) older.newer = newer else oldest = newer;
    entry.newer = null;
    entry.older = null;
  }
}

/** A kept value, found: a null value is still a kept value. */
private class MemoFound<V> {
  public final value:V;

  public function new(value:V) this.value = value;
}

typedef MemoStats = {
  var name:String;
  var size:Int;
  var capacity:Int;
  var hits:Int;
  var misses:Int;
  var evictions:Int;
  /** Time spent computing the values that missed, s (wall, summed over threads). */
  var computeSeconds:Float;
}

/** What the registry knows of a memo. */
interface MemoTable {
  function tableName():String;
  function stats():MemoStats;
  function clear():Void;
}

private class MemoEntry<V> {
  public final key:String;
  public final value:V;
  public var newer:Null<MemoEntry<V>>;
  public var older:Null<MemoEntry<V>>;

  public function new(key:String, value:V) {
    this.key = key;
    this.value = value;
    newer = null;
    older = null;
  }
}
