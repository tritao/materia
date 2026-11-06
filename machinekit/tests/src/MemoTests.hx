import cadkit.modeling.Part;
import machinekit.cache.Memo;
import machinekit.cache.Memos;
import machinekit.cache.Once;
import machinekit.cache.Parallel;
import sys.thread.Mutex;

/** The build-time cache facility: bounds, copy-out, counters, and exclusion under real threads. */
class MemoTests {
  static var passed = 0;

  static function check(ok:Bool, message:String):Void {
    if (!ok) throw message;
    passed++;
  }

  static function near(a:Float, b:Float, message:String, tolerance:Float = 1e-9):Void check(Math.abs(a - b) <= tolerance,
    '$message: $a vs $b');

  static final values = new Memo<Array<Int>>("test-values", 3, list -> list.copy());
  static final nullable = new Memo<Null<String>>("test-nullable", 4);
  static final solids = Memos.parts("test-solids", 8);
  static final counted = new Memo<Int>("test-counted", 64);
  static final once = new Once<Array<Int>>();

  static function bounded():Void {
    var builds = 0;
    function value(key:String):Array<Int> return values.get(key, () -> {
      builds++;
      return [key.length, builds];
    });
    var a = value("a");
    a.push(99);
    check(value("a").length == 2, "a kept value is copied out, not shared");
    check(builds == 1, "a hit computes nothing");
    value("bb");
    value("ccc");
    value("a");
    value("dddd");
    // Capacity 3, least recently used dropped: "bb" (a was used after it).
    var before = builds;
    value("a");
    value("ccc");
    check(builds == before, "recently used entries stay");
    value("bb");
    check(builds == before + 1, "the least recently used entry was dropped");
    var stats = values.stats();
    check(stats.size == 3 && stats.evictions == 2, "size and evictions counted: " + stats.size + "/" + stats.evictions);
    check(stats.hits == 4 && stats.misses == 5, "hits and misses counted: " + stats.hits + "/" + stats.misses);
    var unkept = 0;
    values.get(null, () -> {
      unkept++;
      return [];
    });
    values.get(null, () -> {
      unkept++;
      return [];
    });
    check(unkept == 2, "a null key is never kept");
    var failed = false;
    try values.get("fails", () -> throw "no value") catch (_:Dynamic) failed = true;
    check(failed && !values.contains("fails"), "a computation that throws keeps nothing");
    check(nullable.get("none", () -> null) == null && nullable.contains("none"), "a null value is kept");
    values.clear();
    check(values.stats().size == 0 && values.stats().hits == 0, "clear empties the memo and its counters");
    check(Memos.report().filter(line -> StringTools.startsWith(line, "test-values:")).length == 1, "memos join the registry");
  }

  static function nativeCopies():Void {
    var built = 0;
    function box():Part return solids.get("box", () -> {
      built++;
      return Part.box(10, 20, 30);
    });
    var first = box(), second = box();
    check(built == 1, "the solid is built once");
    check(!first.shape.sameAs(second.shape), "each caller gets its own topology");
    near(first.volume(), 6000, "the copy keeps the geometry", 1e-6);
    // Tessellating one copy leaves the next copy's topology untouched (no triangulation shared).
    first.shape.tessellate(0.5, 0.5);
    var third = box();
    check(!third.shape.sameAs(first.shape), "a later copy does not share the tessellated topology");
    near(third.volume(), first.volume(), "copies stay equal");
    first.close();
    second.close();
    third.close();
  }

  static function threaded():Void {
    if (Parallel.threads() < 2) Parallel.setThreads(4);
    // Many threads, overlapping keys: every caller sees the value its key names, and Once builds exactly once.
    var lock = new Mutex(), onceBuilds = 0;
    var tasks:Array<() -> Int> = [];
    for (t in 0...8) tasks.push(() -> {
      var sum = 0;
      for (i in 0...4000) {
        var key = (i * 7 + t) % 97;
        var value = counted.get(Std.string(key), () -> key * key);
        if (value != key * key) throw "wrong value for " + key;
        sum += value;
        var shared = once.get(() -> {
          lock.acquire();
          onceBuilds++;
          lock.release();
          return [1, 2, 3];
        });
        if (shared.length != 3) throw "once value";
      }
      return sum;
    });
    var sums = Parallel.all(tasks);
    var expected = [for (t in 0...8) {
      var sum = 0;
      for (i in 0...4000) {
        var key = (i * 7 + t) % 97;
        sum += key * key;
      }
      sum;
    }];
    for (t in 0...8) check(sums[t] == expected[t], "results land at their task's index");
    check(onceBuilds == 1, "Once builds exactly once under contention: " + onceBuilds);
    check(counted.stats().size == 64, "the shared memo stays bounded");
    // Native work on every thread: copies of one kept solid, cut and measured concurrently.
    var volumes = Parallel.all([for (t in 0...8) () -> {
      var total = 0.0;
      for (i in 0...20) {
        var block = solids.get("block", () -> Part.box(40, 40, 40));
        var hole = Part.cylinderSpan(5 + t, -1, 41);
        var cut = block.subtract(hole);
        cut.shape.tessellate(1, 0.5);
        total += cut.volume();
        cut.close();
        hole.close();
        block.close();
      }
      total;
    }]);
    for (t in 0...8) near(volumes[t], 20 * (64000 - Math.PI * (5 + t) * (5 + t) * 40), "threaded cut " + t, 1e-3);
    // Errors: the first failing task in task order, after all ran.
    var ran = [false, false, false];
    var message = "";
    try Parallel.all([() -> {
      ran[0] = true;
      return 0;
    }, () -> {
      ran[1] = true;
      throw "second";
    }, () -> {
      ran[2] = true;
      throw "third";
    }]) catch (error:Dynamic) message = Std.string(error);
    check(message == "second" && ran[0] && ran[1] && ran[2], "the first error in task order, after all tasks: " + message);
    // A speculative search finds the same index as the serial scan.
    var odd:Int -> Null<String> = index -> index >= 13 && index % 2 == 1 ? "at " + index : null;
    var found = Parallel.first(20, odd);
    check(found != null && found.index == 13 && found.value == "at 13", "first finds the earliest index");
    var none:Int -> Null<String> = _ -> null;
    check(Parallel.first(5, none) == null, "first finds nothing when nothing does");
    // A step started inside a worker runs serially there.
    var nested = Parallel.all([for (t in 0...4) () -> Parallel.all([for (i in 0...3) () -> t * 10 + i]).join(",")]);
    check(nested.join(";") == "0,1,2;10,11,12;20,21,22;30,31,32", "nested steps keep their order");
  }

  public static function main():Void {
    bounded();
    nativeCopies();
    threaded();
    Sys.println("memo tests: " + passed + " checks passed (" + Parallel.threads() + " threads)");
  }
}
