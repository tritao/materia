package app;

/** One named project-source check, or a named group of other cases. */
class ProjectSourceCase {
  public final name:String;
  public final run:Null<String->Void>;
  public final members:Array<String>;
  /** Part of the default suite, which runs these in registry order. */
  public final full:Bool;

  function new(name:String, run:Null<String->Void>, members:Array<String>, full:Bool) {
    this.name = name;
    this.run = run;
    this.members = members;
    this.full = full;
  }

  /** A check of the default suite. */
  public static function suite(name:String, run:String->Void):ProjectSourceCase
    return new ProjectSourceCase(name, run, [], true);

  /** A check run only when named: a variant, a slower long form or a fixture for iterating. */
  public static function focused(name:String, run:String->Void):ProjectSourceCase
    return new ProjectSourceCase(name, run, [], false);

  /** A name for several cases, run in the order given. */
  public static function group(name:String, members:Array<String>):ProjectSourceCase
    return new ProjectSourceCase(name, null, members, false);
}

/**
 * Selects and runs project-source cases, one report line each.
 *
 * - `PROJECT_SOURCE_ONLY=a,b` runs the named cases (or groups) in that order.
 * - `PROJECT_SOURCE_FROM=a` runs the default suite from case `a` on.
 * - `PROJECT_SOURCE_LIST=1` prints the default suite's case names, one per line, and runs nothing.
 *
 * Each case reports its wall time, the process's resident and peak resident memory, and the GC heap
 * after a full collection, so a runner can schedule cases by cost and a heap that only grows shows up.
 */
class ProjectSourceSuite {
  public static function run(root:String, cases:Array<ProjectSourceCase>):Int {
    var byName = new Map<String, ProjectSourceCase>();
    for (entry in cases) {
      if (byName.exists(entry.name)) throw 'Project-source case "${entry.name}" is registered twice';
      byName.set(entry.name, entry);
    }
    for (entry in cases) for (member in entry.members)
      if (!byName.exists(member) || byName.get(member).run == null)
        throw 'Project-source group "${entry.name}" names "$member", which is not a case';
    var defaults = [for (entry in cases) if (entry.full) entry];
    if (nonEmpty(Sys.getEnv("PROJECT_SOURCE_LIST"))) {
      for (entry in defaults) Sys.println(entry.name);
      return 0;
    }
    var only = Sys.getEnv("PROJECT_SOURCE_ONLY"), from = Sys.getEnv("PROJECT_SOURCE_FROM");
    if (nonEmpty(only) && nonEmpty(from)) throw "PROJECT_SOURCE_ONLY and PROJECT_SOURCE_FROM are exclusive";
    var selected:Array<ProjectSourceCase> = [];
    if (nonEmpty(only)) {
      for (name in only.split(",")) {
        var entry = byName.get(StringTools.trim(name));
        if (entry == null) throw 'Unknown project-source case "$name"; cases: ' + [for (c in cases) c.name].join(", ");
        if (entry.run != null) selected.push(entry);
        else for (member in entry.members) selected.push(byName.get(member));
      }
    } else if (nonEmpty(from)) {
      var start = -1;
      for (index in 0...defaults.length) if (defaults[index].name == from) start = index;
      if (start < 0) throw 'PROJECT_SOURCE_FROM names "$from", which is not in the default suite';
      selected = defaults.slice(start);
    } else selected = defaults;
    for (entry in selected) runCase(root, entry);
    return 0;
  }

  static function runCase(root:String, entry:ProjectSourceCase):Void {
    var check = entry.run;
    if (check == null) throw 'Project-source group "${entry.name}" cannot run as a case';
    var started = Sys.time();
    check(root);
    var seconds = Sys.time() - started;
    #if hl
    hl.Gc.major();
    var heap = Math.round(hl.Gc.heapBytes() / 1048576);
    #else
    var heap = -1;
    #end
    var memory = residentMegabytes();
    Sys.println('project-source case ${entry.name}: ${Math.round(seconds * 10) / 10} s, rss ${memory.rss} MB, ' +
      'peak ${memory.peak} MB, heap ${heap} MB');
  }

  /** Current and peak resident set of this process from `/proc/self/status`; -1 where unavailable. */
  static function residentMegabytes():{rss:Int, peak:Int} {
    var rss = -1, peak = -1;
    try {
      for (line in sys.io.File.getContent("/proc/self/status").split("\n")) {
        var fields = [for (field in ~/\s+/g.split(line)) if (field.length > 0) field];
        if (fields.length < 2) continue;
        if (fields[0] == "VmRSS:") rss = Math.round(Std.parseFloat(fields[1]) / 1024);
        if (fields[0] == "VmHWM:") peak = Math.round(Std.parseFloat(fields[1]) / 1024);
      }
    } catch (_:Dynamic) {}
    return {rss: rss, peak: peak};
  }

  static inline function nonEmpty(value:Null<String>):Bool return value != null && value.length > 0;
}
