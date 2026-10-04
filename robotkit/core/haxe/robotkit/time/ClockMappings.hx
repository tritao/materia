package robotkit.time;

import haxe.Int64;

private class ClockRoute {
  public final clockId:String;
  public final valueNs:Int64;
  public final errorBoundNs:Int64;
  public final visited:Array<String>;

  public function new(clockId:String, valueNs:Int64, errorBoundNs:Int64,
      visited:Array<String>) {
    this.clockId = clockId; this.valueNs = valueNs;
    this.errorBoundNs = errorBoundNs; this.visited = visited;
  }
}

/** Explicit directed mappings; no inverse edge or clock relationship is inferred. */
class ClockMappings {
  final mappings:Array<ClockMapping> = [];

  public function new() {}

  /** Replaces an older mapping for the same directed clock pair. */
  public function add(mapping:ClockMapping):Void {
    if (mapping == null) throw "Clock mapping is required";
    for (index in 0...mappings.length) if (mappings[index].fromClockId == mapping.fromClockId &&
        mappings[index].toClockId == mapping.toClockId) {
      mappings[index] = mapping;
      return;
    }
    mappings.push(mapping);
  }

  /** Chooses the valid path with the lowest sum of error bounds.
   * Equal-bound paths differing by more than one nanosecond are ambiguous. */
  public function map(timestampNs:Int64, fromClockId:String,
      toClockId:String):Null<MappedTimestamp> {
    if (fromClockId == null || fromClockId.length == 0 ||
        toClockId == null || toClockId.length == 0)
      throw "Clock IDs are required";
    if (fromClockId == toClockId)
      return new MappedTimestamp(timestampNs, Int64.ofInt(0));
    var pending:Array<ClockRoute> = [new ClockRoute(fromClockId, timestampNs,
      Int64.ofInt(0), [fromClockId])];
    var best:Null<MappedTimestamp> = null;
    var ambiguous = false;
    while (pending.length > 0) {
      var route = pending.pop();
      if (route == null) break;
      for (mapping in mappings) {
        if (mapping.fromClockId != route.clockId ||
            route.visited.indexOf(mapping.toClockId) >= 0) continue;
        var mapped = mapping.map(route.valueNs);
        if (mapped == null) continue;
        var bound = ClockMapping.checkedAdd(route.errorBoundNs, mapped.errorBoundNs);
        if (bound == null) continue;
        if (best != null && Int64.compare(bound, best.errorBoundNs) > 0) continue;
        if (mapping.toClockId == toClockId) {
          if (best == null || Int64.compare(bound, best.errorBoundNs) < 0) {
            best = new MappedTimestamp(mapped.valueNs, bound);
            ambiguous = false;
          } else {
            var difference = ClockMapping.checkedSubtract(mapped.valueNs, best.valueNs);
            if (difference == null || Int64.compare(difference, Int64.ofInt(1)) > 0 ||
                Int64.compare(difference, Int64.ofInt(-1)) < 0) ambiguous = true;
          }
        } else {
          var visited = route.visited.copy(); visited.push(mapping.toClockId);
          pending.push(new ClockRoute(mapping.toClockId, mapped.valueNs, bound, visited));
        }
      }
    }
    return ambiguous ? null : best;
  }
}
