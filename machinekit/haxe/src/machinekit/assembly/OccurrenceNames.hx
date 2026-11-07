package machinekit.assembly;

/** Occurrences describe installed roles; geometry keys belong to component definitions. */
class OccurrenceNames {
  public static function problems(id:String):Array<String> {
    var result:Array<String> = [];
    if (id == null || id.length == 0) return ["empty occurrence id"];
    if (~/[0-9a-fA-F]{8,}/.match(id)) result.push("hexadecimal run of eight or more characters");
    if (~/([0-9]+\.[0-9]*|\.[0-9]+)/.match(id)) result.push("decimal coordinate");
    return result;
  }

  public static function requireReadable(ids:Array<String>):Void {
    var failures:Array<String> = [];
    for (id in ids) {
      var findings = problems(id);
      if (findings.length > 0) failures.push(id + ": " + findings.join(", "));
    }
    if (failures.length > 0) throw "Unreadable occurrence ids: " + failures.join("; ");
  }
}
