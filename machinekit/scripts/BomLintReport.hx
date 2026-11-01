import machinekit.component.MachineKitComponents;
import machinekit.component.PartNumberLint;

private class LintCounts {
  public var lines:Int = 0;
  public var invalid:Int = 0;
  public function new() {}
}

/** Report default-recipe and exported BOM identity problems without failing the rollout. */
class BomLintReport {
  public static function main():Void {
    var bad = 0, unavailable = 0, checked = 0;
    for (type in MachineKitComponents.standard().all()) {
      var number:String;
      try {
        number = type.create().bom.partNumber;
      } catch (error:Dynamic) {
        unavailable++;
        Sys.println('recipe ${type.id}: default unavailable: ${Std.string(error)}');
        continue;
      }
      checked++;
      var problems = PartNumberLint.problems(number);
      if (problems.length > 0) {
        bad++;
        Sys.println('recipe ${type.id}: $number: ${problems.join(", ")}');
      }
    }
    Sys.println('recipes: $checked checked, $bad invalid, $unavailable defaults unavailable');
    for (path in Sys.args()) {
      var doc:Dynamic = haxe.Json.parse(sys.io.File.getContent(path));
      if (doc.schema != "machinekit.bom" || doc.version != 1) throw 'Unsupported BOM schema: $path';
      if (doc.assembly == null || doc.procurement == null) throw 'Incomplete BOM schema: $path';
      var engineering = new LintCounts();
      lintNode(doc.assembly, path, engineering);
      var lines:Array<Dynamic> = doc.procurement;
      var invalid = 0;
      for (line in lines) {
        var number:String = line.item.partNumber;
        var problems = PartNumberLint.problems(number);
        if (problems.length > 0) {
          invalid++;
          Sys.println('$path: $number [type ${line.item.typeId}]: ${problems.join(", ")}');
        }
      }
      Sys.println('$path: ${engineering.lines} engineering occurrences, ${engineering.invalid} invalid; '
        + '${lines.length} procurement lines, $invalid invalid');
    }
  }

  static function lintNode(node:Dynamic, path:String, counts:LintCounts):Void {
    var lines:Array<Dynamic> = node.lines, children:Array<Dynamic> = node.children;
    if (lines == null || children == null) throw 'Invalid engineering tree: $path';
    for (line in lines) {
      var number:String = line.item.partNumber;
      var problems = PartNumberLint.problems(number);
      counts.lines++;
      if (problems.length > 0) {
        counts.invalid++;
        Sys.println('$path: engineering $number [type ${line.item.typeId}]: ${problems.join(", ")}');
      }
    }
    for (child in children) lintNode(child, path, counts);
  }
}
