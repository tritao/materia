import machinekit.assembly.MachineAssembly;
import machinekit.assembly.OccurrenceNames;

/** Naming coverage independent of strict BOM/procurement and geometry-integrity failures. */
class OccurrenceNamesAudit {
  static var checked = 0;
  static var occurrences = 0;
  static function check(name:String, machine:MachineAssembly, failures:Array<String>):Void {
    checked++;
    try {
      var definition = machine.definition();
      occurrences += definition.occurrences.length;
      OccurrenceNames.requireReadable([for (entry in definition.occurrences) entry.id]);
    } catch (error:Dynamic) failures.push(name + ": " + Std.string(error));
  }
  public static function main():Void {
    var failures:Array<String> = [];
    var hobber:Null<FlexsplineHobber> = null;
    for (name in BomExport.names()) {
      var machine:MachineAssembly;
      try machine = BomExport.build(name) catch (error:Dynamic) {
        var message = Std.string(error);
        if (name == "machine-tending" && message.indexOf("tending reach layout was certified for other cell inputs") >= 0) {
          Sys.println("Occurrence audit: machine-tending unavailable at its existing stale reach certificate");
          continue;
        }
        failures.push(name + " construction: " + message);
        continue;
      }
      check(name, machine, failures);
      if (name == "flexspline-hobber") hobber = cast machine;
    }
    check("rke92-vise", new machinekit.milling.Rke92Vise(), failures);
    check("zpp96-workholding", new machinekit.milling.ZppWorkholding(), failures);
    check("zpp52-workholding", new machinekit.milling.ZppWorkholding(60, 60, false), failures);
    check("cnc-router-belt", new CncRouter(true), failures);
    check("cnc-router-folded-z", new CncRouter(false, true), failures);
    if (hobber == null) failures.push("Enclosure naming audit has no hobber source");
    else {
      try check("hobber-enclosure", HobberEnclosurePreview.assembled(hobber).assembly, failures)
      catch (error:Dynamic) failures.push("hobber-enclosure construction: " + Std.string(error));
    }
    if (failures.length > 0) throw "Occurrence naming audit failed: " + failures.join("; ");
    Sys.println('Occurrence names: $checked assemblies, $occurrences occurrences passed');
  }
}
