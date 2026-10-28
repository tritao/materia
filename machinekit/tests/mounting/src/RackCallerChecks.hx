/** Run existing shared-builder regressions in separate guarded processes. */
class RackCallerChecks {
 public static function main():Void {
  var args=Sys.args();
  switch(args.length==0?"gantry":args[0]) {
   case "gantry":GantryChecks.run();
   case "track":LinearTrackCheck.run();
   case _:throw "Choose gantry or track caller regressions";
  }
  Sys.println("Required-mount caller regressions pass");
 }
}
