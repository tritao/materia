/**
 * Runs only `MachineKitNamingAudit`, which takes about a minute instead of the smoke suite's shard:
 * `haxeon run --project machinekit/tests/haxeon.naming.json` (`MACHINEKIT_NAMING_AUDIT=print` lists the offenders).
 */
class NamingAuditMain {
  static function main():Void {
    MachineKitNamingAudit.run();
    Sys.println("naming audit passed");
  }
}
