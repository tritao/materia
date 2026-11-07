package machinekit.cache;

/**
 * Which code computed a persistent memo's values. Until the compiler provides module-closure fingerprints (OC1) there is no
 * trustworthy identity, so this returns null (the persistent tier stays off) unless `MATERIA_MEMO_STANDIN` is set. The
 * stand-in identity is that variable's value, for measuring and for tests: it never changes with the code, so whoever sets
 * it owns clearing the cache after a code change.
 */
class CodeIdentity {
  public static function of(moduleName:String):Null<String> {
    var standIn = Sys.getEnv("MATERIA_MEMO_STANDIN");
    return standIn == null || standIn == "" ? null : moduleName + "@" + standIn;
  }
}
