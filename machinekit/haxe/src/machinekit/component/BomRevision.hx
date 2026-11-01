package machinekit.component;

/** Fingerprint generator inputs and the author-controlled geometry version. */
class BomRevision {
  public static function fingerprint(key:String, generatorVersion:Int):String {
    if (key == null || key == "" || generatorVersion < 1) throw "Revision requires an exact key and positive generator version";
    return haxe.crypto.Sha256.encode('machinekit.bom-revision|$generatorVersion|${key.length}:$key');
  }
}
