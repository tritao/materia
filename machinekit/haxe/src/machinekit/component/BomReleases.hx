package machinekit.component;

/** One issued drawing revision, in chronological release order. */
typedef BomRelease = {
  var partNumber:String;
  var revision:String;
  var fingerprint:String;
  var date:String;
}

/** Version-one release records. A later record for a part supersedes an earlier one. */
class BomReleases {
  final latest:Map<String, BomRelease> = [];
  final seen:Map<String, Bool> = [];

  public function new(records:Array<BomRelease>) {
    if (records == null) throw "Release records are required";
    for (record in records) {
      if (record == null || record.partNumber == null || record.partNumber == ""
        || record.revision == null || !~/^[A-Z]+$/.match(record.revision)
        || record.fingerprint == null || !~/^[0-9a-f]{64}$/.match(record.fingerprint)
        || record.date == null || !~/^[0-9]{4}-[0-9]{2}-[0-9]{2}$/.match(record.date))
        throw "Invalid BOM release record";
      var key = record.partNumber + "|" + record.revision;
      if (seen.exists(key)) throw 'Duplicate BOM release: $key';
      var previous = latest.get(record.partNumber);
      if (previous != null && previous.date > record.date) throw 'BOM releases are not chronological: ${record.partNumber}';
      seen.set(key, true);
      latest.set(record.partNumber, {partNumber:record.partNumber, revision:record.revision,
        fingerprint:record.fingerprint, date:record.date});
    }
  }

  public static function parse(json:String):BomReleases {
    var document:Dynamic = haxe.Json.parse(json);
    if (document == null || document.version != 1 || document.releases == null)
      throw "Unsupported BOM release schema";
    var raw:Array<Dynamic> = document.releases;
    var records:Array<BomRelease> = [];
    for (row in raw) {
      if (row == null) throw "Invalid BOM release record";
      records.push({partNumber:row.partNumber, revision:row.revision, fingerprint:row.fingerprint, date:row.date});
    }
    return new BomReleases(records);
  }

  public function find(partNumber:String):Null<BomRelease> return latest.get(partNumber);
}
