package machinekit.component;

/** Read the version-one BOM envelope; consumers validate the fields they use. */
class BomJson {
  public static function parse(json:String):machinekit.bomschema.BomV1.Document {
    var document:Dynamic = haxe.Json.parse(json);
    if (document == null || document.schema != "machinekit.bom" || document.version != 1)
      throw "Unsupported BOM schema or version";
    if (document.assembly == null || document.procurement == null)
      throw "Incomplete BOM document";
    return document;
  }
}
