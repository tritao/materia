package materia.casting;

/** Precast material is the same production compound as the surrounding casting.
 * Its supplier and strengths are resolved by the shared shop declaration.
 */
enum abstract StrutMaterial(String) to String {
 var CastingCompound = "casting-compound";
}

enum abstract StrutContact(String) to String {
 var Lower = "lower";
 var Upper = "upper";
}

/** Source construction feature, in casting-local mm. It remains in the casting;
 * it is neither temporary mold hardware nor a new opening through the source.
 */
typedef PrecastStrut = {
 var id:String;
 var coreFeatureId:String;
 var material:StrutMaterial;
 var contact:StrutContact;
 var box:Box3;
}

class PrecastStruts {
 public static inline var VERSION = 1;

 public static function records(struts:Array<PrecastStrut>):Array<Dynamic> return [for (strut in struts) {
  id:strut.id,coreFeatureId:strut.coreFeatureId,material:strut.material,contact:strut.contact,box:BoxAlgebra.copy(strut.box)
 }];

 public static function copy(struts:Array<PrecastStrut>):Array<PrecastStrut> return snapshot(records(struts));

 public static function encode(struts:Array<PrecastStrut>):String
  return haxe.Json.stringify({version:VERSION,struts:records(copy(struts))});

 public static function decode(text:String):Array<PrecastStrut> {
  var raw:Dynamic = haxe.Json.parse(text);
  CastingFeatureCodec.fields(raw,["version","struts"]);
  if (raw.version != VERSION) throw "Unsupported precast strut source schema";
  return snapshot(raw.struts);
 }

 public static function snapshot(raw:Dynamic):Array<PrecastStrut> {
  if (!Std.isOfType(raw,Array)) throw "Precast struts must be an array";
  var rows:Array<Dynamic> = raw;
  if (rows.length > 128) throw "Too many source precast struts";
  var result:Array<PrecastStrut> = [], ids = new Map<String,Bool>();
  for (row in rows) {
   CastingFeatureCodec.fields(row,["id","coreFeatureId","material","contact","box"]);
   var id = CastingRecordCodec.string(row,"id"), core = CastingRecordCodec.string(row,"coreFeatureId");
   CastingFeatureCodec.identifier(id); CastingFeatureCodec.identifier(core);
   if (ids.exists(id)) throw "Duplicate precast strut: " + id;
   ids.set(id,true);
   if (CastingRecordCodec.string(row,"material") != StrutMaterial.CastingCompound)
    throw "Unknown precast strut material: " + id;
   var contact = switch CastingRecordCodec.string(row,"contact") {
    case "lower": StrutContact.Lower;
    case "upper": StrutContact.Upper;
    default: throw "Unknown precast strut contact: " + id;
   };
   var box:Dynamic = Reflect.field(row,"box");
   CastingFeatureCodec.fields(box,["x","y","z","width","depth","height"]);
   var bounds:Box3 = {x:CastingRecordCodec.number(box,"x"),y:CastingRecordCodec.number(box,"y"),
    z:CastingRecordCodec.number(box,"z"),width:CastingRecordCodec.number(box,"width"),
    depth:CastingRecordCodec.number(box,"depth"),height:CastingRecordCodec.number(box,"height")};
   if (bounds.width <= 0 || bounds.depth <= 0 || bounds.height <= 0) throw "Empty precast strut: " + id;
   result.push({id:id,coreFeatureId:core,material:StrutMaterial.CastingCompound,contact:contact,box:bounds});
  }
  result.sort((a,b) -> Reflect.compare(a.id,b.id));
  return result;
 }
}
