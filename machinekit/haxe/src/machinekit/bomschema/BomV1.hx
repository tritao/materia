package machinekit.bomschema;

/** Version-one wire types shared by the document reader and downstream consumers. */
typedef Quantity = { var value:Float; var unit:String; }
typedef Use = { var kind:String; var path:String; var quantity:Quantity; }
typedef Cut = { var length:Float; var count:Int; }
typedef Procurement = {
  var kind:String;
  var process:Null<String>;
  var manufacturer:Null<String>;
  var mpn:Null<String>;
  var standard:Null<String>;
}
typedef Requirement = {
  var name:String;
  var min:Null<String>;
  var max:Null<String>;
  var nominal:Null<String>;
  var unit:String;
}
typedef Item = {
  var partNumber:String;
  var description:String;
  var quantity:Quantity;
  var material:Null<String>;
  var typeId:Null<String>;
  var valuesKey:Null<String>;
  var familyId:Null<String>;
  var manufacturingVariant:Null<String>;
  var manufacturingProcess:Null<String>;
  var properties:Dynamic;
  var cuts:Array<Cut>;
  var includedIn:Null<String>;
  var procurement:Null<Procurement>;
  var maturity:Null<String>;
  var requirements:Array<Requirement>;
  var fingerprint:Null<String>;
  var releasedRevision:Null<String>;
  var releaseCurrent:Null<Bool>;
}
typedef Line = { var item:Item; var uses:Array<Use>; }
typedef Document = {
  var schema:String;
  var version:Int;
  var machineId:String;
  var sourceCommit:String;
  var generatedAt:String;
  // The engineering tree is recursive; the current Haxeon structural reader
  // cannot convert recursive anonymous records from Dynamic.
  var assembly:Dynamic;
  var procurement:Array<Line>;
}
