package app;

typedef ScriptOwnershipRecord = {
  var reference:String;
  var version:Int;
  var overridesEnabled:Bool;
  var overrides:Array<ScriptOverrideRecord>;
}
