package app;

typedef ScriptOwnershipRecord = {var reference: String;
var version:Int;
/** Persisted override contract version, independent of the script's version. */
var overrideVersion:Int;
var overridesEnabled:Bool;
var overrides:Array<ScriptOverrideRecord>;
/** Absent in legacy scene files; filled when they are saved again. */
@:optional var identityVersion:Int;
@:optional var packageId:String;
@:optional var packageVersion:String;
@:optional var sourceSha256:String;
@:optional var configurationSha256:String;
}
