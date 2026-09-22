package app;

typedef ScriptOwnershipRecord = {var reference: String;
var version:Int;
/** Persisted override contract version, independent of the script's version. */
var overrideVersion:Int;
var overridesEnabled:Bool;
var overrides:Array<ScriptOverrideRecord>;
}
