package app;

typedef ScriptOverrideRecord = {var targetId: String;
var property:String;
var kind:String;
/** Canonical JSON for the value selected by kind. */
var encodedValue:String;
}
