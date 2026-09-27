package machinekit.component;

/** Stored input value; its variant must match the parameter type. */
enum ComponentValue {
	Number(value:Float);
	Integer(value:Int);
	Boolean(value:Bool);
	Token(value:String);
}
