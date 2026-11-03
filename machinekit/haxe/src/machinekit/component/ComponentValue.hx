package machinekit.component;

/** Stored input value; its variant must match the parameter type. */
enum ComponentValue implements Equatable {
  Number(value:Float);
  Integer(value:Int);
  Boolean(value:Bool);
  Token(value:String);
  Unset;
}
