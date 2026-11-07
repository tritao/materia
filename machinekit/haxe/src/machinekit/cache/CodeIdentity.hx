package machinekit.cache;

/**
 * Which code computed a persistent memo's values: the compiler's fingerprint of a module and everything it imports
 * (`haxeon.build.CodeIdentity`). Editing the module or a module it depends on changes it, so values computed by the old code
 * are never read; editing any other module does not. Null (the persistent tier stays off) for a module the program does not
 * contain, which also catches a misspelt module name.
 */
class CodeIdentity {
  public static function of(moduleName:String):Null<String> return haxeon.build.CodeIdentity.ofModule(moduleName);

  /** The identity of the module defining `object`'s concrete class (its own code and everything it imports), or null. */
  public static function ofObject(object:Dynamic):Null<String> return haxeon.build.CodeIdentity.ofObject(object);
}
