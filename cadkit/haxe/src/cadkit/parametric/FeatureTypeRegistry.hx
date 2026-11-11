package cadkit.parametric;

/**
 * Feature types that packages above CadKit add to documents: castings, joint holes and other domain features.
 *
 * A registered type is saved and restored through its `FeatureCodec`; everything else about it (evaluation, parameters,
 * dependencies, topology references, history rows) works as for a built-in feature. A document that names a type nobody
 * registered fails to decode, naming the type; it is never skipped.
 */
class FeatureTypeRegistry {
  static var codecs:Map<String, FeatureCodec> = new Map();

  /** Register `codec` for its type. Registering the same codec again is a no-op; another codec for a taken type throws. */
  public static function register(codec:FeatureCodec):Void {
    if (codec == null) throw new ParametricError("feature codec must not be null");
    var type = codec.type();
    if (type == null || StringTools.trim(type) == "" || type != StringTools.trim(type))
      throw new ParametricError("feature type name must be non-empty without surrounding spaces");
    if (DocumentCodec.isBuiltInType(type)) throw new ParametricError("feature type is built in: " + type);
    var existing = codecs.get(type);
    if (existing == codec) return;
    if (existing != null) throw new ParametricError("feature type is already registered: " + type);
    codecs.set(type, codec);
  }

  public static function isRegistered(type:String):Bool return codecs.exists(type);

  public static function codec(type:String):Null<FeatureCodec> return codecs.get(type);

  /** Registered type names, sorted. */
  public static function types():Array<String> {
    var result = [for (type in codecs.keys()) type];
    result.sort(Reflect.compare);
    return result;
  }
}
