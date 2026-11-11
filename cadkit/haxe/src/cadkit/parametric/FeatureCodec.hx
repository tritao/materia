package cadkit.parametric;

/**
 * How a feature type defined outside CadKit is saved and restored (`FeatureTypeRegistry`).
 *
 * The feature class itself evaluates (`Feature.evaluate`) and declares its scalar parameters, dependencies and topology
 * references as any built-in feature does. The codec only turns the feature's own data into a JSON payload and back.
 * `DocumentCodec` writes the common fields around it (id, type, active, label, group, references, named-parameter
 * bindings), so a registered type is saved as `{id, type, data: <payload>, references, ...}`.
 */
interface FeatureCodec {
  /** The saved `type` string; unique across built-in and registered types. */
  function type():String;

  /** The feature's own data, as plain JSON values. Dependencies are saved by feature id (`context.featureId`). */
  function encode(feature:Feature, context:FeatureCodecContext):Dynamic;

  /** A new, unattached feature from `data`. Dependencies are earlier features (`context.feature`). Throw on bad data. */
  function decode(data:Dynamic, context:FeatureCodecContext):Feature;

  /** Topology references the payload stores itself, so the common `references` list leaves them out. Usually none. */
  function ownedReferences(feature:Feature):Array<TopologyReference>;

  /** The history row's name when the feature has no label of its own. */
  function defaultLabel(feature:Feature):String;
}
