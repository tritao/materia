package machinekit.structural;

import cadkit.modeling.Part;
import cadkit.parametric.EvaluationContext;
import cadkit.parametric.EvaluationResult;
import cadkit.parametric.Feature;
import cadkit.parametric.FeatureCodec;
import cadkit.parametric.FeatureCodecContext;
import cadkit.parametric.FeatureId;
import cadkit.parametric.FeatureTypeRegistry;
import cadkit.parametric.ParametricError;
import cadkit.parametric.TopologyReference;
import machinekit.component.Solids;
import materia.casting.CastingCavities;
import materia.casting.CastingCavities.FinishedCastingCavity;
import materia.casting.CastingFeature;
import materia.casting.CastingPatterns;
import materia.casting.CastingPrism;
import materia.casting.CastingRecordCodec;

/**
 * A mineral casting's geometry as document steps, evaluated by the casting kit's own geometry: the envelope (`casting-body`,
 * which also holds the casting features), one `casting-cavity` per casting feature, the embedded anchor hardware
 * (`casting-anchors`) and the post-cure receivers (`casting-receivers`). Registered with CadKit's feature-type registry.
 */
class CastingFeatureTypes {
  static var registered = false;

  public static function register():Void {
    if (registered) return;
    registered = true;
    for (codec in ([new BodyCodec(), new CavityCodec(), new AnchorsCodec(), new ReceiversCodec()] : Array<FeatureCodec>)) FeatureTypeRegistry.register(codec);
  }
}

/** The casting's envelope prisms, fused; it holds the casting features its cavities are cut from. */
class CastingBodyFeature extends Feature {
  public final envelope:Array<CastingPrism>;
  public final features:Array<CastingFeature>;
  var finished:Null<Array<FinishedCastingCavity>> = null;

  public function new(envelope:Array<CastingPrism>, features:Array<CastingFeature>) {
    super();
    if (envelope.length == 0) throw new ParametricError("a casting needs an envelope");
    this.envelope = envelope.copy();
    this.features = features.copy();
  }

  override public function serializationType():String return "casting-body";

  /** Every feature's finished cavity, derived once (a cavity's shape depends on the envelope and the features it opens into). */
  public function cavities():Array<FinishedCastingCavity> {
    var known = finished;
    if (known != null) return known;
    var derived = CastingCavities.derive(envelope, features);
    finished = derived;
    return derived;
  }

  override public function evaluate(context:EvaluationContext):EvaluationResult {
    var body = Solids.union([for (prism in envelope) CastingCavityCad.prism(prism)]);
    var shape = body.shape.cloneShape();
    body.close();
    return EvaluationResult.fromShape(shape);
  }
}

/** One casting feature's finished cavity, as a cutting tool named by the feature. */
class CastingCavityFeature extends Feature {
  public final body:CastingBodyFeature;
  public final featureId:String;

  public function new(body:CastingBodyFeature, featureId:String) {
    super();
    this.body = body;
    this.featureId = featureId;
  }

  override public function serializationType():String return "casting-cavity";

  override public function dependencies():Array<FeatureId> return [body.id];

  override public function dependencyFeatures():Array<Feature> return [body];

  override public function evaluate(context:EvaluationContext):EvaluationResult {
    for (cavity in body.cavities()) if (cavity.featureId == featureId) {
      var part = Solids.named(CastingCavityCad.part(cavity), featureId);
      var shape = part.shape.cloneShape();
      part.close();
      return EvaluationResult.fromShape(shape);
    }
    throw new ParametricError("casting has no feature " + featureId);
  }
}

/** The embedded anchor hardware the casting is poured round, as one tool. */
class CastingAnchorsFeature extends Feature {
  public final anchors:Array<materia.casting.InsertAnchorage.EmbeddedInsertAnchors>;

  public function new(anchors:Array<materia.casting.InsertAnchorage.EmbeddedInsertAnchors>) {
    super();
    this.anchors = anchors.copy();
  }

  override public function serializationType():String return "casting-anchors";

  override public function evaluate(context:EvaluationContext):EvaluationResult {
    var parts = machinekit.tooling.InsertAnchorGeometry.embeddedParts(anchors);
    if (parts.length == 0) throw new ParametricError("no embedded anchors");
    var all = Solids.union(parts), shape = all.shape.cloneShape();
    all.close();
    return EvaluationResult.fromShape(shape);
  }
}

/** The receivers drilled after curing for bonded inserts, as one tool. */
class CastingReceiversFeature extends Feature {
  public final bonds:Array<materia.casting.PostCureBond>;

  public function new(bonds:Array<materia.casting.PostCureBond>) {
    super();
    if (bonds.length == 0) throw new ParametricError("no post-cure receivers");
    this.bonds = bonds.copy();
  }

  override public function serializationType():String return "casting-receivers";

  override public function evaluate(context:EvaluationContext):EvaluationResult {
    var all = Solids.union([for (bond in bonds) CastingCavityCad.receiver(bond.receiver)]), shape = all.shape.cloneShape();
    all.close();
    return EvaluationResult.fromShape(shape);
  }
}

private class CastingCodec implements FeatureCodec {
  public function new() {}

  public function type():String throw new ParametricError("codec without a type");

  public function encode(feature:Feature, context:FeatureCodecContext):Dynamic throw new ParametricError("unsupported");

  public function decode(data:Dynamic, context:FeatureCodecContext):Feature throw new ParametricError("unsupported");

  public function ownedReferences(feature:Feature):Array<TopologyReference> return [];

  public function defaultLabel(feature:Feature):String return type();
}

private class BodyCodec extends CastingCodec {
  public function new() super();

  override public function type():String return "casting-body";

  override public function encode(feature:Feature, context:FeatureCodecContext):Dynamic {
    var body:CastingBodyFeature = cast feature;
    return {envelope: [for (p in body.envelope) {id: p.id, profile: [for (q in p.profile) {x: q.x, y: q.y}], height: p.height, pose: p.pose}],
      features: CastingPatterns.encode(body.features)};
  }

  override public function decode(data:Dynamic, context:FeatureCodecContext):Feature {
    var envelope:Array<CastingPrism> = [for (p in context.array(data, "envelope")) {
      id: context.string(p, "id"),
      profile: [for (q in context.array(p, "profile")) {x: context.number(q, "x"), y: context.number(q, "y")}],
      height: context.number(p, "height"),
      pose: CastingRecordCodec.frame(Reflect.field(p, "pose"))
    }];
    return new CastingBodyFeature(envelope, CastingPatterns.decode(context.string(data, "features")));
  }

  override public function defaultLabel(feature:Feature):String return "Casting envelope";
}

private class CavityCodec extends CastingCodec {
  public function new() super();

  override public function type():String return "casting-cavity";

  override public function encode(feature:Feature, context:FeatureCodecContext):Dynamic {
    var cavity:CastingCavityFeature = cast feature;
    return {body: context.featureId(cavity.body), feature: cavity.featureId};
  }

  override public function decode(data:Dynamic, context:FeatureCodecContext):Feature {
    var body = Std.downcast(context.feature(Std.int(context.number(data, "body"))), CastingBodyFeature);
    if (body == null) throw new ParametricError("a casting cavity's body must be a casting body");
    return new CastingCavityFeature(body, context.string(data, "feature"));
  }

  override public function defaultLabel(feature:Feature):String return (cast feature : CastingCavityFeature).featureId;
}

private class AnchorsCodec extends CastingCodec {
  public function new() super();

  override public function type():String return "casting-anchors";

  override public function encode(feature:Feature, context:FeatureCodecContext):Dynamic
    return {anchors: haxe.Json.stringify(materia.casting.InsertAnchorage.EmbeddedAnchors.records((cast feature : CastingAnchorsFeature).anchors))};

  override public function decode(data:Dynamic, context:FeatureCodecContext):Feature
    return new CastingAnchorsFeature(materia.casting.InsertAnchorage.EmbeddedAnchors.snapshot(haxe.Json.parse(context.string(data, "anchors"))));

  override public function defaultLabel(feature:Feature):String return "Embedded anchors";
}

private class ReceiversCodec extends CastingCodec {
  public function new() super();

  override public function type():String return "casting-receivers";

  override public function encode(feature:Feature, context:FeatureCodecContext):Dynamic
    return {bonds: haxe.Json.stringify(materia.casting.PostCureBond.PostCureBonds.records((cast feature : CastingReceiversFeature).bonds))};

  override public function decode(data:Dynamic, context:FeatureCodecContext):Feature
    return new CastingReceiversFeature(materia.casting.PostCureBond.PostCureBonds.snapshot(haxe.Json.parse(context.string(data, "bonds"))));

  override public function defaultLabel(feature:Feature):String return "Post-cure receivers";
}
