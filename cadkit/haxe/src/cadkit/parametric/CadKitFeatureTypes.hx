package cadkit.parametric;

import haxe.Json;
import haxeon.wire.JsonWire;
import cadkit.modeling.Plane;
import cadkit.modeling.Vector;
import cadkit.parametric.features.BoxBetweenFeature;
import cadkit.parametric.features.CylinderAlongFeature;
import cadkit.parametric.features.HolesFeature;
import cadkit.parametric.features.PlaceFeature;
import cadkit.parametric.features.UnionAllFeature;

/**
 * The feature types CadKit itself adds through `FeatureTypeRegistry`: everything new goes this way, so a type is one class
 * and one codec, not a branch in `DocumentCodec`. `FeatureTypeRegistry` registers this list before its first lookup; other
 * packages register their own types explicitly.
 */
class CadKitFeatureTypes {
  public static function codecs():Array<FeatureCodec> {
    var list:Array<FeatureCodec> = [new PlaceCodec(), new BoxBetweenCodec(), new CylinderAlongCodec(), new UnionAllCodec(), new HolesCodec()];
    for (codec in cadkit.sheetmetal.SheetFeatureTypes.codecs()) list.push(codec);
    return list;
  }
}

/** Shared parts: no owned references, the row named by the type. */
private class CoreCodec implements FeatureCodec {
  public function new() {}

  public function type():String throw new ParametricError("codec without a type");

  public function encode(feature:Feature, context:FeatureCodecContext):Dynamic throw new ParametricError("unsupported");

  public function decode(data:Dynamic, context:FeatureCodecContext):Feature throw new ParametricError("unsupported");

  public function ownedReferences(feature:Feature):Array<TopologyReference> return [];

  public function defaultLabel(feature:Feature):String return type();
}

private class PlaceCodec extends CoreCodec {
  public function new() super();

  override public function type():String return "place";

  override public function encode(feature:Feature, context:FeatureCodecContext):Dynamic {
    var place:PlaceFeature = cast feature;
    var data:Dynamic = {
      source: context.featureId(place.source), x: place.x.value, y: place.y.value, z: place.z.value,
      xDirection: context.encodeVector(place.xDirection), normal: context.encodeVector(place.normal)
    };
    if (place.datum != null) Reflect.setField(data, "datum", context.encodeElementReference(place.datum));
    return data;
  }

  override public function decode(data:Dynamic, context:FeatureCodecContext):Feature {
    var datum:Dynamic = Reflect.field(data, "datum");
    return new PlaceFeature(context.feature(Std.int(context.number(data, "source"))),
      new Plane(new Vector(context.number(data, "x"), context.number(data, "y"), context.number(data, "z")),
        context.decodeVector(Reflect.field(data, "xDirection")), context.decodeVector(Reflect.field(data, "normal"))),
      datum == null ? null : context.decodeElementReference(datum));
  }
}

private class BoxBetweenCodec extends CoreCodec {
  public function new() super();

  override public function type():String return "box-between";

  override public function encode(feature:Feature, context:FeatureCodecContext):Dynamic {
    var box:BoxBetweenFeature = cast feature;
    return {x0: box.x0.value, x1: box.x1.value, y0: box.y0.value, y1: box.y1.value, z0: box.z0.value, z1: box.z1.value};
  }

  override public function decode(data:Dynamic, context:FeatureCodecContext):Feature return new BoxBetweenFeature(
    context.number(data, "x0"), context.number(data, "x1"), context.number(data, "y0"), context.number(data, "y1"),
    context.number(data, "z0"), context.number(data, "z1"));
}

private class CylinderAlongCodec extends CoreCodec {
  public function new() super();

  override public function type():String return "cylinder-along";

  override public function encode(feature:Feature, context:FeatureCodecContext):Dynamic {
    var cylinder:CylinderAlongFeature = cast feature;
    return {origin: context.encodeVector(cylinder.origin), axis: context.encodeVector(cylinder.axis), radius: cylinder.radius.value,
      from: cylinder.from.value, to: cylinder.to.value};
  }

  override public function decode(data:Dynamic, context:FeatureCodecContext):Feature return new CylinderAlongFeature(
    context.decodeVector(Reflect.field(data, "origin")), context.decodeVector(Reflect.field(data, "axis")), context.number(data, "radius"),
    context.number(data, "from"), context.number(data, "to"));
}

private class UnionAllCodec extends CoreCodec {
  public function new() super();

  override public function type():String return "union-all";

  override public function encode(feature:Feature, context:FeatureCodecContext):Dynamic {
    var union:UnionAllFeature = cast feature;
    return {parts: [for (part in union.parts) context.featureId(part)]};
  }

  override public function decode(data:Dynamic, context:FeatureCodecContext):Feature
    return new UnionAllFeature([for (id in context.array(data, "parts")) context.feature(Std.int(id))]);
}

private class HolesCodec extends CoreCodec {
  public function new() super();

  override public function type():String return "holes";

  override public function encode(feature:Feature, context:FeatureCodecContext):Dynamic {
    var drilled:HolesFeature = cast feature;
    var specs:Array<HoleSpec> = drilled.holes;
    return {source: context.featureId(drilled.source), holes: Json.parse(JsonWire.encode(specs))};
  }

  override public function decode(data:Dynamic, context:FeatureCodecContext):Feature {
    var specs:Array<HoleSpec> = JsonWire.decode(Json.stringify(Reflect.field(data, "holes")));
    return new HolesFeature(context.feature(Std.int(context.number(data, "source"))), specs);
  }
}
