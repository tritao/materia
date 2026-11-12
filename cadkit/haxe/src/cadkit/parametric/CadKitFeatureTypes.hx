package cadkit.parametric;

import haxe.Json;
import haxeon.wire.JsonWire;
import cadkit.modeling.Plane;
import cadkit.modeling.Vector;
import cadkit.parametric.features.BlockFeature;
import cadkit.parametric.features.CutAllFeature;
import cadkit.parametric.features.PolygonFeature;
import cadkit.parametric.features.BoxBetweenFeature;
import cadkit.parametric.features.CylinderAlongFeature;
import cadkit.parametric.features.HolesFeature;
import cadkit.parametric.features.PlaceFeature;
import cadkit.parametric.features.StrokeTextFeature;
import cadkit.parametric.features.FaceDetailFeature;
import cadkit.parametric.features.FaceDetailFeature.FaceDetail;
import cadkit.threads.ThreadedFace.ThreadStart;
import cadkit.threads.SurfaceDetail;
import cadkit.threads.ThreadSpec;
import cadkit.parametric.features.UnionAllFeature;

/**
 * The feature types CadKit itself adds through `FeatureTypeRegistry`: everything new goes this way, so a type is one class
 * and one codec, not a branch in `DocumentCodec`. `FeatureTypeRegistry` registers this list before its first lookup; other
 * packages register their own types explicitly.
 */
class CadKitFeatureTypes {
  public static function codecs():Array<FeatureCodec> {
    var list:Array<FeatureCodec> = [new PlaceCodec(), new BlockCodec(), new PolygonCodec(), new BoxBetweenCodec(), new CylinderAlongCodec(), new UnionAllCodec(), new CutAllCodec(), new HolesCodec(), new StrokeTextCodec(), new FaceDetailCodec()];
    for (codec in ModelingFeatureTypes.codecs()) list.push(codec);
    for (codec in SolidFeatureTypes.codecs()) list.push(codec);
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

private class PolygonCodec extends CoreCodec {
  public function new() super();

  override public function type():String return "polygon";

  override public function encode(feature:Feature, context:FeatureCodecContext):Dynamic {
    var polygon:PolygonFeature = cast feature;
    return {points: [for (p in polygon.points) {x: p.x, y: p.y}], origin: context.encodeVector(polygon.plane.origin),
      xDirection: context.encodeVector(polygon.plane.xDirection), normal: context.encodeVector(polygon.plane.normal)};
  }

  override public function decode(data:Dynamic, context:FeatureCodecContext):Feature
    return new PolygonFeature([for (p in context.array(data, "points")) new Vector(context.number(p, "x"), context.number(p, "y"), 0)],
      new Plane(context.decodeVector(Reflect.field(data, "origin")), context.decodeVector(Reflect.field(data, "xDirection")),
        context.decodeVector(Reflect.field(data, "normal"))));
}

private class BlockCodec extends CoreCodec {
  public function new() super();

  override public function type():String return "block";

  override public function encode(feature:Feature, context:FeatureCodecContext):Dynamic {
    var block:BlockFeature = cast feature;
    return {x: block.x.value, y: block.y.value, z: block.z.value, align: block.align.copy()};
  }

  override public function decode(data:Dynamic, context:FeatureCodecContext):Feature
    return new BlockFeature(context.number(data, "x"), context.number(data, "y"), context.number(data, "z"),
      [for (a in context.array(data, "align")) Std.string(a)]);
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

private class CutAllCodec extends CoreCodec {
  public function new() super();

  override public function type():String return "cut-all";

  override public function encode(feature:Feature, context:FeatureCodecContext):Dynamic {
    var cut:CutAllFeature = cast feature;
    return {target: context.featureId(cut.target), tools: [for (tool in cut.tools) context.featureId(tool)]};
  }

  override public function decode(data:Dynamic, context:FeatureCodecContext):Feature
    return new CutAllFeature(context.feature(Std.int(context.number(data, "target"))), [for (id in context.array(data, "tools")) context.feature(Std.int(id))]);
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

private class StrokeTextCodec extends CoreCodec {
  public function new() super();

  override public function type():String return "stroke-text";

  override public function encode(feature:Feature, context:FeatureCodecContext):Dynamic {
    var text:StrokeTextFeature = cast feature;
    return {text: text.text, height: text.height.value, stroke: text.stroke.value, z0: text.z0.value, z1: text.z1.value, mirror: text.mirror,
      x: text.x.value, y: text.y.value, rotation: text.rotation.value};
  }

  override public function decode(data:Dynamic, context:FeatureCodecContext):Feature return new StrokeTextFeature(
    Std.string(Reflect.field(data, "text")), context.number(data, "height"), context.number(data, "stroke"), context.number(data, "z0"),
    context.number(data, "z1"), Reflect.field(data, "mirror") == true, context.number(data, "x"), context.number(data, "y"),
    context.number(data, "rotation"));
}

private class FaceDetailCodec extends CoreCodec {
  public function new() super();

  override public function type():String return "face-detail";

  override public function encode(feature:Feature, context:FeatureCodecContext):Dynamic {
    var value:FaceDetailFeature = cast feature;
    var detail:Dynamic = switch value.detail {
      case Thread(spec, start, length, offset):
        {kind: "thread", thread: Json.parse(JsonWire.encode(spec.data())), start: start == AxisMinimum ? "minimum" : "maximum", length: length,
          offset: offset};
      case Surface(Knurl(teeth, depth)):
        {kind: "knurl", teeth: teeth, depth: depth};
      case Surface(Graduation(divisions, every, slant, endRadius)):
        {kind: "graduation", divisions: divisions, every: every, slant: slant, endRadius: endRadius};
      case Surface(Display(spec)):
        {kind: "display", width: spec.width, height: spec.height, offsetX: spec.offsetX, offsetY: spec.offsetY, source: spec.source,
          profile: spec.profile, channel: spec.channel, round: spec.round,
          bindings: [for (binding in spec.bindings) {signal: binding.signal, joint: binding.joint}]};
    };
    return {source: context.featureId(value.source), face: value.face, detail: detail};
  }

  override public function decode(data:Dynamic, context:FeatureCodecContext):Feature {
    var d:Dynamic = context.field(data, "detail");
    var detail:FaceDetail = switch context.string(d, "kind") {
      case "thread":
        var length:Null<Float> = Reflect.field(d, "length");
        var data:materia.project.ThreadSpecData = JsonWire.decode(Json.stringify(Reflect.field(d, "thread")));
        Thread(ThreadSpec.fromData(data),
          context.string(d, "start") == "minimum" ? AxisMinimum : AxisMaximum, length, context.number(d, "offset"));
      case "knurl":
        Surface(Knurl(context.integer(d, "teeth"), context.number(d, "depth")));
      case "graduation":
        Surface(Graduation(context.integer(d, "divisions"), context.integer(d, "every"), context.number(d, "slant"), context.number(d, "endRadius")));
      case "display":
        var rows:Array<Dynamic> = context.field(d, "bindings");
        Surface(Display(new cadkit.threads.SurfaceDetail.DisplaySpec(context.number(d, "width"), context.number(d, "height"),
          context.number(d, "offsetX"), context.number(d, "offsetY"), context.string(d, "source"), context.string(d, "profile"),
          [for (row in rows) new cadkit.threads.SurfaceDetail.DisplayBinding(Std.string(Reflect.field(row, "signal")), Std.string(Reflect.field(row, "joint")))],
          context.string(d, "channel"), context.boolean(d, "round"))));
      case other:
        throw new ParametricError('unknown face detail "$other"');
    };
    return new FaceDetailFeature(context.feature(context.integer(data, "source")), context.string(data, "face"), detail);
  }
}
