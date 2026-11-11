package cadkit.parametric;

import haxe.Json;
import haxeon.wire.JsonWire;
import cadkit.modeling.Vector;
import cadkit.parametric.features.GridFeature;
import cadkit.parametric.features.HoleFeature;
import cadkit.parametric.features.LinearPatternFeature;
import cadkit.parametric.features.LoftFeature;
import cadkit.parametric.features.OffsetFeature;
import cadkit.parametric.features.PolarPatternFeature;
import cadkit.parametric.features.PolylineFeature;
import cadkit.parametric.features.ProjectFeature;
import cadkit.parametric.features.ShellFeature;
import cadkit.parametric.features.SweepFeature;
import cadkit.parametric.features.WireFeature;
import cadkit.threads.ThreadSpec;
import materia.project.ThreadSpecData;

/** Codecs for CadKit's wire, sweep, loft, offset, shell, projection, pattern and hole features. */
class ModelingFeatureTypes {
  public static function codecs():Array<FeatureCodec> return [new WireCodec(), new PolylineCodec(), new LoftCodec(), new SweepCodec(),
    new OffsetCodec(), new ShellCodec(), new ProjectCodec(), new GridCodec(), new LinearPatternCodec(), new PolarPatternCodec(),
    new HoleCodec()];
}

/** Shared parts: no owned references, the row named by the type. */
private class ModelingCodec implements FeatureCodec {
  public function new() {}

  public function type():String throw new ParametricError("codec without a type");

  public function encode(feature:Feature, context:FeatureCodecContext):Dynamic throw new ParametricError("unsupported");

  public function decode(data:Dynamic, context:FeatureCodecContext):Feature throw new ParametricError("unsupported");

  public function ownedReferences(feature:Feature):Array<TopologyReference> return [];

  public function defaultLabel(feature:Feature):String return type();
}

private class WireCodec extends ModelingCodec {
  public function new() super();

  override public function type():String return "wire";

  override public function encode(feature:Feature, context:FeatureCodecContext):Dynamic
    return {source: context.featureId((cast feature : WireFeature).source)};

  override public function decode(data:Dynamic, context:FeatureCodecContext):Feature
    return new WireFeature(context.feature(context.integer(data, "source")));
}

private class PolylineCodec extends ModelingCodec {
  public function new() super();

  override public function type():String return "polyline";

  override public function encode(feature:Feature, context:FeatureCodecContext):Dynamic {
    var value:PolylineFeature = cast feature;
    return {points: [for (point in value.points()) context.encodeVector(point)], closed: value.closed};
  }

  override public function decode(data:Dynamic, context:FeatureCodecContext):Feature
    return new PolylineFeature([for (point in context.array(data, "points")) context.decodeVector(point)], context.boolean(data, "closed"));
}

private class LoftCodec extends ModelingCodec {
  public function new() super();

  override public function type():String return "loft";

  override public function encode(feature:Feature, context:FeatureCodecContext):Dynamic {
    var value:LoftFeature = cast feature;
    return {sections: [for (section in value.dependencyFeatures()) context.featureId(section)], ruled: value.ruled};
  }

  override public function decode(data:Dynamic, context:FeatureCodecContext):Feature
    return new LoftFeature([for (id in context.array(data, "sections")) context.feature(Std.int(id))], context.boolean(data, "ruled"));
}

private class SweepCodec extends ModelingCodec {
  public function new() super();

  override public function type():String return "sweep";

  override public function encode(feature:Feature, context:FeatureCodecContext):Dynamic {
    var value:SweepFeature = cast feature;
    return {profile: context.featureId(value.profile), path: context.featureId(value.path)};
  }

  override public function decode(data:Dynamic, context:FeatureCodecContext):Feature
    return new SweepFeature(context.feature(context.integer(data, "profile")), context.feature(context.integer(data, "path")));
}

private class OffsetCodec extends ModelingCodec {
  public function new() super();

  override public function type():String return "offset";

  override public function encode(feature:Feature, context:FeatureCodecContext):Dynamic {
    var value:OffsetFeature = cast feature;
    return {source: context.featureId(value.source), distance: value.distance.value};
  }

  override public function decode(data:Dynamic, context:FeatureCodecContext):Feature
    return new OffsetFeature(context.feature(context.integer(data, "source")), context.number(data, "distance"));
}

private class ShellCodec extends ModelingCodec {
  public function new() super();

  override public function type():String return "shell";

  override public function encode(feature:Feature, context:FeatureCodecContext):Dynamic {
    var value:ShellFeature = cast feature;
    return {source: context.featureId(value.source), thickness: value.thickness.value, selection: context.encodeSelection(value.selection)};
  }

  override public function decode(data:Dynamic, context:FeatureCodecContext):Feature
    return new ShellFeature(context.feature(context.integer(data, "source")), context.number(data, "thickness"),
      context.decodeSelection(context.field(data, "selection")));
}

private class ProjectCodec extends ModelingCodec {
  public function new() super();

  override public function type():String return "project";

  override public function encode(feature:Feature, context:FeatureCodecContext):Dynamic {
    var value:ProjectFeature = cast feature;
    return {source: context.featureId(value.source), target: context.featureId(value.target),
      direction: context.encodeVector(new Vector(value.x.value, value.y.value, value.z.value))};
  }

  override public function decode(data:Dynamic, context:FeatureCodecContext):Feature
    return new ProjectFeature(context.feature(context.integer(data, "source")), context.feature(context.integer(data, "target")),
      context.decodeVector(context.field(data, "direction")));
}

private class GridCodec extends ModelingCodec {
  public function new() super();

  override public function type():String return "grid";

  override public function encode(feature:Feature, context:FeatureCodecContext):Dynamic {
    var value:GridFeature = cast feature;
    return {source: context.featureId(value.source), columns: value.columns, rows: value.rows, spacingX: value.spacingX.value,
      spacingY: value.spacingY.value};
  }

  override public function decode(data:Dynamic, context:FeatureCodecContext):Feature
    return new GridFeature(context.feature(context.integer(data, "source")), context.integer(data, "columns"), context.integer(data, "rows"),
      context.number(data, "spacingX"), context.number(data, "spacingY"));
}

private class LinearPatternCodec extends ModelingCodec {
  public function new() super();

  override public function type():String return "linear-pattern";

  override public function encode(feature:Feature, context:FeatureCodecContext):Dynamic {
    var value:LinearPatternFeature = cast feature;
    return {source: context.featureId(value.source), count: value.count.value, spacing: value.spacing.value,
      direction: context.encodeVector(value.direction), secondCount: value.secondCount.value, secondSpacing: value.secondSpacing.value,
      secondDirection: value.secondDirection == null ? null : context.encodeVector(value.secondDirection)};
  }

  override public function decode(data:Dynamic, context:FeatureCodecContext):Feature {
    var second:Dynamic = Reflect.field(data, "secondDirection");
    return new LinearPatternFeature(context.feature(context.integer(data, "source")), context.number(data, "count"),
      context.number(data, "spacing"), context.decodeVector(context.field(data, "direction")), context.number(data, "secondCount"),
      context.number(data, "secondSpacing"), second == null ? null : context.decodeVector(second));
  }
}

private class PolarPatternCodec extends ModelingCodec {
  public function new() super();

  override public function type():String return "polar-pattern";

  override public function encode(feature:Feature, context:FeatureCodecContext):Dynamic {
    var value:PolarPatternFeature = cast feature;
    return {source: context.featureId(value.source), count: value.count.value, radius: value.radius.value,
      startAngle: value.startAngle.value, angularSpan: value.angularSpan.value, axisOrigin: context.encodeVector(value.axisOrigin),
      axisDirection: context.encodeVector(value.axisDirection), radialDirection: context.encodeVector(value.radialDirection),
      orientInstances: value.orientInstances};
  }

  override public function decode(data:Dynamic, context:FeatureCodecContext):Feature
    return new PolarPatternFeature(context.feature(context.integer(data, "source")), context.number(data, "count"),
      context.number(data, "radius"), context.number(data, "angularSpan"), context.decodeVector(context.field(data, "axisOrigin")),
      context.decodeVector(context.field(data, "axisDirection")), context.decodeVector(context.field(data, "radialDirection")),
      context.boolean(data, "orientInstances"), context.number(data, "startAngle"));
}

private class HoleCodec extends ModelingCodec {
  public function new() super();

  override public function type():String return "hole";

  override public function encode(feature:Feature, context:FeatureCodecContext):Dynamic {
    var value:HoleFeature = cast feature;
    return {
      target: context.featureId(value.target), selection: context.encodeSelection(value.selection),
      xDirection: context.encodeVector(value.xDirection), offset: value.offset, flipped: value.flipped, style: value.style,
      mode: value.mode, x: value.x.value, y: value.y.value, diameter: value.diameter.value, depth: value.depth.value,
      recessDiameter: value.recessDiameter.value, recessDepth: value.recessDepth.value, includedAngle: value.includedAngle.value,
      thread: value.thread == null ? null : Json.parse(JsonWire.encode((cast value.thread : ThreadSpec).data())),
      threadDepth: value.threadDepth.value
    };
  }

  override public function decode(data:Dynamic, context:FeatureCodecContext):Feature {
    if (!Reflect.hasField(data, "thread")) throw new ParametricError("document field is missing: thread");
    var rawThread:Dynamic = Reflect.field(data, "thread");
    var thread:Null<ThreadSpecData> = rawThread == null ? null : JsonWire.decode(Json.stringify(rawThread));
    return new HoleFeature(context.feature(context.integer(data, "target")), context.decodeSelection(context.field(data, "selection")),
      context.decodeVector(context.field(data, "xDirection")), context.string(data, "style"), context.string(data, "mode"),
      context.number(data, "x"), context.number(data, "y"), context.number(data, "diameter"), context.number(data, "depth"),
      context.number(data, "recessDiameter"), context.number(data, "recessDepth"), context.number(data, "offset"),
      context.boolean(data, "flipped"), context.number(data, "includedAngle"), thread == null ? null : ThreadSpec.fromData(cast thread),
      context.number(data, "threadDepth"));
  }
}
