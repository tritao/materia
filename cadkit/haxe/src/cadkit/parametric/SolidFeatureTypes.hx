package cadkit.parametric;

import cadkit.modeling.Plane;
import cadkit.modeling.Vector;
import cadkit.parametric.features.BooleanFeature;
import cadkit.parametric.features.BooleanOperation;
import cadkit.parametric.features.ChamferFeature;
import cadkit.parametric.features.ConstrainedSketchFeature;
import cadkit.sketch.ConstrainedSketch;
import cadkit.sketch.SketchConstraint;
import cadkit.sketch.SketchEntity;
import cadkit.sketch.SketchPoint;
import cadkit.sketch.SolverSettings;
import cadkit.parametric.features.DatumSketchFeature;
import cadkit.parametric.features.DefinitionOutputFeature;
import cadkit.parametric.features.ExtrudeFeature;
import cadkit.parametric.features.FaceFeature;
import cadkit.parametric.features.FilletFeature;
import cadkit.parametric.features.ImportedShapeFeature;
import cadkit.parametric.features.LevelBoxFeature;
import cadkit.parametric.features.LevelExtrudeFeature;
import cadkit.parametric.features.MirrorFeature;
import cadkit.parametric.features.PocketFeature;
import cadkit.parametric.features.RevolveFeature;
import cadkit.parametric.features.RotationFeature;
import cadkit.parametric.features.SketchFeature;
import cadkit.parametric.features.ToolCollectionFeature;

/**
 * Codecs for CadKit's profile, solid and placement features: sketches (constrained, simple and on a datum), imported STEP, face, extrude,
 * pocket, revolve, fillet, chamfer, rotation, mirror, boolean, level-driven boxes and extrusions, definition outputs and tool
 * collections.
 */
class SolidFeatureTypes {
  public static function codecs():Array<FeatureCodec> return [new ConstrainedSketchCodec(), new SketchCodec(), new DatumSketchCodec(), new LevelExtrudeCodec(),
    new LevelBoxCodec(), new ImportedStepCodec(), new FaceCodec(), new ExtrudeCodec(), new PocketCodec(), new RevolveCodec(),
    new FilletCodec(), new ChamferCodec(), new RotationCodec(), new MirrorCodec(), new BooleanCodec(), new DefinitionOutputCodec(),
    new ToolCollectionCodec()];

  /** Edge fingerprints a fillet or chamfer keeps, or null when it selects by rule. */
  public static function encodeEdges(references:Array<TopologyReference>):Null<Array<Dynamic>>
    return references.length == 0 ? null : [for (reference in references) DocumentCodec.encodeFingerprint(reference.fingerprintData())];

  public static function decodeEdges(data:Dynamic):Null<Array<TopologyFingerprint>> {
    var value:Dynamic = Reflect.field(data, "edges");
    if (value == null) return null;
    var records:Array<Dynamic> = cast value;
    if (records.length == 0) throw new ParametricError("document edge selection must not be empty");
    return [for (record in records) DocumentCodec.decodeFingerprint(record, CadKit.ShapeKind.Edge)];
  }

  public static function optionalSelection(data:Dynamic, context:FeatureCodecContext):Null<SelectionRecipe> {
    var value:Dynamic = Reflect.field(data, "selection");
    return value == null ? null : context.decodeSelection(value);
  }
}

/** Shared parts: no owned references, the row named by the type. */
private class SolidCodec implements FeatureCodec {
  public function new() {}

  public function type():String throw new ParametricError("codec without a type");

  public function encode(feature:Feature, context:FeatureCodecContext):Dynamic throw new ParametricError("unsupported");

  public function decode(data:Dynamic, context:FeatureCodecContext):Feature throw new ParametricError("unsupported");

  public function ownedReferences(feature:Feature):Array<TopologyReference> return [];

  public function defaultLabel(feature:Feature):String return type();
}

private class SketchCodec extends SolidCodec {
  public function new() super();

  override public function type():String return "sketch";

  override public function encode(feature:Feature, context:FeatureCodecContext):Dynamic {
    var sketch:SketchFeature = cast feature;
    var data:Dynamic = {
      profile: sketch.profile, width: sketch.width.value, height: sketch.height.value,
      plane: {origin: context.encodeVector(sketch.plane.origin), xDirection: context.encodeVector(sketch.plane.xDirection),
        normal: context.encodeVector(sketch.plane.normal)}
    };
    if (sketch.radius != null) Reflect.setField(data, "radius", sketch.radius.value);
    return data;
  }

  override public function decode(data:Dynamic, context:FeatureCodecContext):Feature {
    var plane = context.field(data, "plane");
    return new SketchFeature(context.string(data, "profile"), context.number(data, "width"), context.number(data, "height"),
      new Plane(context.decodeVector(context.field(plane, "origin")), context.decodeVector(context.field(plane, "xDirection")),
        context.decodeVector(context.field(plane, "normal"))), Reflect.hasField(data, "radius") ? context.number(data, "radius") : 0);
  }
}

private class DatumSketchCodec extends SolidCodec {
  public function new() super();

  override public function type():String return "datum-sketch";

  override public function encode(feature:Feature, context:FeatureCodecContext):Dynamic {
    var sketch:DatumSketchFeature = cast feature;
    return {profile: sketch.profile, width: sketch.width.value, height: sketch.height.value, offset: sketch.offset.value,
      datum: context.encodeElementReference(sketch.datum)};
  }

  override public function decode(data:Dynamic, context:FeatureCodecContext):Feature
    return new DatumSketchFeature(context.string(data, "profile"), context.number(data, "width"), context.number(data, "height"),
      context.decodeElementReference(context.field(data, "datum")), context.number(data, "offset"));
}

private class LevelExtrudeCodec extends SolidCodec {
  public function new() super();

  override public function type():String return "level-extrude";

  override public function encode(feature:Feature, context:FeatureCodecContext):Dynamic {
    var extrusion:LevelExtrudeFeature = cast feature;
    return {source: context.featureId(extrusion.source), base: context.encodeElementReference(extrusion.base),
      top: context.encodeElementReference(extrusion.top), baseOffset: extrusion.baseOffset.value, topOffset: extrusion.topOffset.value};
  }

  override public function decode(data:Dynamic, context:FeatureCodecContext):Feature
    return new LevelExtrudeFeature(context.feature(context.integer(data, "source")), context.decodeElementReference(context.field(data, "base")),
      context.decodeElementReference(context.field(data, "top")), context.number(data, "baseOffset"), context.number(data, "topOffset"));
}

private class LevelBoxCodec extends SolidCodec {
  public function new() super();

  override public function type():String return "level-box";

  override public function encode(feature:Feature, context:FeatureCodecContext):Dynamic {
    var wall:LevelBoxFeature = cast feature;
    return {width: wall.width.value, depth: wall.depth.value, base: context.encodeElementReference(wall.base),
      top: context.encodeElementReference(wall.top), baseOffset: wall.baseOffset.value, topOffset: wall.topOffset.value};
  }

  override public function decode(data:Dynamic, context:FeatureCodecContext):Feature
    return new LevelBoxFeature(context.number(data, "width"), context.number(data, "depth"), context.decodeElementReference(context.field(data, "base")),
      context.decodeElementReference(context.field(data, "top")), context.number(data, "baseOffset"), context.number(data, "topOffset"));
}

private class ImportedStepCodec extends SolidCodec {
  public function new() super();

  override public function type():String return "imported-step";

  override public function encode(feature:Feature, context:FeatureCodecContext):Dynamic
    return {stepText: (cast feature : ImportedShapeFeature).stepText};

  override public function decode(data:Dynamic, context:FeatureCodecContext):Feature
    return new ImportedShapeFeature(context.string(data, "stepText"));
}

private class FaceCodec extends SolidCodec {
  public function new() super();

  override public function type():String return "face";

  override public function encode(feature:Feature, context:FeatureCodecContext):Dynamic {
    var face:FaceFeature = cast feature;
    return {source: context.featureId(face.source), index: face.index, fingerprint: DocumentCodec.encodeFingerprint(face.fingerprintData())};
  }

  override public function decode(data:Dynamic, context:FeatureCodecContext):Feature {
    var fingerprint:Dynamic = Reflect.field(data, "fingerprint");
    return new FaceFeature(context.feature(context.integer(data, "source")), context.integer(data, "index"),
      fingerprint == null ? null : DocumentCodec.decodeFingerprint(fingerprint, CadKit.ShapeKind.Face));
  }
}

private class ExtrudeCodec extends SolidCodec {
  public function new() super();

  override public function type():String return "extrude";

  override public function encode(feature:Feature, context:FeatureCodecContext):Dynamic {
    var extrude:ExtrudeFeature = cast feature;
    return {source: context.featureId(extrude.source), x: extrude.x.value, y: extrude.y.value, z: extrude.z.value,
      amount: extrude.amount == null ? null : extrude.amount.value, reversed: extrude.reversed, symmetric: extrude.symmetric};
  }

  override public function decode(data:Dynamic, context:FeatureCodecContext):Feature {
    var source = context.feature(context.integer(data, "source"));
    var x = context.number(data, "x"), y = context.number(data, "y"), z = context.number(data, "z");
    if (Reflect.field(data, "amount") == null) return new ExtrudeFeature(source, x, y, z);
    return ExtrudeFeature.along(source, context.number(data, "amount"), new Vector(x, y, z), context.boolean(data, "reversed"),
      context.boolean(data, "symmetric"));
  }
}

private class PocketCodec extends SolidCodec {
  public function new() super();

  override public function type():String return "pocket";

  override public function encode(feature:Feature, context:FeatureCodecContext):Dynamic {
    var pocket:PocketFeature = cast feature;
    return {target: context.featureId(pocket.target), profile: context.featureId(pocket.profile), mode: pocket.mode, depth: pocket.depth.value};
  }

  override public function decode(data:Dynamic, context:FeatureCodecContext):Feature
    return new PocketFeature(context.feature(context.integer(data, "target")), context.feature(context.integer(data, "profile")),
      context.string(data, "mode"), context.number(data, "depth"));
}

private class RevolveCodec extends SolidCodec {
  public function new() super();

  override public function type():String return "revolve";

  override public function encode(feature:Feature, context:FeatureCodecContext):Dynamic {
    var revolve:RevolveFeature = cast feature;
    return {source: context.featureId(revolve.source), originX: revolve.originX.value, originY: revolve.originY.value,
      originZ: revolve.originZ.value, axisX: revolve.axisX.value, axisY: revolve.axisY.value, axisZ: revolve.axisZ.value,
      angle: revolve.angle.value};
  }

  override public function decode(data:Dynamic, context:FeatureCodecContext):Feature
    return new RevolveFeature(context.feature(context.integer(data, "source")), context.number(data, "originX"),
      context.number(data, "originY"), context.number(data, "originZ"), context.number(data, "axisX"), context.number(data, "axisY"),
      context.number(data, "axisZ"), context.number(data, "angle"));
}

private class FilletCodec extends SolidCodec {
  public function new() super();

  override public function type():String return "fillet";

  override public function ownedReferences(feature:Feature):Array<TopologyReference> return (cast feature : FilletFeature).edgeReferences;

  override public function encode(feature:Feature, context:FeatureCodecContext):Dynamic {
    var fillet:FilletFeature = cast feature;
    return {source: context.featureId(fillet.source), radius: fillet.radius.value, selection: context.encodeSelection(fillet.selection),
      edges: SolidFeatureTypes.encodeEdges(fillet.edgeReferences)};
  }

  override public function decode(data:Dynamic, context:FeatureCodecContext):Feature
    return new FilletFeature(context.feature(context.integer(data, "source")), context.number(data, "radius"), null,
      SolidFeatureTypes.decodeEdges(data), SolidFeatureTypes.optionalSelection(data, context));
}

private class ChamferCodec extends SolidCodec {
  public function new() super();

  override public function type():String return "chamfer";

  override public function ownedReferences(feature:Feature):Array<TopologyReference> return (cast feature : ChamferFeature).edgeReferences;

  override public function encode(feature:Feature, context:FeatureCodecContext):Dynamic {
    var chamfer:ChamferFeature = cast feature;
    return {source: context.featureId(chamfer.source), distance: chamfer.distance.value,
      selection: context.encodeSelection(chamfer.selection), edges: SolidFeatureTypes.encodeEdges(chamfer.edgeReferences)};
  }

  override public function decode(data:Dynamic, context:FeatureCodecContext):Feature
    return new ChamferFeature(context.feature(context.integer(data, "source")), context.number(data, "distance"), null,
      SolidFeatureTypes.decodeEdges(data), SolidFeatureTypes.optionalSelection(data, context));
}

private class RotationCodec extends SolidCodec {
  public function new() super();

  override public function type():String return "rotation";

  override public function encode(feature:Feature, context:FeatureCodecContext):Dynamic {
    var rotation:RotationFeature = cast feature;
    return {source: context.featureId(rotation.source), pivot: context.encodeVector(rotation.pivot), axis: context.encodeVector(rotation.axis),
      angle: rotation.angle.value};
  }

  override public function decode(data:Dynamic, context:FeatureCodecContext):Feature
    return new RotationFeature(context.feature(context.integer(data, "source")), context.decodeVector(context.field(data, "pivot")),
      context.decodeVector(context.field(data, "axis")), context.number(data, "angle"));
}

private class MirrorCodec extends SolidCodec {
  public function new() super();

  override public function type():String return "mirror";

  override public function encode(feature:Feature, context:FeatureCodecContext):Dynamic {
    var mirror:MirrorFeature = cast feature;
    return {source: context.featureId(mirror.source), planeOrigin: context.encodeVector(mirror.planeOrigin),
      planeNormal: context.encodeVector(mirror.planeNormal), mode: mirror.mode};
  }

  override public function decode(data:Dynamic, context:FeatureCodecContext):Feature
    return new MirrorFeature(context.feature(context.integer(data, "source")), context.decodeVector(context.field(data, "planeOrigin")),
      context.decodeVector(context.field(data, "planeNormal")), context.string(data, "mode"));
}

private class BooleanCodec extends SolidCodec {
  public function new() super();

  override public function type():String return "boolean";

  override public function encode(feature:Feature, context:FeatureCodecContext):Dynamic {
    var value:BooleanFeature = cast feature;
    var operation = switch value.operation {
      case Fuse: "fuse";
      case Cut: "cut";
      case Common: "common";
    };
    return {first: context.featureId(value.first), second: context.featureId(value.second), operation: operation};
  }

  override public function decode(data:Dynamic, context:FeatureCodecContext):Feature {
    var operation = switch context.string(data, "operation") {
      case "fuse": BooleanOperation.Fuse;
      case "cut": BooleanOperation.Cut;
      case "common": BooleanOperation.Common;
      case other: throw new ParametricError("unsupported boolean operation: " + other);
    };
    return new BooleanFeature(context.feature(context.integer(data, "first")), context.feature(context.integer(data, "second")), operation);
  }
}

private class DefinitionOutputCodec extends SolidCodec {
  public function new() super();

  override public function type():String return "definition-output";

  override public function encode(feature:Feature, context:FeatureCodecContext):Dynamic {
    var output:DefinitionOutputFeature = cast feature;
    return {instance: context.encodeElementReference(output.instance), output: output.outputName};
  }

  override public function decode(data:Dynamic, context:FeatureCodecContext):Feature
    return new DefinitionOutputFeature(context.decodeElementReference(context.field(data, "instance")), context.string(data, "output"));
}

private class ToolCollectionCodec extends SolidCodec {
  public function new() super();

  override public function type():String return "tool-collection";

  override public function encode(feature:Feature, context:FeatureCodecContext):Dynamic {
    var collection:ToolCollectionFeature = cast feature;
    return {tools: [for (tool in collection.tools) context.featureId(tool)], labels: collection.labels.copy()};
  }

  override public function decode(data:Dynamic, context:FeatureCodecContext):Feature {
    var labels:Array<String> = [];
    for (label in context.array(data, "labels")) {
      if (!Std.isOfType(label, String)) throw new ParametricError("document tool label is not a string");
      labels.push(cast label);
    }
    return new ToolCollectionFeature([for (id in context.array(data, "tools")) context.feature(Std.int(id))], labels);
  }
}

private class ConstrainedSketchCodec extends SolidCodec {
  public function new() super();

  override public function type():String return "constrained-sketch";

  override public function ownedReferences(feature:Feature):Array<TopologyReference> {
    var support = (cast feature : ConstrainedSketchFeature).supportFaceReference;
    return support == null ? [] : [support];
  }

  override public function encode(feature:Feature, context:FeatureCodecContext):Dynamic {
    var value:ConstrainedSketchFeature = cast feature;
    var sketch = value.sketch();
    var constraints:Array<Dynamic> = [];
    for (constraint in sketch.constraints()) {
      var number = constraint.value;
      // A driving dimension saves its current value, in the sketch's units.
      if (!constraint.reference && SketchConstraint.isLengthDimension(constraint.kind))
        number = UnitConversion.fromCanonical(value.dimension(constraint.id).value, ParameterKind.Length, sketch.units);
      else if (!constraint.reference && constraint.kind == "angle") number = value.dimension(constraint.id).value;
      var record:Dynamic = {id: constraint.id, kind: constraint.kind, first: constraint.first, second: constraint.second,
        third: constraint.third, value: number};
      if (constraint.reference) Reflect.setField(record, "reference", true);
      constraints.push(record);
    }
    return {
      units: sketch.units,
      plane: {origin: context.encodeVector(sketch.plane.origin), xDirection: context.encodeVector(sketch.plane.xDirection),
        normal: context.encodeVector(sketch.plane.normal)},
      settings: {tolerance: sketch.settings.tolerance, rankTolerance: sketch.settings.rankTolerance,
        maxIterations: sketch.settings.maxIterations, initialDamping: sketch.settings.initialDamping},
      points: [for (point in sketch.points()) {id: point.id, x: point.x, y: point.y}],
      entities: [for (entity in sketch.entities()) {id: entity.id, kind: entity.kind, first: entity.first, second: entity.second,
        radius: entity.radius, startAngle: entity.startAngle, endAngle: entity.endAngle, clockwise: entity.clockwise,
        construction: entity.construction}],
      constraints: constraints,
      support: value.support == null ? null : context.featureId(value.support),
      supportSelection: context.encodeSelection(value.supportSelection),
      supportFaceFingerprint: value.supportFaceReference == null ? null
        : DocumentCodec.encodeFingerprint(value.supportFaceReference.fingerprintData()),
      supportXDirection: value.supportXDirection == null ? null : context.encodeVector(value.supportXDirection),
      supportOffset: value.supportOffset,
      supportFlipped: value.supportFlipped
    };
  }

  static function optionalString(record:Dynamic, name:String):Null<String> {
    var value:Dynamic = Reflect.field(record, name);
    if (value == null) return null;
    if (!Std.isOfType(value, String)) throw new ParametricError('document field is not a string: $name');
    return cast value;
  }

  override public function decode(data:Dynamic, context:FeatureCodecContext):Feature {
    var plane = context.field(data, "plane"), settings = context.field(data, "settings");
    var sketch = new ConstrainedSketch(new Plane(context.decodeVector(context.field(plane, "origin")),
      context.decodeVector(context.field(plane, "xDirection")), context.decodeVector(context.field(plane, "normal"))),
      context.string(data, "units"), new SolverSettings(context.number(settings, "tolerance"), context.number(settings, "rankTolerance"),
        context.integer(settings, "maxIterations"), context.number(settings, "initialDamping")));
    for (point in context.array(data, "points"))
      sketch.addPoint(new SketchPoint(context.string(point, "id"), context.number(point, "x"), context.number(point, "y")));
    for (value in context.array(data, "entities")) sketch.addEntity(switch context.string(value, "kind") {
      case "line": SketchEntity.line(context.string(value, "id"), context.string(value, "first"), context.string(value, "second"),
          context.boolean(value, "construction"));
      case "circle": SketchEntity.circle(context.string(value, "id"), context.string(value, "first"), context.number(value, "radius"),
          context.boolean(value, "construction"));
      case "arc": SketchEntity.arc(context.string(value, "id"), context.string(value, "first"), context.number(value, "radius"),
          context.number(value, "startAngle"), context.number(value, "endAngle"), context.boolean(value, "clockwise"),
          context.boolean(value, "construction"));
      case _: throw new ParametricError("unsupported constrained sketch entity");
    });
    for (value in context.array(data, "constraints")) {
      var constraint = SketchConstraint.raw(context.string(value, "id"), context.string(value, "kind"), context.string(value, "first"),
        optionalString(value, "second"), optionalString(value, "third"), context.number(value, "value"));
      var reference:Null<Bool> = Reflect.field(value, "reference");
      sketch.addConstraint(reference == true ? constraint.asReference() : constraint);
    }
    var support:Dynamic = Reflect.field(data, "support");
    if (support == null) return new ConstrainedSketchFeature(sketch);
    var selection:Dynamic = Reflect.field(data, "supportSelection");
    var face:Dynamic = Reflect.field(data, "supportFaceFingerprint");
    return new ConstrainedSketchFeature(sketch, context.feature(context.integer(data, "support")),
      selection == null ? null : context.decodeSelection(selection), context.decodeVector(context.field(data, "supportXDirection")),
      context.number(data, "supportOffset"), context.boolean(data, "supportFlipped"), null,
      face == null ? null : DocumentCodec.decodeFingerprint(face, CadKit.ShapeKind.Face));
  }
}
