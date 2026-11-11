package cadkit.sheetmetal;

import CadKit;
import haxe.Json;
import haxeon.wire.JsonWire;
import cadkit.parametric.Feature;
import cadkit.parametric.FeatureCodec;
import cadkit.parametric.FeatureCodecContext;
import cadkit.parametric.ParametricError;
import cadkit.parametric.TopologyFingerprint;
import cadkit.parametric.TopologyReference;
import cadkit.parametric.features.ConstrainedSketchFeature;
import cadkit.sheetmetal.SheetFeatureRecord.SheetBaseRecord;
import cadkit.sheetmetal.SheetFeatureRecord.SheetCornerRecord;
import cadkit.sheetmetal.SheetFeatureRecord.SheetCutRecord;
import cadkit.sheetmetal.SheetFeatureRecord.SheetEdgeRecord;
import cadkit.sheetmetal.SheetFeatureRecord.SheetFaceRecord;
import cadkit.sheetmetal.SheetFeatureRecord.SheetFlangeRecord;
import cadkit.sheetmetal.SheetFeatureRecord.SheetHemRecord;
import cadkit.sheetmetal.SheetFeatureRecord.SheetRipRecord;
import cadkit.sheetmetal.SheetFeatureRecord.SheetSavedBendRule;
import cadkit.sheetmetal.SheetFeatureRecord.SheetStockRecord;
import cadkit.sheetmetal.process.ProcessBendRule;

/**
 * Sheet-metal document features, saved through `FeatureTypeRegistry` like CadKit's other added types (`CadKitFeatureTypes`).
 * Each payload is the typed `SheetFeatureRecord`, validated by `JsonWire` before a feature is constructed.
 */
class SheetFeatureTypes {
  /** The six sheet feature types; `CadKitFeatureTypes` registers them with CadKit's other types. */
  public static function codecs():Array<FeatureCodec> return [
    new SheetBaseCodec(),
    new SheetEdgeFlangeCodec(),
    new SheetHemCodec(),
    new SheetRipCodec(),
    new SheetCornerCodec(),
    new SheetCutCodec(),
    new SheetHolesCodec()
  ];

  /** A sheet feature dependency of the expected class, or a named error. */
  public static function sheetSource(context:FeatureCodecContext, id:Int, what:String):SheetFeature {
    var source = Std.downcast(context.feature(id), SheetFeature);
    if (source == null) throw new ParametricError('$what source must be a sheet feature');
    return source;
  }

  public static function wire(data:Dynamic):String return Json.stringify(data);

  public static function payload(encoded:String):Dynamic return Json.parse(encoded);

  public static function encodeFace(face:TopologyFingerprint):SheetFaceRecord {
    if (face.kind != CadKit.ShapeKind.Face || face.surfaceKind != CadKit
      .SurfaceKind.Plane) throw new ParametricError("sheet flange requires a planar wall-side face fingerprint");
    return {
      surface: "plane",
      curve: "unknown",
      x: face.x,
      y: face.y,
      z: face.z,
      dx: face.dx,
      dy: face.dy,
      dz: face.dz,
      measure: face.measure,
      name: face.name,
      naming: face.name == null ? null : cadkit.Shape.namingScheme()
    };
  }

  /** Named edges are saved only when the feature chose its edges by name. */
  public static function encodeEdges(feature:SheetEdgeFeature):Null<Array<SheetEdgeRecord>> {
    var named = feature.namedEdges();
    return named.length == 0 ? null : [for (edge in named) {wall: edge.wall, identity: edge.identity}];
  }

  public static function decodeEdges(edges:Null<Array<SheetEdgeRecord>>):Null<Array<SheetEdgeFeature.SheetSelectedEdge>>
    return edges == null ? null : [for (edge in edges) {wall: edge.wall, identity: edge.identity}];

  public static function decodeFaces(faces:Array<SheetFaceRecord>, edges:Null<Array<SheetEdgeRecord>>):Null<Array<TopologyFingerprint>>
    return edges == null ? [for (face in faces) decodeFace(face)] : null;

  public static function decodeFace(face:SheetFaceRecord):TopologyFingerprint {
    if (face.surface != "plane" || face.curve
      != "unknown") throw new ParametricError("sheet flange requires a planar wall-side face fingerprint");
    var name = face.naming == cadkit.Shape.namingScheme() ? face.name : null;
    return TopologyFingerprint.fromData(
      CadKit.ShapeKind.Face,
      CadKit.SurfaceKind.Plane,
      CadKit.CurveKind.Unknown,
      face.x,
      face.y,
      face.z,
      face.dx,
      face.dy,
      face.dz,
      face.measure,
      false,
      name
    );
  }

  public static function encodeStock(stock:SheetMaterial):SheetStockRecord {
    var rule:Null<SheetSavedBendRule> = null;
    if (stock.bendRule != null) switch (stock.bendRule) {
      case KFactor(table):
        rule = SheetSavedBendRule.KFactor(table.rows());
      case Process(setup):
        rule = SheetSavedBendRule.Process(setup.record());
    }
    return {
      id: stock.id,
      rule: rule,
      mechanical: stock.mechanical,
      defaultRadiusRatio: stock.defaultRadiusRatio,
      minRadiusRatio: stock.minRadiusRatio,
      grainAngle: stock.grainAngle
    };
  }

  public static function decodeStock(record:SheetStockRecord, thickness:Float):SheetMaterial {
    var rule:Null<BendRule> = null;
    if (record.rule != null) switch (record.rule) {
      case KFactor(rows):
        rule = BendRule.KFactor(new KFactorTable([for (row in rows) row.ratio], [for (row in rows) row.k]));
      case Process(setup):
        rule = BendRule.Process(ProcessBendRule.fromRecord(setup));
    }
    return new SheetMaterial(
      record.id,
      thickness,
      rule,
      record.mechanical,
      record.defaultRadiusRatio,
      record.minRadiusRatio,
      record.grainAngle
    );
  }
}

/** Shared parts of the sheet codecs: no owned references unless the feature stores face fingerprints, rows named by type. */
private class SheetCodec implements FeatureCodec {
  public function new() {}

  public function type():String throw new ParametricError("sheet codec without a type");

  public function encode(feature:Feature, context:FeatureCodecContext):Dynamic throw new ParametricError("unsupported");

  public function decode(data:Dynamic, context:FeatureCodecContext):Feature throw new ParametricError("unsupported");

  public function ownedReferences(feature:Feature):Array<TopologyReference> return [];

  public function defaultLabel(feature:Feature):String return type();
}

private class SheetBaseCodec extends SheetCodec {
  public function new() super();

  override public function type():String return "sheet-base";

  override public function encode(feature:Feature, context:FeatureCodecContext):Dynamic {
    var base = Std.downcast(feature, SheetBaseFeature);
    if (base == null) throw new ParametricError("sheet-base has the wrong feature class");
    var data:SheetBaseRecord = {
      source: context.featureId(base.source),
      thickness: base.thickness.value,
      side: base.side,
      material: SheetFeatureTypes.encodeStock(base.stock()),
      identities: {nextWall: base.identityLedger().wallCounter(), nextBend: base.identityLedger().bendCounter()}
    };
    return SheetFeatureTypes.payload(JsonWire.encode(data));
  }

  override public function decode(raw:Dynamic, context:FeatureCodecContext):Feature {
    var data:SheetBaseRecord = JsonWire.decode(SheetFeatureTypes.wire(raw));
    var source = Std.downcast(context.feature(data.source), ConstrainedSketchFeature);
    if (source == null) throw new ParametricError("sheet base source must be a constrained sketch");
    return new SheetBaseFeature(
      source,
      SheetFeatureTypes.decodeStock(data.material, data.thickness),
      data.side,
      new SheetIdentityLedger(data.identities.nextWall, data.identities.nextBend)
    );
  }
}

private class SheetEdgeFlangeCodec extends SheetCodec {
  public function new() super();

  override public function type():String return "sheet-edge-flange";

  override public function ownedReferences(feature:Feature):Array<TopologyReference> {
    var flange = Std.downcast(feature, SheetEdgeFeature);
    if (flange == null) throw new ParametricError("sheet edge feature has the wrong feature class");
    return flange.faceReferences();
  }

  override public function encode(feature:Feature, context:FeatureCodecContext):Dynamic {
    var flange = Std.downcast(feature, SheetEdgeFlangeFeature);
    if (flange == null) throw new ParametricError("sheet-edge-flange has the wrong feature class");
    var data:SheetFlangeRecord = {
      source: context.featureId(flange.source),
      angle: flange.angle.value,
      length: flange.length.value,
      radius: flange.useMaterialRadius ? null : flange.radius.value,
      lengthSpec: flange.lengthSpec,
      position: flange.position,
      offset: flange.offset.value,
      gap1: flange.gap1.value,
      gap2: flange.gap2.value,
      extend1: flange.extend1.value,
      extend2: flange.extend2.value,
      walls: flange.wallIds(),
      bends: flange.bendIds(),
      faces: [for (reference in flange.faceReferences()) SheetFeatureTypes.encodeFace(reference.fingerprintData())],
      reliefType: flange.reliefType,
      reliefSizing: flange.reliefSizing,
      reliefWidth: flange.reliefWidth.value,
      reliefDepth: flange.reliefDepth.value,
      edges: SheetFeatureTypes.encodeEdges(flange)
    };
    return SheetFeatureTypes.payload(JsonWire.encode(data));
  }

  override public function decode(raw:Dynamic, context:FeatureCodecContext):Feature {
    var data:SheetFlangeRecord = JsonWire.decode(SheetFeatureTypes.wire(raw));
    return new SheetEdgeFlangeFeature(
      SheetFeatureTypes.sheetSource(context, data.source, "flange"),
      null,
      data.length,
      data.angle,
      data.radius,
      data.lengthSpec,
      data.position,
      data.offset,
      data.gap1,
      data.gap2,
      data.extend1,
      data.extend2,
      SheetFeatureTypes.decodeFaces(data.faces, data.edges),
      data.walls,
      data.bends,
      data.reliefType,
      data.reliefSizing,
      data.reliefWidth,
      data.reliefDepth,
      SheetFeatureTypes.decodeEdges(data.edges)
    );
  }
}

private class SheetHemCodec extends SheetCodec {
  public function new() super();

  override public function type():String return "sheet-hem";

  override public function ownedReferences(feature:Feature):Array<TopologyReference> {
    var hem = Std.downcast(feature, SheetEdgeFeature);
    if (hem == null) throw new ParametricError("sheet edge feature has the wrong feature class");
    return hem.faceReferences();
  }

  override public function encode(feature:Feature, context:FeatureCodecContext):Dynamic {
    var hem = Std.downcast(feature, SheetHemFeature);
    if (hem == null) throw new ParametricError("sheet-hem has the wrong feature class");
    var data:SheetHemRecord = {
      source: context.featureId(hem.source),
      length: hem.length.value,
      spec: hem.authoredSpec(),
      side: hem.side,
      walls: hem.wallIds(),
      bends: hem.bendIds(),
      faces: [for (reference in hem.faceReferences()) SheetFeatureTypes.encodeFace(reference.fingerprintData())],
      edges: SheetFeatureTypes.encodeEdges(hem)
    };
    return SheetFeatureTypes.payload(JsonWire.encode(data));
  }

  override public function decode(raw:Dynamic, context:FeatureCodecContext):Feature {
    var data:SheetHemRecord = JsonWire.decode(SheetFeatureTypes.wire(raw));
    return new SheetHemFeature(
      SheetFeatureTypes.sheetSource(context, data.source, "hem"),
      null,
      data.spec,
      data.length,
      data.side,
      SheetFeatureTypes.decodeFaces(data.faces, data.edges),
      data.walls,
      data.bends,
      SheetFeatureTypes.decodeEdges(data.edges)
    );
  }
}

private class SheetRipCodec extends SheetCodec {
  public function new() super();

  override public function type():String return "sheet-rip";

  override public function encode(feature:Feature, context:FeatureCodecContext):Dynamic {
    var rip = Std.downcast(feature, SheetRipFeature);
    if (rip == null) throw new ParametricError("sheet-rip has the wrong feature class");
    var data:SheetRipRecord = {source: context.featureId(rip.source), bend: rip.bend};
    return SheetFeatureTypes.payload(JsonWire.encode(data));
  }

  override public function decode(raw:Dynamic, context:FeatureCodecContext):Feature {
    var data:SheetRipRecord = JsonWire.decode(SheetFeatureTypes.wire(raw));
    return new SheetRipFeature(SheetFeatureTypes.sheetSource(context, data.source, "rip"), data.bend);
  }
}

private class SheetCornerCodec extends SheetCodec {
  public function new() super();

  override public function type():String return "sheet-corner";

  override public function encode(feature:Feature, context:FeatureCodecContext):Dynamic {
    var corner = Std.downcast(feature, SheetCornerFeature);
    if (corner == null) throw new ParametricError("sheet-corner has the wrong feature class");
    var data:SheetCornerRecord = {
      source: context.featureId(corner.source),
      firstWall: corner.first.wall,
      firstEdge: corner.first.identity,
      secondWall: corner.second.wall,
      secondEdge: corner.second.identity,
      join: corner.join,
      gap: corner.gap.value,
      overlap: corner.overlap.value,
      relief: corner.relief,
      reliefSize: corner.reliefSize.value
    };
    return SheetFeatureTypes.payload(JsonWire.encode(data));
  }

  override public function decode(raw:Dynamic, context:FeatureCodecContext):Feature {
    var data:SheetCornerRecord = JsonWire.decode(SheetFeatureTypes.wire(raw));
    return new SheetCornerFeature(
      SheetFeatureTypes.sheetSource(context, data.source, "corner"),
      new SheetCornerEdge(data.firstWall, data.firstEdge),
      new SheetCornerEdge(data.secondWall, data.secondEdge),
      data.join,
      data.gap,
      data.overlap,
      data.relief,
      data.reliefSize
    );
  }
}

private class SheetCutCodec extends SheetCodec {
  public function new() super();

  override public function type():String return "sheet-cut";

  override public function encode(feature:Feature, context:FeatureCodecContext):Dynamic {
    var cut = Std.downcast(feature, SheetCutFeature);
    if (cut == null) throw new ParametricError("sheet-cut has the wrong feature class");
    var data:SheetCutRecord = {
      source: context.featureId(cut.source),
      profile: context.featureId(cut.profile),
      wall: cut.wall,
      side: cut.side
    };
    return SheetFeatureTypes.payload(JsonWire.encode(data));
  }

  override public function decode(raw:Dynamic, context:FeatureCodecContext):Feature {
    var data:SheetCutRecord = JsonWire.decode(SheetFeatureTypes.wire(raw));
    var source = Std.downcast(context.feature(data.source), SheetFeature);
    var profile = Std.downcast(context.feature(data.profile), ConstrainedSketchFeature);
    if (source == null || profile == null) throw new ParametricError("sheet cut requires a sheet source and constrained sketch");
    return new SheetCutFeature(source, profile, data.wall, data.side);
  }
}

private class SheetHolesCodec extends SheetCodec {
  public function new() super();

  override public function type():String return "sheet-holes";

  override public function encode(feature:Feature, context:FeatureCodecContext):Dynamic {
    var drilled = Std.downcast(feature, SheetHolesFeature);
    if (drilled == null) throw new ParametricError("sheet-holes has the wrong feature class");
    var specs:Array<cadkit.parametric.HoleSpec> = drilled.holes;
    return {source: context.featureId(drilled.source), holes: Json.parse(JsonWire.encode(specs))};
  }

  override public function decode(data:Dynamic, context:FeatureCodecContext):Feature {
    var specs:Array<cadkit.parametric.HoleSpec> = JsonWire.decode(Json.stringify(Reflect.field(data, "holes")));
    return new SheetHolesFeature(SheetFeatureTypes.sheetSource(context, Std.int(context.number(data, "source")), "holes"), specs);
  }

  override public function defaultLabel(feature:Feature):String return "Holes";
}
