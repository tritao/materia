import cadkit.Shape;
import cadkit.parametric.Document;
import cadkit.parametric.DocumentCodec;
import cadkit.parametric.EvaluationContext;
import cadkit.parametric.EvaluationResult;
import cadkit.parametric.Feature;
import cadkit.parametric.FeatureCodec;
import cadkit.parametric.FeatureCodecContext;
import cadkit.parametric.FeatureId;
import cadkit.parametric.FeatureTypeRegistry;
import cadkit.parametric.Parameter;
import cadkit.parametric.ParameterKind;
import cadkit.parametric.TimelineModel;
import cadkit.parametric.TopologyReference;
import cadkit.parametric.features.BoxFeature;

/** A feature type defined outside CadKit: a vertical drill at the source's origin corner, `depth` deep. */
class CornerDrillFeature extends Feature {
  public final source:Feature;
  public final radius:Parameter;
  public final depth:Parameter;
  public final note:String;

  public function new(source:Feature, radius:Float, depth:Float, note:String) {
    super();
    this.source = source;
    this.note = note;
    this.radius = new Parameter(this, "drill.radius", radius, 0.0, false, 1e300, ParameterKind.Length);
    this.depth = new Parameter(this, "drill.depth", depth, 0.0, false, 1e300, ParameterKind.Length);
  }

  override public function serializationType():String return "test-corner-drill";

  override public function dependencies():Array<FeatureId> return [source.id];

  override public function dependencyFeatures():Array<Feature> return [source];

  override public function evaluate(context:EvaluationContext):EvaluationResult {
    var tool = Shape.cylinder(radius.value, depth.value);
    var result = context.shape(source).cutOperation(tool);
    tool.close();
    return EvaluationResult.fromOperation(result);
  }
}

class CornerDrillCodec implements FeatureCodec {
  public function new() {}

  public function type():String return "test-corner-drill";

  public function encode(feature:Feature, context:FeatureCodecContext):Dynamic {
    var drill:CornerDrillFeature = cast feature;
    return {source: context.featureId(drill.source), radius: drill.radius.value, depth: drill.depth.value, note: drill.note};
  }

  public function decode(data:Dynamic, context:FeatureCodecContext):Feature return new CornerDrillFeature(
    context.feature(Std.int(context.number(data, "source"))),
    context.number(data, "radius"),
    context.number(data, "depth"),
    context.string(data, "note")
  );

  public function ownedReferences(feature:Feature):Array<TopologyReference> return [];

  public function defaultLabel(feature:Feature):String return "Corner drill";
}

/** Same type name as `CornerDrillCodec`, a different codec: must be refused. */
class RivalCodec extends CornerDrillCodec {
  public function new() super();
}

/** Tries to take a sheet-metal name, which CadKit registers itself. */
class SheetNameCodec extends CornerDrillCodec {
  public function new() super();
  override public function type():String return "sheet-base";
}

/** Tries to take a built-in name. */
class BoxCodec extends CornerDrillCodec {
  public function new() super();
  override public function type():String return "box";
}

/** FEATURE_PARTS_PLAN FP1a: a feature type registered from outside CadKit saves, restores, recomputes and names its row. */
class FeatureRegistrySmoke {
  static function check(value:Bool, message:String):Void {
    if (!value) throw "FeatureRegistrySmoke: " + message;
  }

  static function throws(action:() -> Void, expected:String, what:String):Void {
    var message:Null<String> = null;
    try action() catch (error:Dynamic) message = Std.string(error);
    check(message != null, what + " did not throw");
    check(message.indexOf(expected) >= 0, what + ' threw "$message", expected "$expected"');
  }

  static function expectedVolume(width:Float, depth:Float, height:Float, radius:Float):Float
    return width * depth * height - Math.PI * radius * radius * height / 4;

  public static function run():Void {
    // Unknown types fail to decode, naming the type; this runs before registration on purpose.
    var probe = new Document();
    var box = probe.add(new BoxFeature(20, 20, 10));
    probe.add(new CornerDrillFeature(box, 4, 10, "probe"));
    throws(() -> DocumentCodec.encode(probe), "unsupported feature type: test-corner-drill", "encoding an unregistered type");
    probe.close();

    var codec = new CornerDrillCodec();
    FeatureTypeRegistry.register(codec);
    FeatureTypeRegistry.register(codec);
    check(FeatureTypeRegistry.isRegistered("test-corner-drill"), "type is registered");
    throws(() -> FeatureTypeRegistry.register(new RivalCodec()), "already registered: test-corner-drill", "a second codec");
    throws(() -> FeatureTypeRegistry.register(new BoxCodec()), "feature type is built in: box", "a built-in name");

    // Sheet metal is not built into the document core: its six types arrive through the registry.
    for (type in ["sheet-base", "sheet-edge-flange", "sheet-hem", "sheet-rip", "sheet-corner", "sheet-cut"]) {
      check(!DocumentCodec.isBuiltInType(type), '$type is not built in');
      check(FeatureTypeRegistry.isRegistered(type), '$type is registered');
    }
    throws(() -> FeatureTypeRegistry.register(new SheetNameCodec()), "already registered: sheet-base", "a sheet-metal name");

    var document = new Document();
    var plate = document.add(new BoxFeature(20, 20, 10));
    var drill = document.add(new CornerDrillFeature(plate, 4, 10, "keeps its own data"));
    var radius = document.defineParameter("drill.r", 4);
    radius.bind(drill.parameter("drill.radius"));
    document.setOutput(drill);
    document.recompute();
    var volume = document.outputFeature().currentShape().volume();
    check(Math.abs(volume - expectedVolume(20, 20, 10, 4)) < 1e-6, 'drilled volume $volume');

    var rows = TimelineModel.of(document);
    check(rows.filter(row -> row.label == "Corner drill").length == 1, "the history row uses the codec's label");

    var text = DocumentCodec.encode(document);
    check(text.indexOf('"type":"test-corner-drill"') >= 0, "saved under its type");
    check(text.indexOf('"note":"keeps its own data"') >= 0, "payload saved");
    document.close();

    var restored = DocumentCodec.decode(text);
    var restoredDrill = Std.downcast(restored.outputFeature(), CornerDrillFeature);
    if (restoredDrill == null) throw "FeatureRegistrySmoke: not restored as the registered class";
    check(restoredDrill.note == "keeps its own data", "payload restored");
    check(restoredDrill.source == restored.featureAt(0), "dependency restored by id");
    var again = restored.outputFeature().currentShape().volume();
    check(Math.abs(again - volume) < 1e-6, 'restored volume $again vs $volume');
    check(DocumentCodec.encode(restored) == text, "save after restore is identical");

    // The named parameter binding survives the round trip and drives the registered feature.
    restored.parameter("drill.r").set(6);
    restored.recompute();
    var bigger = restored.outputFeature().currentShape().volume();
    check(Math.abs(bigger - expectedVolume(20, 20, 10, 6)) < 1e-6, 'recomputed volume $bigger');
    restored.close();

    // A document naming a type nobody registered is rejected, not skipped.
    var unknown = StringTools.replace(text, '"type":"test-corner-drill"', '"type":"test-unknown"');
    throws(() -> DocumentCodec.decode(unknown), "unsupported feature type: test-unknown", "decoding an unknown type");

    Sys.println("FeatureRegistrySmoke: registered type saves, restores, recomputes and labels its row");
  }

  public static function main():Void run();
}
