package cadkit.parametric.features;

import cadkit.parametric.EvaluationContext;
import cadkit.parametric.EvaluationResult;
import cadkit.parametric.Feature;
import cadkit.parametric.FeatureId;
import cadkit.parametric.ParametricError;
import cadkit.threads.Details;
import cadkit.threads.SurfaceDetail;
import cadkit.threads.ThreadSpec;
import cadkit.threads.ThreadedFace.ThreadStart;
import cadkit.threads.Threads;

/** What a `FaceDetailFeature` puts on its face. */
enum FaceDetail {
  /** A cosmetic thread from `start`, over `length` (the whole face when null), `offset` in from that end. */
  Thread(spec:ThreadSpec, start:ThreadStart, length:Null<Float>, offset:Float);
  /** Knurling, a scale or a display (`Details`). */
  Surface(detail:SurfaceDetail);
}

/**
 * A cosmetic detail on one face that `source` made, named as `source` names it (`cyl.side`, `side(seg.2)`): the solid is unchanged,
 * the face carries the attribute and keeps it through later steps, and a viewer draws it. Threads on studs, knurling on a grip,
 * a scale on a skirt and a display on a glass are recorded this way, in the part's history.
 */
class FaceDetailFeature extends Feature {
  public final source:Feature;
  public final face:String;
  public final detail:FaceDetail;

  public function new(source:Feature, face:String, detail:FaceDetail) {
    super();
    if (face.length == 0) throw new ParametricError("a face detail names its face");
    this.source = source;
    this.face = face;
    this.detail = detail;
  }

  override public function serializationType():String return "face-detail";

  override public function dependencies():Array<FeatureId> return [source.id];

  override public function dependencyFeatures():Array<Feature> return [source];

  /** The face's full name in `source`'s result: a detail makes no faces, so the maker is the first source that is not a detail. */
  public function faceName():String {
    var maker = source;
    while (Std.isOfType(maker, FaceDetailFeature)) maker = (cast maker : FaceDetailFeature).source;
    return "f" + maker.id.toInt() + ":" + face;
  }

  override public function evaluate(context:EvaluationContext):EvaluationResult {
    var shape = context.shape(source);
    return EvaluationResult.fromShape(switch detail {
      case Thread(spec, start, length, offset): Threads.attach(shape, faceName(), spec, start, length, offset);
      case Surface(surface): Details.attach(shape, faceName(), surface);
    });
  }
}
