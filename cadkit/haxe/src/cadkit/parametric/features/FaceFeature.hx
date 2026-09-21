package cadkit.parametric.features;

import CadKit;
import cadkit.Shape;
import cadkit.parametric.EvaluationContext;
import cadkit.parametric.EvaluationResult;
import cadkit.parametric.Feature;
import cadkit.parametric.FeatureId;
import cadkit.parametric.ParametricError;
import cadkit.parametric.TopologyFingerprint;

/** Extracts an owning face profile, persisting a fingerprint after first evaluation. */
class FaceFeature extends Feature {
	public final source:Feature;
	public final index:Int;
	private var fingerprint:Null<TopologyFingerprint>;

	public function new(
		source:Feature,
		index:Int,
		?savedFingerprint:TopologyFingerprint) {
		super();
		if (index < 0)
			throw new ParametricError("face index must not be negative");
		this.source = source;
		this.index = index;
		fingerprint = savedFingerprint;
	}

	override public function serializationType():String {
		return "face";
	}

	override public function dependencies():Array<FeatureId> {
		return [source.id];
	}

	override public function dependencyFeatures():Array<Feature> {
		return [source];
	}

	override public function evaluate(context:EvaluationContext):EvaluationResult {
		var result = selectFace(context.shape(source));
		return EvaluationResult.fromShape(result);
	}

	/** The persisted profile key; null until an index-only feature is evaluated. */
	public function fingerprintData():Null<TopologyFingerprint> {
		return fingerprint;
	}

	private function selectFace(sourceShape:Shape):Shape {
		if (fingerprint == null) {
			var face = sourceShape.faces().at(index);
			var result = face.cloneShape();
			face.close();
			fingerprint = TopologyFingerprint.capture(result);
			return result;
		}

		var best:Null<Shape> = null;
		var bestScore = -1.0e30;
		var secondBestScore = -1.0e30;
		var count = sourceShape.subshapeCount(CadKit.ShapeKind.Face);
		for (faceIndex in 0...count) {
			var candidate = sourceShape.subshape(CadKit.ShapeKind.Face, faceIndex);
			var score = fingerprint.score(candidate);
			if (score > bestScore) {
				secondBestScore = bestScore;
				if (best != null)
					best.close();
				best = candidate;
				bestScore = score;
			} else {
				if (score > secondBestScore)
					secondBestScore = score;
				candidate.close();
			}
		}

		if (best == null || bestScore <= -1.0e29) {
			if (best != null)
				best.close();
			throw new ParametricError("face profile could not be remapped");
		}
		if (secondBestScore > -1.0e29 &&
			bestScore - secondBestScore <= 1.0e-6) {
			best.close();
			throw new ParametricError("face profile remap is ambiguous");
		}
		return best;
	}
}
