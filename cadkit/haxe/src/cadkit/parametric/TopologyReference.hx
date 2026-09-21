package cadkit.parametric;

import CadKit;
import cadkit.Edge;
import cadkit.Face;
import cadkit.Operation;
import cadkit.Shape;
import cadkit.Vertex;
import cadkit.parametric.Feature;
import cadkit.parametric.ParametricError;
import cadkit.parametric.ReferenceState;
import cadkit.parametric.TopologyFingerprint;
import cadkit.parametric.TopologyHistoryMap;

/** A document-owned topology selection that can be remapped after recompute. */
class TopologyReference {
	public final feature:Feature;
	public final kind:CadKit.ShapeKind;
	public var state(default, null):ReferenceState;

	private final remapFeature:Feature;
	private var current:Null<Shape>;
	private final fingerprint:TopologyFingerprint;
	private var fallbackAmbiguous:Bool;
	private var stateGenerationValue:Int;

	public function new(
		feature:Feature,
		topology:Null<Shape>,
		?savedKind:CadKit.ShapeKind,
		?savedFingerprint:TopologyFingerprint,
		?remapFeature:Feature) {
		this.feature = feature;
		this.remapFeature = remapFeature == null ? feature : remapFeature;
		if (topology == null) {
			if (savedKind == null || savedFingerprint == null)
				throw new ParametricError("unresolved topology references need fingerprint data");
			this.kind = savedKind;
			this.current = null;
			this.fingerprint = savedFingerprint;
		} else {
			this.kind = topology.kind();
			this.current = topology;
			this.fingerprint = TopologyFingerprint.capture(topology);
		}
		if (kind != CadKit.ShapeKind.Face &&
			kind != CadKit.ShapeKind.Edge &&
			kind != CadKit.ShapeKind.Vertex)
			throw new ParametricError("topology references require faces, edges, or vertices");
		this.state = current == null ? ReferenceState.Unresolved : ReferenceState.Resolved;
		this.fallbackAmbiguous = false;
		this.stateGenerationValue = 0;
		feature.registerTopologyReference(this);
	}

	public static function fromFingerprint(
		feature:Feature,
		kind:CadKit.ShapeKind,
		fingerprint:TopologyFingerprint,
		?remapFeature:Feature):TopologyReference {
		return new TopologyReference(feature, null, kind, fingerprint, remapFeature);
	}

	public static function fromShape(
		feature:Feature,
		topology:Shape,
		?remapFeature:Feature):TopologyReference {
		return new TopologyReference(feature, topology, null, null, remapFeature);
	}

	public static function fromFace(
		feature:Feature,
		face:Face,
		?remapFeature:Feature):TopologyReference {
		return new TopologyReference(feature, face.cloneShape(), null, null, remapFeature);
	}

	public static function fromEdge(
		feature:Feature,
		edge:Edge,
		?remapFeature:Feature):TopologyReference {
		return new TopologyReference(feature, edge.cloneShape(), null, null, remapFeature);
	}

	public static function fromVertex(
		feature:Feature,
		vertex:Vertex,
		?remapFeature:Feature):TopologyReference {
		return new TopologyReference(feature, vertex.cloneShape(), null, null, remapFeature);
	}

	public function isResolved():Bool {
		return (state == ReferenceState.Resolved || state == ReferenceState.Remapped) &&
			current != null;
	}

	public function stateGeneration():Int {
		return stateGenerationValue;
	}

	/** The reference owns the returned shape; callers must not close it directly. */
	public function currentShape():Null<Shape> {
		return current;
	}

	public function fingerprintData():TopologyFingerprint {
		return fingerprint;
	}

	/** Resolve against a staged or committed shape, recording failure state. */
	public function resolveFor(result:Shape, ?operation:Operation):Shape {
		if (state == ReferenceState.Closed)
			throw new ParametricError("closed topology references cannot be resolved");

		if (operation != null && current != null) {
			var historyResult = new TopologyHistoryMap(operation).remap(current, kind);
			switch historyResult.state {
				case ReferenceState.Remapped:
					if (historyResult.shape != null)
						return historyResult.shape;
				case ReferenceState.Deleted:
					markDeleted();
					throw new ParametricError(
						"topology reference is Deleted", ReferenceState.Deleted);
				case ReferenceState.Ambiguous:
					markAmbiguous();
					throw new ParametricError(
						"topology reference is Ambiguous", ReferenceState.Ambiguous);
				case ReferenceState.Resolved, ReferenceState.Unresolved, ReferenceState.Closed:
			}
		}

		var resolved = findFallback(result);
		if (resolved != null)
			return resolved;
		if (fallbackAmbiguous) {
			markAmbiguous();
			throw new ParametricError(
				"topology reference is Ambiguous", ReferenceState.Ambiguous);
		}
		if (current != null) {
			markDeleted();
			throw new ParametricError(
				"topology reference is Deleted", ReferenceState.Deleted);
		}
		if (state == ReferenceState.Deleted)
			throw new ParametricError("topology reference is Deleted", ReferenceState.Deleted);
		if (state == ReferenceState.Ambiguous)
			throw new ParametricError("topology reference is Ambiguous", ReferenceState.Ambiguous);
		markUnresolved();
		throw new ParametricError(
			"topology reference is Unresolved", ReferenceState.Unresolved);
	}

	public function remap():ReferenceState {
		if (state == ReferenceState.Closed)
			return state;

		var operation = remapFeature.provenance;
		if (operation != null && current != null) {
			var historyResult = new TopologyHistoryMap(operation).remap(current, kind);
			switch historyResult.state {
				case ReferenceState.Remapped:
					if (historyResult.shape != null) {
						replace(historyResult.shape, ReferenceState.Remapped);
						return state;
					}
				case ReferenceState.Deleted:
					markDeleted();
					return state;
				case ReferenceState.Ambiguous:
					markAmbiguous();
					return state;
				case ReferenceState.Resolved, ReferenceState.Unresolved, ReferenceState.Closed:
			}
		}

		var result = remapFeature.currentShape();
		if (result == null) {
			markUnresolved();
			return state;
		}

		var fallback = findFallback(result);
		if (fallback != null) {
			replace(fallback, ReferenceState.Remapped);
			return state;
		} else if (fallbackAmbiguous) {
			markAmbiguous();
			return state;
		} else if (current != null) {
			markDeleted();
			return state;
		} else {
			markUnresolved();
			return state;
		}
	}

	public function close():Void {
		if (state == ReferenceState.Closed)
			return;
		if (current != null)
			current.close();
		current = null;
		state = ReferenceState.Closed;
	}

	private function findFallback(result:Shape):Null<Shape> {
		fallbackAmbiguous = false;
		var count = result.subshapeCount(kind);
		var best:Null<Shape> = null;
		var bestScore = -1.0e30;
		var secondBestScore = -1.0e30;
		for (index in 0...count) {
			var candidate = result.subshape(kind, index);
			if (current != null && current.sameAs(candidate)) {
				if (best != null)
					best.close();
				return candidate;
			}
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
			return null;
		}
		if (secondBestScore > -1.0e29 && bestScore - secondBestScore <= 1.0e-6) {
			best.close();
			fallbackAmbiguous = true;
			return null;
		}
		return best;
	}

	private function replace(next:Shape, nextState:ReferenceState):Void {
		if (current != null)
			current.close();
		current = next;
		setState(nextState);
	}

	private function markUnresolved():Void {
		if (current != null)
			current.close();
		current = null;
		setState(ReferenceState.Unresolved);
	}

	private function markAmbiguous():Void {
		if (current != null)
			current.close();
		current = null;
		setState(ReferenceState.Ambiguous);
	}

	private function markDeleted():Void {
		if (current != null)
			current.close();
		current = null;
		setState(ReferenceState.Deleted);
	}

	private function setState(next:ReferenceState):Void {
		if (state != next)
			stateGenerationValue++;
		state = next;
	}
}
