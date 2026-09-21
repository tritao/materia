package cadkit.parametric;

import CadKit;
import cadkit.Edge;
import cadkit.Face;
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

	private var current:Null<Shape>;
	private final fingerprint:TopologyFingerprint;
	private var fallbackAmbiguous:Bool;

	public function new(
		feature:Feature,
		topology:Null<Shape>,
		?savedKind:CadKit.ShapeKind,
		?savedFingerprint:TopologyFingerprint) {
		this.feature = feature;
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
		feature.registerTopologyReference(this);
	}

	public static function fromFingerprint(
		feature:Feature,
		kind:CadKit.ShapeKind,
		fingerprint:TopologyFingerprint):TopologyReference {
		return new TopologyReference(feature, null, kind, fingerprint);
	}

	public static function fromFace(feature:Feature, face:Face):TopologyReference {
		return new TopologyReference(feature, face.cloneShape());
	}

	public static function fromEdge(feature:Feature, edge:Edge):TopologyReference {
		return new TopologyReference(feature, edge.cloneShape());
	}

	public static function fromVertex(feature:Feature, vertex:Vertex):TopologyReference {
		return new TopologyReference(feature, vertex.cloneShape());
	}

	public function isResolved():Bool {
		return (state == ReferenceState.Resolved || state == ReferenceState.Remapped) &&
			current != null;
	}

	/** The reference owns the returned shape; callers must not close it directly. */
	public function currentShape():Null<Shape> {
		return current;
	}

	public function fingerprintData():TopologyFingerprint {
		return fingerprint;
	}

	public function remap():ReferenceState {
		if (state == ReferenceState.Closed)
			return state;

		var operation = feature.provenance;
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

		var result = feature.currentShape();
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
		state = nextState;
	}

	private function markUnresolved():Void {
		if (current != null)
			current.close();
		current = null;
		state = ReferenceState.Unresolved;
	}

	private function markAmbiguous():Void {
		if (current != null)
			current.close();
		current = null;
		state = ReferenceState.Ambiguous;
	}

	private function markDeleted():Void {
		if (current != null)
			current.close();
		current = null;
		state = ReferenceState.Deleted;
	}
}
