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
	/** Explicit geometric intent used only when identity/history cannot resolve the topology. */
	private final fallbackSelection:Null<SelectionRecipe>;
	private var current:Null<Shape>;
	private var fingerprint:TopologyFingerprint;
	private var fallbackAmbiguous:Bool;
	private var stateGenerationValue:Int;

	public function new(
		feature:Feature,
		topology:Null<Shape>,
		?savedKind:CadKit.ShapeKind,
		?savedFingerprint:TopologyFingerprint,
		?remapFeature:Feature,
		?fallbackSelection:SelectionRecipe) {
		this.feature = feature;
		this.remapFeature = remapFeature == null ? feature : remapFeature;
		this.fallbackSelection = fallbackSelection;
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
		?remapFeature:Feature,
		?fallbackSelection:SelectionRecipe):TopologyReference {
		return new TopologyReference(feature, null, kind, fingerprint, remapFeature, fallbackSelection);
	}

	public static function fromShape(
		feature:Feature,
		topology:Shape,
		?remapFeature:Feature,
		?fallbackSelection:SelectionRecipe):TopologyReference {
		return new TopologyReference(feature, topology, null, null, remapFeature, fallbackSelection);
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

	/** Rebind this document-owned reference to an explicitly selected topology. */
	public function rebind(topology:Shape):Void {
		if (topology == null || topology.kind() != kind)
			throw new ParametricError("replacement topology has the wrong kind");
		restore(topology, TopologyFingerprint.capture(topology), ReferenceState.Resolved);
	}

	/** Restore a saved reference state while applying an authored undo/redo change. */
	public function restore(topology:Null<Shape>, savedFingerprint:TopologyFingerprint,
		nextState:ReferenceState):Void {
		if (state == ReferenceState.Closed)
			throw new ParametricError("closed topology references cannot be restored");
		if (savedFingerprint == null || savedFingerprint.kind != kind)
			throw new ParametricError("saved topology fingerprint has the wrong kind");
		var needsTopology = nextState == ReferenceState.Resolved || nextState == ReferenceState.Remapped;
		if (needsTopology != (topology != null))
			throw new ParametricError("resolved topology references require a replacement shape");
		if (topology != null && topology.kind() != kind)
			throw new ParametricError("replacement topology has the wrong kind");
		if (nextState == ReferenceState.Closed)
			throw new ParametricError("closed topology references cannot be restored");

		var next = topology == null ? null : topology.cloneShape();
		if (current != null)
			current.close();
		current = next;
		fingerprint = savedFingerprint;
		fallbackAmbiguous = false;
		stateGenerationValue++;
		state = nextState;
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
		if (fallbackSelection != null)
			return fallbackSelection.resolve(result)[0];
		if (fallbackAmbiguous) {
			markAmbiguous();
			throw new ParametricError(
				"topology reference is Ambiguous", ReferenceState.Ambiguous);
		}
		if (current != null) {
			markUnresolved();
			throw new ParametricError(
				"topology reference could not be matched with sufficient confidence", ReferenceState.Unresolved);
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
		if (fallback == null && fallbackSelection != null) {
			try {
				fallback = fallbackSelection.resolve(result)[0];
			} catch (error:Dynamic) {
				if (Std.isOfType(error, ParametricError)) {
					var parametric:ParametricError = cast error;
					fallbackAmbiguous = parametric.referenceState == ReferenceState.Ambiguous;
				} else {
					throw error;
				}
			}
		}
		if (fallback != null) {
			replace(fallback, ReferenceState.Remapped);
			return state;
		} else if (fallbackAmbiguous) {
			markAmbiguous();
			return state;
		} else if (current != null) {
			markUnresolved();
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
		var resolution = TopologyResolver.resolve(result, fingerprint, kind, current);
		if (resolution.state == ReferenceState.Ambiguous) {
			fallbackAmbiguous = true;
			return null;
		}
		if (resolution.state != ReferenceState.Resolved)
			return null;
		return result.subshape(kind, resolution.index);
	}

	private function replace(next:Shape, nextState:ReferenceState):Void {
		var nextFingerprint = TopologyFingerprint.capture(next);
		if (current != null)
			current.close();
		current = next;
		fingerprint = nextFingerprint;
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
