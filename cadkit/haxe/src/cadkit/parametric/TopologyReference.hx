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
import cadkit.parametric.TopologyReferenceUpdate;

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

	/** Feature whose result is used when this reference is refreshed after recompute. */
	public function remapTargetFeature():Feature
		return remapFeature;

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

	public function prepareRemap(result:Null<Shape>, operation:Null<Operation>):TopologyReferenceUpdate {
		if (state == ReferenceState.Closed)
			return new TopologyReferenceUpdate(this, null, fingerprint, state, fallbackAmbiguous, true);

		var next:Null<Shape> = null;
		var nextState:ReferenceState = ReferenceState.Unresolved;
		var nextFingerprint = fingerprint;
		var nextAmbiguous = false;
		try {
			if (operation != null && current != null) {
				var historyResult = new TopologyHistoryMap(operation).remap(current, kind);
				switch historyResult.state {
					case ReferenceState.Remapped:
						if (historyResult.shape != null) {
							next = historyResult.shape;
							nextState = ReferenceState.Remapped;
						}
					case ReferenceState.Deleted:
						nextState = ReferenceState.Deleted;
					case ReferenceState.Ambiguous:
						nextState = ReferenceState.Ambiguous;
					case ReferenceState.Resolved, ReferenceState.Unresolved, ReferenceState.Closed:
				}
			}

			if (next == null && nextState != ReferenceState.Deleted && nextState != ReferenceState.Ambiguous && result != null) {
				var resolution = TopologyResolver.resolve(result, fingerprint, kind, current);
				if (resolution.state == ReferenceState.Resolved)
					next = result.subshape(kind, resolution.index);
				else if (resolution.state == ReferenceState.Ambiguous)
					nextAmbiguous = true;

				if (next == null && fallbackSelection != null) {
					try {
						var selected = fallbackSelection.resolve(result);
						if (selected.length > 0) {
							next = selected[0];
							for (index in 1...selected.length)
								selected[index].close();
						}
					} catch (error:Dynamic) {
						if (Std.isOfType(error, ParametricError)) {
							var parametric:ParametricError = cast error;
							nextAmbiguous = parametric.referenceState == ReferenceState.Ambiguous;
						} else {
							throw error;
						}
					}
				}
			}

			if (next != null) {
				nextFingerprint = TopologyFingerprint.capture(next);
				nextState = ReferenceState.Remapped;
			} else if (nextState != ReferenceState.Deleted && nextState != ReferenceState.Ambiguous) {
				nextState = nextAmbiguous ? ReferenceState.Ambiguous : ReferenceState.Unresolved;
			}
			return new TopologyReferenceUpdate(this, next, nextFingerprint, nextState, nextAmbiguous);
		} catch (error:Dynamic) {
			if (next != null)
				next.close();
			throw error;
		}
	}

	/** Publish a prepared remap without native calls; the document retires the old shape afterward. */
	public function validateRemapUpdate(update:TopologyReferenceUpdate):Void {
		if (update.reference != this || update.published)
			throw new ParametricError("topology remap update does not belong to this reference");
	}

	public function publishRemap(update:TopologyReferenceUpdate):Void {
		if (update.skipped) {
			update.markPublished();
			return;
		}
		current = update.current;
		fingerprint = update.fingerprint;
		fallbackAmbiguous = update.fallbackAmbiguous;
		if (state != update.state)
			stateGenerationValue++;
		state = update.state;
		update.markPublished();
	}

	public function remap():ReferenceState {
		var update = prepareRemap(remapFeature.currentShape(), remapFeature.provenance);
		var previous = current;
		publishRemap(update);
		if (previous != null)
			previous.close();
		return state;
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
