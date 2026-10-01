package cadkit.parametric;

import CadKit;
import cadkit.Edge;
import cadkit.Face;
import cadkit.ElementNames;
import cadkit.Shape;
import cadkit.Vertex;
import cadkit.parametric.Feature;
import cadkit.parametric.ParametricError;
import cadkit.parametric.ReferenceState;
import cadkit.parametric.TopologyFingerprint;
import cadkit.parametric.TopologyReferenceUpdate;
import cadkit.parametric.TopologyResolution.ResolutionMethod;

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
	private var resolvedByValue:ResolutionMethod;
	private var candidatesValue:Array<Int>;

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
		this.resolvedByValue = current == null ? ResolutionMethod.NotFound : ResolutionMethod.Identity;
		this.candidatesValue = [];
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

	/**
		Resolve against a staged or committed shape, recording failure state. Operation history plays no part: it only
		relates one evaluation's inputs to its outputs, so across recomputes the element is found by its name, then its
		geometry (plans/TOPOLOGICAL_NAMING.md, TN-D2).
	*/
	public function resolveFor(result:Shape):Shape {
		if (state == ReferenceState.Closed)
			throw new ParametricError("closed topology references cannot be resolved");
		var resolution = TopologyResolver.resolve(result, fingerprint, kind, current);
		fallbackAmbiguous = resolution.state == ReferenceState.Ambiguous;
		resolvedByValue = resolution.method;
		candidatesValue = resolution.candidates.copy();
		if (resolution.state == ReferenceState.Resolved)
			return result.subshape(kind, resolution.index);
		if (fallbackSelection != null) {
			var selected = fallbackSelection.resolve(result)[0];
			resolvedByValue = ResolutionMethod.Selection;
			return selected;
		}
		if (fallbackAmbiguous) {
			markAmbiguous();
			throw new ParametricError("topology reference is Ambiguous", ReferenceState.Ambiguous);
		}
		if (creatorRemoved()) {
			markDeleted();
			throw new ParametricError("topology reference is Deleted: the feature that made it is gone", ReferenceState.Deleted);
		}
		markUnresolved();
		throw new ParametricError("topology reference could not be matched with sufficient confidence", ReferenceState.Unresolved);
	}

	public function prepareRemap(result:Null<Shape>):TopologyReferenceUpdate {
		if (state == ReferenceState.Closed)
			return new TopologyReferenceUpdate(this, null, fingerprint, state, fallbackAmbiguous, true);

		var next:Null<Shape> = null;
		var nextState:ReferenceState = ReferenceState.Unresolved;
		var nextFingerprint = fingerprint;
		var nextAmbiguous = false;
		var method:ResolutionMethod = ResolutionMethod.NotFound;
		var candidates:Array<Int> = [];
		try {
			if (result != null) {
				var resolution = TopologyResolver.resolve(result, fingerprint, kind, current);
				if (resolution.state == ReferenceState.Resolved) {
					next = result.subshape(kind, resolution.index);
					method = resolution.method;
				} else if (resolution.state == ReferenceState.Ambiguous) {
					nextAmbiguous = true;
					candidates = resolution.candidates;
				}

				if (next == null && fallbackSelection != null) {
					try {
						var selected = fallbackSelection.resolve(result);
						if (selected.length > 0) {
							next = selected[0];
							method = ResolutionMethod.Selection;
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
				candidates = [];
			} else {
				nextState = nextAmbiguous ? ReferenceState.Ambiguous
					: creatorRemoved() ? ReferenceState.Deleted : ReferenceState.Unresolved;
			}
			return new TopologyReferenceUpdate(this, next, nextFingerprint, nextState, nextAmbiguous, false, method, candidates);
		} catch (error:Dynamic) {
			if (next != null)
				next.close();
			throw error;
		}
	}

	/** How the last resolution found the element (`NotFound` when it did not). */
	public function resolvedBy():ResolutionMethod
		return resolvedByValue;

	/**
		When ambiguous: the indices, among the producer's subshapes of this kind, of the elements it could not choose
		between (for a repair UI to offer). Empty otherwise.
	*/
	public function candidates():Array<Int>
		return candidatesValue.copy();

	/**
		Whether the feature whose tag starts the element's name no longer exists or is suppressed: the element was not
		lost by an edit, its maker was removed (`Deleted` rather than `Unresolved`).
	*/
	function creatorRemoved():Bool {
		var name = fingerprint.name;
		var owner = feature.document;
		if (name == null || owner == null)
			return false;
		var tag = ElementNames.creatorTag(name);
		if (tag.length < 2 || tag.charAt(0) != "f")
			return false;
		var id = Std.parseInt(tag.substr(1));
		if (id == null || Std.string(id) != tag.substr(1))
			return false;
		var creator = owner.featureById(id);
		return creator == null || !creator.active;
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
		resolvedByValue = update.method;
		candidatesValue = update.candidates.copy();
		if (state != update.state)
			stateGenerationValue++;
		state = update.state;
		update.markPublished();
	}

	public function remap():ReferenceState {
		var update = prepareRemap(remapFeature.currentShape());
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
