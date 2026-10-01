package cadkit.parametric;

import cadkit.Shape;
import cadkit.parametric.TopologyResolution.ResolutionMethod;

/** Owned, prepared replacement for one topology reference during recompute. */
class TopologyReferenceUpdate {
	public final reference:TopologyReference;
	public final current:Null<Shape>;
	public final fingerprint:TopologyFingerprint;
	public final state:ReferenceState;
	public final fallbackAmbiguous:Bool;
	public final skipped:Bool;
	/** How the element was found (`NotFound` when it was not). */
	public final method:ResolutionMethod;
	/** When ambiguous: the candidate indices in the producer's shape. */
	public final candidates:Array<Int>;
	public var published(default, null):Bool;

	public function new(reference:TopologyReference, current:Null<Shape>, fingerprint:TopologyFingerprint,
		state:ReferenceState, fallbackAmbiguous:Bool, skipped:Bool = false, ?method:ResolutionMethod, ?candidates:Array<Int>) {
		this.reference = reference;
		this.current = current;
		this.fingerprint = fingerprint;
		this.state = state;
		this.fallbackAmbiguous = fallbackAmbiguous;
		this.skipped = skipped;
		this.method = method == null ? ResolutionMethod.NotFound : method;
		this.candidates = candidates == null ? [] : candidates;
		published = false;
	}

	/** Release a prepared shape if publication did not consume it. */
	public function dispose():Void {
		if (published)
			return;
		if (current != null)
			current.close();
		published = true;
	}

	public function markPublished():Void
		published = true;
}
