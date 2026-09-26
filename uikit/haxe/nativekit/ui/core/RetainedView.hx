package nativekit.ui.core;

import nativekit.ui.style.StyleState;

/** Reuses a stable render subtree while an application-owned revision key is unchanged. */
class RetainedView implements View {
	final builder:BuildContext->View;
	final key:String;
	final revision:Null<Void->String>;

	public function new(key:String, builder:BuildContext->View,
		?revision:Void->String) {
		if (key == null || key.length == 0 || builder == null)
			throw "Retained views require a stable key and builder";
		this.key = key;
		this.builder = builder;
		this.revision = revision;
	}

	public function build(context:BuildContext):RenderNode {
		var cache:RetainedViewCache = context.state(
			context.id("retained-view-cache:" + key), new RetainedViewCache()).value;
		var revisionBuilder:Null<Void->String> = revision;
		var cacheKey = revisionBuilder == null ? "" : revisionBuilder();
		cacheKey += "|style=" + context.styleRevision +
			"|viewport=" + context.viewportWidth + "x" + context.viewportHeight;
		var cached = cache.entry;
		if (cached != null && cached.key == cacheKey && cached.statesMatch(context)) {
			context.retainStateIds(cached.stateIds);
			context.claimRetainedTree(cached.root);
			cached.root.detach();
			return cached.root;
		}
		if (cached != null)
			cached.root.detach();
		var marker = context.stateUsageMarker();
		var view = builder(context);
		if (view == null)
			throw 'Retained view "$key" returned no view';
		var root = view.build(context);
		cache.replace(cacheKey, root, context.stateIdsUsedSince(marker));
		return root;
	}
}

private class RetainedViewCache {
	public var entry(default, null):Null<RetainedViewEntry>;

	public function new()
		entry = null;

	public function replace(key:String, root:RenderNode, stateIds:Array<Int>):Void {
		if (entry != null && entry.root != root)
			entry.root.detach();
		entry = new RetainedViewEntry(key, root, stateIds);
	}
}

private class RetainedViewEntry {
	public final key:String;
	public final root:RenderNode;
	public final stateIds:Array<Int>;

	public function new(key:String, root:RenderNode, stateIds:Array<Int>) {
		this.key = key;
		this.root = root;
		this.stateIds = stateIds == null ? [] : stateIds.copy();
	}

	public function statesMatch(context:BuildContext):Bool {
		var mask = StyleState.Hovered | StyleState.Pressed | StyleState.Focused;
		var result = true;
		root.walk(function(node) {
			if ((node.states & mask) != (context.interactionStates.get(node.id) & mask))
				result = false;
		});
		return result;
	}
}
