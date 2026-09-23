package nativekit.scene;

/**
 * Reusable base presentation rules for a SceneView.
 *
 * Policies are composable: apply them in order and a later rule for the same
 * node replaces an earlier rule in the same layer. Selection and hover
 * are intentionally not part of this layer; ScenePresentation owns those
 * transient interaction layers.
 */
class SceneViewPolicy {
	final visibility:VisibilityFilter;
	final materialNodes:Array<NodeId> = [];
	final materialValues:Array<Material> = [];
	final isolatedSources:Array<haxe.Int64> = [];
	final sourceVisibilitySources:Array<haxe.Int64> = [];
	final sourceVisibilityValues:Array<Bool> = [];
	final sourceMaterialSources:Array<haxe.Int64> = [];
	final sourceMaterialValues:Array<Material> = [];

	public function new() {
		visibility = new VisibilityFilter();
	}

	public function setVisibility(node:NodeId, visible:Bool):SceneViewPolicy {
		visibility.set(node, visible);
		return this;
	}

	public function hide(node:NodeId):SceneViewPolicy
		return setVisibility(node, false);

	public function show(node:NodeId):SceneViewPolicy
		return setVisibility(node, true);

	/** Visibility follows the node hierarchy, so this affects descendants. */
	public function hideSubtree(node:NodeId):SceneViewPolicy
		return hide(node);

	/** Restores this subtree unless another ancestor policy keeps it hidden. */
	public function showSubtree(node:NodeId):SceneViewPolicy
		return show(node);

	public function setMaterial(node:NodeId, material:Material):SceneViewPolicy {
		var stable = node.stableValue();
		for (index in 0...materialNodes.length) {
			if (materialNodes[index].stableValue() == stable) {
				materialValues[index] = material;
				return this;
			}
		}
		materialNodes.push(node);
		materialValues.push(material);
		return this;
	}

	/** Adds or replaces a source-level visibility rule. */
	public function setSourceVisibility(source:haxe.Int64, visible:Bool):SceneViewPolicy {
		for (index in 0...sourceVisibilitySources.length) {
			if (sourceVisibilitySources[index] == source) {
				sourceVisibilityValues[index] = visible;
				return this;
			}
		}
		sourceVisibilitySources.push(source);
		sourceVisibilityValues.push(visible);
		return this;
	}

	/** Adds or replaces a source-level base material rule. */
	public function setSourceMaterial(source:haxe.Int64, material:Material):SceneViewPolicy {
		for (index in 0...sourceMaterialSources.length) {
			if (sourceMaterialSources[index] == source) {
				sourceMaterialValues[index] = material;
				return this;
			}
		}
		sourceMaterialSources.push(source);
		sourceMaterialValues.push(material);
		return this;
	}

	/** Replaces the source isolation set with one source. */
	public function isolateSource(source:haxe.Int64):SceneViewPolicy {
		isolatedSources.resize(0);
		isolatedSources.push(source);
		return this;
	}

	/** Replaces the source isolation set with the supplied sources. */
	public function isolateSources(sources:Array<haxe.Int64>):SceneViewPolicy {
		isolatedSources.resize(0);
		for (source in sources) {
			var found = false;
			for (existing in isolatedSources)
				if (existing == source) {
					found = true;
					break;
				}
			if (!found)
				isolatedSources.push(source);
		}
		return this;
	}

	public function clearIsolation():SceneViewPolicy {
		isolatedSources.resize(0);
		return this;
	}

	public function clear():SceneViewPolicy {
		visibility.clear();
		materialNodes.resize(0);
		materialValues.resize(0);
		isolatedSources.resize(0);
		sourceVisibilitySources.resize(0);
		sourceVisibilityValues.resize(0);
		sourceMaterialSources.resize(0);
		sourceMaterialValues.resize(0);
		return this;
	}

	public function visibilityCount():Int
		return visibility.count();

	public function materialCount():Int
		return materialNodes.length;

	public function isolationRuleCount():Int
		return isolatedSources.length;

	public function sourceVisibilityRuleCount():Int
		return sourceVisibilitySources.length;

	public function sourceMaterialRuleCount():Int
		return sourceMaterialSources.length;

	public function applySourceFilter(snapshot:SceneSnapshot,
			filter:SceneViewFilter):SceneViewPolicy
		return filter.apply(snapshot, this);

	/** Applies this policy without disturbing other view layers. */
	public function apply(view:SceneView):SceneView {
		view.applyVisibilityFilter(visibility);
		for (index in 0...materialNodes.length)
			view.setMaterial(materialNodes[index], materialValues[index]);
		for (index in 0...sourceVisibilitySources.length)
			view.setSourceVisibility(sourceVisibilitySources[index], sourceVisibilityValues[index]);
		for (index in 0...sourceMaterialSources.length)
			view.setSourceMaterial(sourceMaterialSources[index], sourceMaterialValues[index]);
		for (source in isolatedSources)
			view.setIsolatedSource(source, true);
		return view;
	}
}
