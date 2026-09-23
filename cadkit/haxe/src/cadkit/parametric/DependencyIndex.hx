package cadkit.parametric;

import cadkit.parametric.InstanceElement;
import cadkit.parametric.LevelElement;

/** Reverse dependency index shared by document invalidation paths. */
class DependencyIndex {
	private final reverse:Map<String, Array<DependencyNode>>;

	public function new(document:Document) {
		reverse = new Map();

		for (index in 0...document.featureCount()) {
			var feature = document.featureAt(index);
			var target = DependencyNode.FeatureNode(feature.id.toInt());
			for (dependency in feature.dependencies())
				add(DependencyNode.FeatureNode(dependency.toInt()), target);
			for (dependency in feature.datumDependencies())
				add(DependencyNode.ElementNode(dependency), target);
			for (dependency in feature.elementDependencies())
				add(DependencyNode.ElementNode(dependency), target);
		}

		for (element in document.allElements()) {
			if (element.output != null)
				add(DependencyNode.FeatureNode(element.output.id.toInt()),
					DependencyNode.ElementNode(element.id.value));

			if (element.placementParent != null)
				add(DependencyNode.ElementNode(element.placementParent.elementId.value),
					DependencyNode.ElementNode(element.id.value));

			if (Std.isOfType(element, LevelElement)) {
				var level:LevelElement = cast element;
				if (level.relativeTo != null && level.relativeTo.documentId.value == document.id.value)
					add(DependencyNode.ElementNode(level.relativeTo.elementId.value),
						DependencyNode.ElementNode(level.id.value));
			}

			if (Std.isOfType(element, InstanceElement)) {
				var instance:InstanceElement = cast element;
				add(DependencyNode.DefinitionNode(instance.definitionId.value),
					DependencyNode.ElementNode(instance.id.value));
			}
		}

		for (named in document.namedParameters()) {
			for (dependency in named.dependencies())
				add(DependencyNode.ParameterNode(dependency), DependencyNode.ParameterNode(named.name));
			for (binding in named.bindings()) {
				var owner = DependencyNode.FeatureNode(binding.ownerFeature().id.toInt());
				var parameter = DependencyNode.ParameterNode(named.name);
				add(owner, parameter);
				add(parameter, owner);
			}
		}
	}

	public function dependents(source:DependencyNode):Array<DependencyNode> {
		var result = reverse.get(key(source));
		return result == null ? [] : result.copy();
	}

	public static function key(node:DependencyNode):String {
		return switch (node) {
			case FeatureNode(id): "feature:" + id;
			case ElementNode(id): "element:" + id;
			case DefinitionNode(id): "definition:" + id;
			case ParameterNode(name): "parameter:" + name;
		};
	}

	function add(source:DependencyNode, target:DependencyNode):Void {
		var sourceKey = key(source);
		var targets = reverse.get(sourceKey);
		if (targets == null) {
			targets = [];
			reverse.set(sourceKey, targets);
		}
		var targetKey = key(target);
		for (existing in targets)
			if (key(existing) == targetKey)
				return;
		targets.push(target);
	}
}
