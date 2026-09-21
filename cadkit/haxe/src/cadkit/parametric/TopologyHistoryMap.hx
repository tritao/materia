package cadkit.parametric;

import CadKit;
import cadkit.Operation;
import cadkit.Shape;

/**
 * Centralized generated/modified/deleted topology mapping for one operation.
 * Returned mapped shapes are caller-owned through TopologyRemapResult.
 */
class TopologyHistoryMap {
	private final operation:Operation;

	public function new(operation:Operation) {
		this.operation = operation;
	}

	public function remap(current:Null<Shape>, kind:CadKit.ShapeKind):TopologyRemapResult {
		if (current == null)
			return new TopologyRemapResult(ReferenceState.Unresolved, null);

		var targets:Array<Shape> = [];
		appendTargets(current, kind, CadKit.HistoryRelation.Generated, targets);
		appendTargets(current, kind, CadKit.HistoryRelation.Modified, targets);

		if (targets.length == 1)
			return new TopologyRemapResult(ReferenceState.Remapped, targets[0]);
		if (targets.length > 1) {
			for (target in targets)
				target.close();
			return new TopologyRemapResult(ReferenceState.Ambiguous, null);
		}

		if (isDeleted(current, kind))
			return new TopologyRemapResult(ReferenceState.Deleted, null);
		return new TopologyRemapResult(ReferenceState.Unresolved, null);
	}

	private function appendTargets(
		current:Shape,
		kind:CadKit.ShapeKind,
		relation:CadKit.HistoryRelation,
		targets:Array<Shape>):Void {
		var count = operation.historyCount(relation);
		for (index in 0...count) {
			var source = operation.historySourceAt(relation, index);
			if (source.kind() != kind || !current.sameAs(source)) {
				source.close();
				continue;
			}
			source.close();

			var target = operation.historyTargetAt(relation, index);
			if (target.kind() != kind) {
				target.close();
				continue;
			}

			var duplicate = false;
			for (existing in targets) {
				if (existing.sameAs(target)) {
					duplicate = true;
					break;
				}
			}
			if (duplicate)
				target.close();
			else
				targets.push(target);
		}
	}

	private function isDeleted(current:Shape, kind:CadKit.ShapeKind):Bool {
		var count = operation.historyCount(CadKit.HistoryRelation.Deleted);
		for (index in 0...count) {
			var source = operation.historySourceAt(CadKit.HistoryRelation.Deleted, index);
			var matches = source.kind() == kind && current.sameAs(source);
			source.close();
			if (matches)
				return true;
		}
		return false;
	}
}
