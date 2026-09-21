package cadkit;

import CadKit;

/** Indexed view over an operation's generated, modified, or deleted entries. */
class OperationHistory {
	private final operation:Operation;

	public function new(operation:Operation) {
		this.operation = operation;
	}

	public function count(relation:CadKit.HistoryRelation):Int {
		return operation.historyCount(relation);
	}

	public function sourceAt(
		relation:CadKit.HistoryRelation,
		index:Int):Shape {
		return operation.historySourceAt(relation, index);
	}

	/** Targets exist for generated and modified entries, not deleted entries. */
	public function targetAt(
		relation:CadKit.HistoryRelation,
		index:Int):Shape {
		return operation.historyTargetAt(relation, index);
	}
}
