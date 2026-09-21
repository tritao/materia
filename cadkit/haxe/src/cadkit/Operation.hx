package cadkit;

import CadKit;

/** Owning result and topology history for one modeling operation. */
class Operation {
	private var native:CadKit.OwnedOperationHandle;

	public function new(native:CadKit.OwnedOperationHandle) {
		this.native = native;
	}

	public function resultShape():Shape {
		return Shape.fromOwnedHandle(CadKit.operationResultShapeChecked(native.borrow()));
	}

	public function history():OperationHistory {
		return new OperationHistory(this);
	}

	public function historyCount(relation:CadKit.HistoryRelation):Int {
		return CadKit.operationHistoryCountChecked(native.borrow(), relation);
	}

	public function historySourceAt(
		relation:CadKit.HistoryRelation,
		index:Int):Shape {
		return Shape.fromOwnedHandle(
			CadKit.operationHistorySourceAtChecked(native.borrow(), relation, index));
	}

	public function historyTargetAt(
		relation:CadKit.HistoryRelation,
		index:Int):Shape {
		return Shape.fromOwnedHandle(
			CadKit.operationHistoryTargetAtChecked(native.borrow(), relation, index));
	}

	public function close():Bool {
		return native.close();
	}

	public function isClosed():Bool {
		return native.isClosed();
	}
}
