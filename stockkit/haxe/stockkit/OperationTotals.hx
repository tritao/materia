package stockkit;

/** Removal and contact summed over the moves of one source operation. */
class OperationTotals {
  public final opIndex:Int;
  public final operationId:Null<String>;
  public final toolId:Int;
  public var moves(default, null) = 0;
  public var removed(default, null) = 0.0;
  public var shankContact(default, null) = 0.0;
  public var holderContact(default, null) = 0.0;

  public function new(move:CutMove) {
    this.opIndex = move.opIndex;
    this.operationId = move.operationId;
    this.toolId = move.toolId;
  }

  public function add(outcome:MoveOutcome):Void {
    moves++;
    removed += outcome.removed;
    shankContact += outcome.shankContact;
    holderContact += outcome.holderContact;
  }
}
