package stockkit;

/** What one `Stock.cut` call did, move by move. */
class CutReport {
  public final moves:Array<MoveOutcome>;

  public function new(moves:Array<MoveOutcome>) {
    this.moves = moves;
  }

  /** Volume each move removed, in the order given. */
  public function removed():Array<Float>
    return [for (outcome in moves) outcome.removed];

  public function removedVolume():Float {
    var total = 0.0;
    for (outcome in moves) total += outcome.removed;
    return total;
  }

  /** Rapid moves that removed more than `tolerance` of material (cubic metres). */
  public function rapidContacts(tolerance:Float = 0.0):Array<MoveOutcome>
    return [for (outcome in moves) if (outcome.move.rapid && outcome.removed > tolerance) outcome];

  /** Moves whose shank or holder overlapped more than `tolerance` of stock (cubic metres). */
  public function collisions(tolerance:Float = 0.0):Array<MoveOutcome>
    return [for (outcome in moves)
      if (outcome.shankContact > tolerance || outcome.holderContact > tolerance) outcome];

  /** Totals per source operation, in order of first appearance. */
  public function byOperation():Array<OperationTotals> {
    var totals:Array<OperationTotals> = [];
    for (outcome in moves) {
      var entry:Null<OperationTotals> = null;
      for (candidate in totals) if (candidate.toolId == outcome.move.toolId &&
          candidate.operationId == outcome.move.operationId &&
          (candidate.operationId != null || candidate.opIndex == outcome.move.opIndex))
        entry = candidate;
      if (entry == null) {
        entry = new OperationTotals(outcome.move);
        totals.push(entry);
      }
      entry.add(outcome);
    }
    return totals;
  }
}
