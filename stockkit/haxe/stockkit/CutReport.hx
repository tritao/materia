package stockkit;

/** What one `Stock.cut` call did. */
class CutReport {
  /** Volume each move removed, in the order given, in cubic metres; each ray stands for spacing squared of area. */
  public final removed:Array<Float>;
  /** Sources (indices in the stock's history) of rapid moves that removed material. */
  public final rapidContacts:Array<Int>;

  public function new(removed:Array<Float>, rapidContacts:Array<Int>) {
    this.removed = removed;
    this.rapidContacts = rapidContacts;
  }

  public function removedVolume():Float {
    var total = 0.0;
    for (volume in removed) total += volume;
    return total;
  }
}
