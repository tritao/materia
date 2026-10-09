package cnckit.parse;

import toolpathkit.path.Provenance;

/** A comment kept as data (printer dialect): slicers use them as layer and feature markers. */
class CncComment {
  /** The text after `;` (or inside parentheses), trimmed. */
  public final text:String;
  public final span:Provenance;

  public function new(text:String, span:Provenance) {
    this.text = text;
    this.span = span;
  }
}
