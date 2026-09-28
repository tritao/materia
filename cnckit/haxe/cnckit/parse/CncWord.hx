package cnckit.parse;

import toolpathkit.path.Provenance;
/** Numeric G-code word, retaining its original spelling location. */
class CncWord {
  public final letter:String;
  public final value:Float;
  public final span:Provenance;
  public var column(get, never):Int;
  inline function get_column():Int return span.column;

  public function new(letter:String, value:Float, span:Provenance) {
    this.letter = letter;
    this.value = value;
    this.span = span;
  }
}
