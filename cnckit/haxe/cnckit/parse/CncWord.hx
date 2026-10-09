package cnckit.parse;

import toolpathkit.path.Provenance;
/** Numeric G-code word, retaining its original spelling location. */
class CncWord {
  public final letter:String;
  public final value:Float;
  public final span:Provenance;
  /** The quoted text of a string word such as Marlin's `A"name"`; null for a numeric word. */
  public final text:Null<String>;
  public var column(get, never):Int;
  inline function get_column():Int return span.column;

  public function new(letter:String, value:Float, span:Provenance, ?text:String) {
    this.letter = letter;
    this.value = value;
    this.span = span;
    this.text = text;
  }
}
