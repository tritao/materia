package cnckit.parse;

/** Numeric G-code word, retaining its original spelling location. */
class CncWord {
  public final letter:String;
  public final value:Float;
  public final span:CncSpan;
  public var column(get, never):Int;
  inline function get_column():Int return span.column;

  public function new(letter:String, value:Float, span:CncSpan) {
    this.letter = letter;
    this.value = value;
    this.span = span;
  }
}
