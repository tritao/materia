package cnckit.parse;

import toolpathkit.path.Provenance;
/** One source line after comments have been removed. */
class CncBlock {
  public final words:Array<CncWord>;
  public final span:Provenance;
  /** Comments of the line; kept only by the printer dialect. */
  public final comments:Array<CncComment>;
  /** A Klipper-style extended command such as `EXCLUDE_OBJECT_START`, upper-cased; null for a lettered block. */
  public final command:Null<String>;
  /** The `KEY=value` arguments of an extended command, keys upper-cased, values unquoted. */
  public final arguments:Map<String, String>;

  public function new(words:Array<CncWord>, span:Provenance, ?comments:Array<CncComment>,
      ?command:String, ?arguments:Map<String, String>) {
    this.words = words.copy();
    this.span = span;
    this.comments = comments == null ? [] : comments.copy();
    this.command = command;
    this.arguments = arguments == null ? new Map() : arguments;
  }
}
