package stockkit;

import toolpathkit.tool.Tool;
import toolpathkit.path.Provenance;
import toolpathkit.path.MoveKind;

/** One tool motion through the stock, with where it came from. */
class CutMove {
  public final tool:Tool;
  public final motion:CutMotion;
  public final kind:MoveKind;
  /** Rapid moves should not touch stock; cutting them is a diagnostic. */
  public var rapid(get, never):Bool;
  /** Index of the source operation in the program's op list. */
  public final opIndex:Int;
  public final provenance:Provenance;
  public final operationId:Null<String>;
  public final featureRef:Null<String>;
  public final toolId:Int;

  public function new(tool:Tool, motion:CutMotion, kind:MoveKind,
      opIndex:Int, provenance:Provenance) {
    if (tool == null || motion == null || provenance == null)
      throw "cut move needs a tool, motion and provenance";
    this.tool = tool;
    this.motion = motion;
    this.kind = kind;
    this.opIndex = opIndex;
    this.provenance = provenance;
    this.operationId = provenance.operationId;
    this.featureRef = provenance.featureRef;
    this.toolId = tool.number;
  }

  function get_rapid():Bool
    return kind == Rapid || kind == Link || kind == Retract;
}
