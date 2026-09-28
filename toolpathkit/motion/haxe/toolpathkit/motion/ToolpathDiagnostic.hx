package toolpathkit.motion;

import toolpathkit.path.Provenance;

/** A lowering warning tied to its authored operation. */
class ToolpathDiagnostic {
  public final code:String;
  public final provenance:Provenance;
  public final message:String;

  public function new(code:String, provenance:Provenance, message:String) {
    this.code = code; this.provenance = provenance; this.message = message;
  }
}
