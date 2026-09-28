package cnckit;

import toolpathkit.path.Provenance;

enum CncSeverity {
  Error;
  Warning;
}

/** Stable code and source location for parser, interpreter, and lowering errors. */
class CncDiagnostic {
  public final severity:CncSeverity;
  public final code:String;
  public final span:Provenance;
  public final message:String;

  public function new(severity:CncSeverity, code:String, span:Provenance,
      message:String) {
    this.severity = severity;
    this.code = code;
    this.span = span;
    this.message = message;
  }

  public function toString():String
    return 'G-code line ${span.line} column ${span.column}: $message';
}
