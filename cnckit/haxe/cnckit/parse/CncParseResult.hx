package cnckit.parse;

import cnckit.CncDiagnostic;

class CncParseResult {
  public final blocks:Array<CncBlock>;
  public final diagnostics:Array<CncDiagnostic>;

  public function new(blocks:Array<CncBlock>, diagnostics:Array<CncDiagnostic>) {
    this.blocks = blocks;
    this.diagnostics = diagnostics;
  }
}
