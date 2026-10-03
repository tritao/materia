package motionkit.robot;

import motionkit.trajectory.PlanDiagnostic;

/** What the plan checks of a run found, added up: the findings and how near the drives came to their limits. */
class PlanCheckSummary {
  /** Findings kept, most recent last; later ones are only counted. */
  public static inline final MAX_KEPT = 1000;

  public var plans(default, null):Int = 0;
  /** Plans with at least one finding. */
  public var flagged(default, null):Int = 0;
  public final diagnostics:Array<PlanDiagnostic> = [];
  /** Hardware ceilings that explain the compiled planning speeds. */
  public final speedLimits:Array<String> = [];
  public var findings(default, null):Int = 0;
  public var worstTorqueRatio(default, null):Float = 0.0;
  public var worstMotor(default, null):String = "";
  public var worstDeviation(default, null):Float = 0.0;
  public var worstAxis(default, null):String = "";
  final byKind = new Map<String, Int>();

  public function new() {}

  public function add(result:PlanCheckResult):Void {
    plans++;
    for (limit in result.speedLimits) if (speedLimits.indexOf(limit) < 0) speedLimits.push(limit);
    if (result.diagnostics.length > 0) flagged++;
    for (diagnostic in result.diagnostics) {
      findings++;
      byKind.set(diagnostic.kind, count(diagnostic.kind) + 1);
      if (diagnostics.length < MAX_KEPT) diagnostics.push(diagnostic);
    }
    if (result.worstTorqueRatio > worstTorqueRatio) {
      worstTorqueRatio = result.worstTorqueRatio;
      worstMotor = result.worstMotor;
    }
    if (result.worstDeviation > worstDeviation) {
      worstDeviation = result.worstDeviation;
      worstAxis = result.worstAxis;
    }
  }

  /** Findings of one kind. */
  public function count(kind:PlanDiagnosticKind):Int {
    var found = byKind.get(kind);
    return found == null ? 0 : found;
  }

  public function reset():Void {
    plans = 0;
    flagged = 0;
    findings = 0;
    worstTorqueRatio = 0.0;
    worstMotor = "";
    worstDeviation = 0.0;
    worstAxis = "";
    while (diagnostics.length > 0) diagnostics.pop();
    while (speedLimits.length > 0) speedLimits.pop();
    byKind.clear();
  }
}
