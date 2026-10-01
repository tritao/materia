package cadkit.sketch;

import cadkit.solve.ConstraintDiagnosis;

class SolveDiagnostic {
	public final status:String;
	public final converged:Bool;
	public final residual:Float;
	public final degreesOfFreedom:Int;
	public final iterations:Int;
	public final constraintIds:Array<String>;
	public final message:String;
	/** The full diagnosis (subsystems, dependency groups, suggestions); null when the sketch was invalid. */
	public final report:Null<DiagnosisReport>;
	/** The constraints are independent in general, but the solved pose loses rank (the report is the general one). */
	public final degenerate:Bool;
	/**
		False when a solve without diagnosis (a drag) reported some parts'
		previous diagnosis instead of checking them again; the next normal
		solve re-checks them.
	*/
	public final diagnosed:Bool;

	public function new(status:String, converged:Bool, residual:Float, degreesOfFreedom:Int, iterations:Int,
		constraintIds:Array<String>, message:String, ?report:DiagnosisReport, degenerate:Bool = false, diagnosed:Bool = true) {
		this.status = status; this.converged = converged; this.residual = residual;
		this.degreesOfFreedom = degreesOfFreedom; this.iterations = iterations;
		this.constraintIds = constraintIds.copy(); this.message = message;
		this.report = report;
		this.degenerate = degenerate;
		this.diagnosed = diagnosed;
	}
}
