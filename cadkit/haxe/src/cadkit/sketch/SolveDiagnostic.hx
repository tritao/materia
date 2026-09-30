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

	public function new(status:String, converged:Bool, residual:Float, degreesOfFreedom:Int, iterations:Int,
		constraintIds:Array<String>, message:String, ?report:DiagnosisReport) {
		this.status = status; this.converged = converged; this.residual = residual;
		this.degreesOfFreedom = degreesOfFreedom; this.iterations = iterations;
		this.constraintIds = constraintIds.copy(); this.message = message;
		this.report = report;
	}
}
