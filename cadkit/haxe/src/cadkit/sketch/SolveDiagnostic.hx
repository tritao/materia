package cadkit.sketch;

class SolveDiagnostic {
	public final status:String;
	public final converged:Bool;
	public final residual:Float;
	public final degreesOfFreedom:Int;
	public final iterations:Int;
	public final constraintIds:Array<String>;
	public final message:String;

	public function new(status:String, converged:Bool, residual:Float, degreesOfFreedom:Int, iterations:Int,
		constraintIds:Array<String>, message:String) {
		this.status = status; this.converged = converged; this.residual = residual;
		this.degreesOfFreedom = degreesOfFreedom; this.iterations = iterations;
		this.constraintIds = constraintIds.copy(); this.message = message;
	}
}
