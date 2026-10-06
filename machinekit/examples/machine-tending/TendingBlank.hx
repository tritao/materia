import cadkit.modeling.Part;
import machinekit.component.ComponentDetail;
import machinekit.component.ConnectorRole;
import machinekit.component.MachineComponent;
import materia.assembly.AssemblyFrames;

/** Free stock for the tending cell, with a top datum for saved pick references. */
class TendingBlank extends MachineComponent {
	public final width:Float;
	public final depth:Float;
	public final height:Float;

	public function new(width:Float, depth:Float, height:Float) {
		if (!positive(width) || !positive(depth) || !positive(height))
			throw "Tending blank needs positive stock dimensions";
		super('TENDING-BLANK-${width}x${depth}x${height}', "Aluminium bearing block blank", "aluminium 6061", true);
		this.width = width;
		this.depth = depth;
		this.height = height;
		addConnector("grasp", Face, AssemblyFrames.alongY(0, 0, height, 0, 0, 1));
	}

	override public function hasGeometry():Bool return true;

	override public function geometry(detail:ComponentDetail = Preview):Part
		return Part.box(width, depth, height);

	static function positive(value:Float):Bool return Math.isFinite(value) && value > 0;
}
