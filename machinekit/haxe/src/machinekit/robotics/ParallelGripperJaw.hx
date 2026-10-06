package machinekit.robotics;

import cadkit.modeling.Part;
import machinekit.component.ComponentDetail;
import machinekit.component.ComponentType;
import machinekit.component.ComponentValues;
import machinekit.component.ComponentRecipeSupport;
import machinekit.component.MachineComponent;
import machinekit.component.Solids;
import materia.assembly.AssemblyFrames;

/** One moving carriage. The second jaw uses the same part and the mirrored
 * prismatic coupling; its handed finger seat is derived from `side`. */
class ParallelGripperJaw extends MachineComponent {
	public final spec:ParallelGripperSpec;
	public final side:Int;
	static var recipe:Null<ComponentType>;

	public function new(spec:ParallelGripperSpec, side:Int) {
		if (spec == null || (side != -1 && side != 1)) throw "Gripper jaw needs a size row and side -1 or +1";
		super('GRIPPER-${spec.size}-JAW-${side}', "Reference gripper jaw carriage", "steel");
		this.spec = spec;
		this.side = side;
		addConnector("start", Axis, AssemblyFrames.alongY(0, 0, 0, 1, 0, 0));
		addConnector("cylinderSeat", Axis, AssemblyFrames.alongY(0, 0, 0, 1, 0, 0));
		addConnector("rodMount", Mount, AssemblyFrames.compose(AssemblyFrames.turnY(Math.PI / 2.0),
			Solids.axial(0, 0, 0)));
		addConnector("fingerSeat", Mount, Solids.axial(-side *
			(spec.jawWidth - spec.fingerWidth) / 2.0, 0, spec.jawHeight));
	}

	public static function recipeType():ComponentType {
		if (recipe == null) recipe = new ComponentType("machinekit.robotics.parallel-gripper-jaw", [
			ComponentRecipeSupport.count("size", 32), ComponentRecipeSupport.choice("side", ["left", "right"], "left")
		], v -> new ParallelGripperJaw(new ParallelGripperSpec(v.integer("size")),
			v.token("side") == "left" ? -1 : 1), true);
		return recipe;
	}

	override public function componentType():Null<ComponentType>
		return Std.isExactType(this, ParallelGripperJaw) ? recipeType() : null;

	override public function values():ComponentValues return new ComponentValues()
		.setInteger("size", spec.size).setToken("side", side < 0 ? "left" : "right")
		.setToken("material", materialSpec());

	override public function hasGeometry():Bool return true;
	override public function geometry(detail:ComponentDetail = Preview):Part
		return Part.box(spec.jawWidth, spec.bodyDepth, spec.jawHeight);
}
