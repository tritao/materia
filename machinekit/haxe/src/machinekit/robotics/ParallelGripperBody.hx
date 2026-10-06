package machinekit.robotics;

import cadkit.modeling.Part;
import machinekit.component.ComponentDetail;
import machinekit.component.ComponentType;
import machinekit.component.ComponentValues;
import machinekit.component.ComponentRecipeSupport;
import machinekit.component.MachineComponent;
import machinekit.component.PortKind;
import machinekit.component.PortRole;
import machinekit.component.Solids;
import materia.assembly.AssemblyFrames;

/** Fixed housing and design interface for a two-jaw gripper. The complete
 * mechanism, moving jaws and pneumatic drive live in `ParallelGripper`. */
class ParallelGripperBody extends MachineComponent {
	public final spec:ParallelGripperSpec;
	static var recipe:Null<ComponentType>;

	public function new(spec:ParallelGripperSpec) {
		if (spec == null) throw "Parallel gripper body needs its assumed size row";
		super('GRIPPER-${spec.size}-${spec.expectedPartWidth}', "Reference pneumatic parallel-gripper housing", "aluminium 6061");
		this.spec = spec;
		var centre = spec.openJawCentre();
		addConnector("mount", Mount, Solids.axial(0, 0, 0));
		addConnector("leftGuide", Axis, AssemblyFrames.alongY(-centre, 0,
			spec.bodyLength - spec.jawHeight, 1, 0, 0));
		addConnector("rightGuide", Axis, AssemblyFrames.alongY(centre, 0,
			spec.bodyLength - spec.jawHeight, 1, 0, 0));
		var cylinder = spec.cylinder();
		var cylinderAxis = AssemblyFrames.compose(AssemblyFrames.turnY(Math.PI / 2.0),
			cylinder.connector("mount").frame);
		addConnector("cylinderMount", Mount, AssemblyFrames.compose(
			AssemblyFrames.translation(-centre - cylinder.bodyLength - cylinder.stemLength, 0,
				spec.bodyLength - spec.jawHeight), cylinderAxis));
		// Keep the valve at the wrist-side end of the housing so its envelope does
		// not occupy the finger approach or the part-sensing zone.
		addConnector("valveSeat", Mount, Solids.axial(spec.bodyWidth / 2.0 - 30, 0, spec.bodyLength * 0.25));
		// Grasp planning uses tool +X for jaw travel and +Z for finger insertion.
		// The mechanism's guide axes are already body +X, with fingers along +Z.
		addConnector("tcp", Face, {x: 0, y: 0, z: spec.bodyLength + spec.fingerLength,
			qx: 0, qy: 0, qz: 0, qw: 1});
		addPort({name: "open", kind: Signal, role: Consumer, iface: Plug("24V-input", 2), required: false});
		addPort({name: "close", kind: Signal, role: Consumer, iface: Plug("24V-input", 2), required: false});
		addFacet(new GripFacet(spec.totalJawTravel, spec.ratedGripForce, "open", "close", "gripped", "tcp",
			spec.minimumPartWidth(), spec.expectedPartWidth, spec.maximumPartWidth(), spec.fingerLength,
			spec.fingerPadFrictionCoefficient, spec.supportStep));
	}

	public static function recipeType():ComponentType {
		if (recipe == null) recipe = new ComponentType("machinekit.robotics.parallel-gripper-body", [
		ComponentRecipeSupport.count("size", 32), ComponentRecipeSupport.length("expectedPartWidth", 60),
			ComponentRecipeSupport.length("fingerLength", 60), ComponentRecipeSupport.length("openingTravel", 16),
			ComponentRecipeSupport.flag("supportStep", false)
		], v -> new ParallelGripperBody(new ParallelGripperSpec(v.integer("size"),
			v.number("expectedPartWidth"), v.number("fingerLength"), v.number("openingTravel"),
			v.boolean("supportStep"))), true);
		return recipe;
	}

	override public function componentType():Null<ComponentType>
		return Std.isExactType(this, ParallelGripperBody) ? recipeType() : null;

	override public function values():ComponentValues return new ComponentValues()
		.setInteger("size", spec.size).setNumber("expectedPartWidth", spec.expectedPartWidth)
		.setNumber("fingerLength", spec.fingerLength).setNumber("openingTravel", spec.openingTravel)
		.setBoolean("supportStep", spec.supportStep)
		.setToken("material", materialSpec());

	override public function hasGeometry():Bool return true;
	override public function geometry(detail:ComponentDetail = Preview):Part
		return Part.box(spec.bodyWidth, spec.bodyDepth, spec.bodyLength);
}
