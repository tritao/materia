package machinekit.robotics;

import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.component.ComponentDetail;
import machinekit.component.ComponentType;
import machinekit.component.ComponentValues;
import machinekit.component.ComponentRecipeSupport;
import machinekit.component.CollisionHullFacet;
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
		addConnector("leftGuide", Axis, AssemblyFrames.alongY(-centre - spec.jawWidth / 2, 0,
			spec.cylinderAxisZ(), 1, 0, 0));
		addConnector("rightGuide", Axis, AssemblyFrames.alongY(centre - spec.jawWidth / 2, 0,
			spec.cylinderAxisZ(), 1, 0, 0));
		var cylinder = spec.cylinder();
		var cylinderAxis = AssemblyFrames.compose(AssemblyFrames.turnY(Math.PI / 2.0),
			cylinder.connector("mount").frame);
		addConnector("cylinderMount", Mount, AssemblyFrames.compose(
			AssemblyFrames.translation(spec.cylinderStartX(), 0, spec.cylinderAxisZ()), cylinderAxis));
		// Mount the valve flush on the outside -Y face, with its ports pointing away
		// from the housing. The former internal seat buried most of the valve in solid material.
		addConnector("valveSeat", Mount, AssemblyFrames.compose(
			AssemblyFrames.translation(spec.valveSeatX(), -spec.bodyDepth / 2, spec.bodyLength * 0.25),
			AssemblyFrames.compose(
				{x: 0, y: 0, z: 0, qx: Math.sin(Math.PI / 4), qy: 0, qz: 0, qw: Math.cos(Math.PI / 4)},
				Solids.axial(0, 0, 0))));
		// Grasp planning uses tool +X for jaw travel and +Z for finger insertion.
		// The mechanism's guide axes are already body +X, with fingers along +Z.
		addConnector("tcp", Face, {x: 0, y: 0, z: spec.bodyLength + spec.fingerLength,
			qx: 0, qy: 0, qz: 0, qw: 1});
		addPort({name: "open", kind: Signal, role: Consumer, iface: Plug("24V-input", 2), required: false});
		addPort({name: "close", kind: Signal, role: Consumer, iface: Plug("24V-input", 2), required: false});
		addFacet(new GripFacet(spec.totalJawTravel, spec.ratedGripForce, "open", "close", "gripped", "tcp",
			spec.minimumPartWidth(), spec.expectedPartWidth, spec.maximumPartWidth(), spec.fingerLength,
			spec.fingerPadFrictionCoefficient, [ParallelGripper.FINGER_LEFT, ParallelGripper.FINGER_RIGHT], spec.supportStep));
		addFacet(CollisionHullFacet.fromBoxes(housingCollisionBoxes(spec)));
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

	/** The two carriages slide through the front of the housing. Keep their full
	 * swept envelope open in both the CAD solid and the collision pieces; one box
	 * would fill those slots and report the designed jaw motion as a collision.
	 */
	static function housingCollisionBoxes(spec:ParallelGripperSpec):Array<Array<Float>> {
		var clearance = ParallelGripperSpec.HOUSING_SIDE_CLEARANCE;
		var guideZ = spec.bodyLength - spec.jawHeight;
		var reliefZ = guideZ - ParallelGripperSpec.GUIDE_TONGUE_HEIGHT;
		var innerHalf = spec.openJawCentre() - spec.jawTravel() - spec.jawWidth / 2 - spec.guideTongueReach() - clearance;
		if (reliefZ <= 0 || innerHalf <= 0) throw "Gripper housing needs room for its swept jaw reliefs";
		return [
			[-spec.bodyWidth / 2, -spec.bodyDepth / 2, 0,
				spec.bodyWidth / 2, spec.bodyDepth / 2, reliefZ],
			[-innerHalf, -spec.bodyDepth / 2, reliefZ,
				innerHalf, spec.bodyDepth / 2, spec.bodyLength]
		];
	}

	override public function hasGeometry():Bool return true;
	override public function geometry(detail:ComponentDetail = Preview):Part {
		var clearance = ParallelGripperSpec.HOUSING_SIDE_CLEARANCE;
		var guideZ = spec.bodyLength - spec.jawHeight;
		var reliefZ = guideZ - ParallelGripperSpec.GUIDE_TONGUE_HEIGHT;
		var innerHalf = spec.openJawCentre() - spec.jawTravel() - spec.jawWidth / 2 - spec.guideTongueReach() - clearance;
		if (reliefZ <= 0 || innerHalf <= 0) throw "Gripper housing needs room for its swept jaw reliefs";
		var body = Solids.named(Part.box(spec.bodyWidth, spec.bodyDepth, spec.bodyLength), "housing");
		var notchHeight = spec.bodyLength - reliefZ + clearance;
		var notchDepth = spec.bodyDepth + 2 * clearance;
		var leftMin = -spec.openJawCentre() - spec.jawWidth / 2 - spec.guideTongueReach() - clearance;
		var leftMax = -spec.openJawCentre() + spec.jawTravel() + spec.jawWidth / 2 +
			spec.guideTongueReach() + clearance;
		var rightMin = -leftMax, rightMax = -leftMin;
		// Give each boolean operand its design role before cutting. The housing's
		// broad faces split around the two reliefs; unique feature tags keep those
		// resulting names strong instead of relying on boolean input ordinals.
		return Solids.building([
			body,
			Solids.named(Part.box(leftMax - leftMin, notchDepth, notchHeight).translated(
				new Vector((leftMin + leftMax) / 2, 0, reliefZ)), "left-jaw-relief"),
			Solids.named(Part.box(rightMax - rightMin, notchDepth, notchHeight).translated(
				new Vector((rightMin + rightMax) / 2, 0, reliefZ)), "right-jaw-relief")
		], parts -> Solids.cut(parts[0], [parts[1], parts[2]]));
	}
}
