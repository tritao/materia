package machinekit.robotics;

import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.component.ComponentDetail;
import machinekit.component.ComponentType;
import machinekit.component.ComponentValues;
import machinekit.component.ComponentRecipeSupport;
import machinekit.component.CollisionHullFacet;
import machinekit.component.MachineComponent;
import machinekit.component.Solids;

/** Assumed replaceable contact pad sized for the bearing-block blank. */
class ParallelGripperFinger extends MachineComponent {
	public final spec:ParallelGripperSpec;
	public final side:Int;
	static var recipe:Null<ComponentType>;

	public function new(spec:ParallelGripperSpec, side:Int = -1) {
		if (spec == null || (side != -1 && side != 1)) throw "Gripper finger needs its size row and side";
		super('GRIPPER-${spec.size}-PAD-${spec.fingerWidth}x${spec.fingerDepth}x${spec.fingerLength}-SIDE-${side}',
			"Assumed soft parallel-gripper contact finger", "rubber");
		this.spec = spec;
		this.side = side;
		// Part.box starts at local z=0, so the mount is the wrist-side end and
		// +Z reaches the finger tip at the gripper TCP plane.
		addConnector("mount", Mount, Solids.axial(0, 0, 0));
		if (spec.supportStep) {
			var halfWidth = spec.fingerWidth / 2.0;
			var overlap = ParallelGripperSpec.FINGER_SUPPORT_STEP_ROOT_OVERLAP;
			var projection = ParallelGripperSpec.FINGER_SUPPORT_STEP_PROJECTION;
			var xMin = side < 0 ? halfWidth - overlap : -halfWidth - projection;
			var xMax = side < 0 ? halfWidth + projection : -halfWidth + overlap;
			var zMin = spec.fingerLength;
			var zMax = spec.fingerLength + projection;
			addFacet(CollisionHullFacet.fromBoxes([
				[-halfWidth, -spec.fingerDepth / 2.0, 0, halfWidth, spec.fingerDepth / 2.0, spec.fingerLength],
				[xMin, -spec.fingerDepth / 2.0, zMin, xMax, spec.fingerDepth / 2.0, zMax]
			]));
		}
	}

	public static function recipeType():ComponentType {
		if (recipe == null) recipe = new ComponentType("machinekit.robotics.parallel-gripper-finger", [
			ComponentRecipeSupport.count("size", 32),
			ComponentRecipeSupport.length("expectedPartWidth", 60),
			ComponentRecipeSupport.length("fingerLength", 60),
			ComponentRecipeSupport.length("openingTravel", 16),
			ComponentRecipeSupport.choice("side", ["left", "right"], "left"),
			ComponentRecipeSupport.flag("supportStep", false)
		], v -> new ParallelGripperFinger(new ParallelGripperSpec(v.integer("size"),
			v.number("expectedPartWidth"), v.number("fingerLength"), v.number("openingTravel"),
			v.boolean("supportStep")),
			v.token("side") == "left" ? -1 : 1), true);
		return recipe;
	}

	override public function componentType():Null<ComponentType>
		return Std.isExactType(this, ParallelGripperFinger) ? recipeType() : null;

	override public function values():ComponentValues return new ComponentValues()
		.setInteger("size", spec.size).setNumber("expectedPartWidth", spec.expectedPartWidth)
		.setNumber("fingerLength", spec.fingerLength).setNumber("openingTravel", spec.openingTravel)
		.setToken("side", side < 0 ? "left" : "right")
		.setBoolean("supportStep", spec.supportStep)
		.setToken("material", materialSpec());

	override public function hasGeometry():Bool return true;
	override public function geometry(detail:ComponentDetail = Preview):Part {
		var finger = Part.box(spec.fingerWidth, spec.fingerDepth, spec.fingerLength);
		if (!spec.supportStep) return finger;
		var overlap = ParallelGripperSpec.FINGER_SUPPORT_STEP_ROOT_OVERLAP;
		var toeWidth = ParallelGripperSpec.FINGER_SUPPORT_STEP_PROJECTION + overlap;
		var toeLength = ParallelGripperSpec.FINGER_SUPPORT_STEP_PROJECTION;
		var toeCentreX = -side * (spec.fingerWidth / 2.0 +
			(ParallelGripperSpec.FINGER_SUPPORT_STEP_PROJECTION - overlap) / 2.0);
		var toe = Part.box(toeWidth, spec.fingerDepth, toeLength).translated(
			new Vector(toeCentreX, 0, spec.fingerLength));
		return Solids.union([finger, toe]);
	}
}
