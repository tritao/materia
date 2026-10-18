package machinekit.robotics;

import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.component.CollisionHullFacet;
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
		addConnector("start", Axis, AssemblyFrames.alongY(-spec.jawWidth / 2, 0,
			spec.cylinderAxisZ() - (spec.bodyLength - spec.jawHeight), 1, 0, 0));
		addConnector("cylinderSeat", Axis, AssemblyFrames.alongY(0, 0, 0, 1, 0, 0));
		addConnector("rodMount", Mount, AssemblyFrames.compose(
			AssemblyFrames.translation(-spec.jawWidth / 2, 0, spec.cylinderAxisZ() - (spec.bodyLength - spec.jawHeight)),
			AssemblyFrames.compose(AssemblyFrames.turnY(Math.PI / 2.0), Solids.axial(0, 0, 0))));
		var reach = spec.guideTongueReach();
		var inner = -side * spec.jawWidth / 2;
		addFacet(CollisionHullFacet.fromBoxes([
			[-spec.jawWidth / 2, -spec.bodyDepth / 2, 0, spec.jawWidth / 2, spec.bodyDepth / 2, spec.jawHeight],
			[Math.min(inner, inner - side * reach), -spec.bodyDepth / 2, -ParallelGripperSpec.GUIDE_TONGUE_HEIGHT,
				Math.max(inner, inner - side * reach), spec.bodyDepth / 2, 0]
		]));
		addConnector("fingerSeat", Mount, Solids.axial(-side *
			(spec.jawWidth - spec.fingerWidth) / 2.0, 0, spec.jawHeight));
	}

	public static function recipeType():ComponentType {
		if (recipe == null) recipe = new ComponentType("machinekit.robotics.parallel-gripper-jaw", [
			ComponentRecipeSupport.count("size", 32),
			ComponentRecipeSupport.length("expectedPartWidth", 60),
			ComponentRecipeSupport.length("openingTravel", 16), ComponentRecipeSupport.choice("side", ["left", "right"], "left"),
			ComponentRecipeSupport.optionalLength("fingerWidth")
		], v -> new ParallelGripperJaw(new ParallelGripperSpec(v.integer("size"), v.number("expectedPartWidth"), 60, v.number("openingTravel"), false,
			null, v.optionalNumber("fingerWidth")),
			v.token("side") == "left" ? -1 : 1), true);
		return recipe;
	}

	override public function componentType():Null<ComponentType>
		return Std.isExactType(this, ParallelGripperJaw) ? recipeType() : null;

	override public function values():ComponentValues return new ComponentValues()
		.setInteger("size", spec.size).setNumber("expectedPartWidth", spec.expectedPartWidth)
		.setNumber("openingTravel", spec.openingTravel).setToken("side", side < 0 ? "left" : "right")
		.setNumber("fingerWidth", spec.fingerWidth)
		.setToken("material", materialSpec());

	override public function hasGeometry():Bool return true;
	override public function geometry(detail:ComponentDetail = Preview):Part
		return Solids.building([], parts -> {
			parts.push(Part.box(spec.jawWidth, spec.bodyDepth, spec.jawHeight));
			parts.push(Part.box(spec.guideTongueReach(), spec.bodyDepth,
				ParallelGripperSpec.GUIDE_TONGUE_HEIGHT).translated(new Vector(
					-side * (spec.jawWidth + spec.guideTongueReach()) / 2,
					0, -ParallelGripperSpec.GUIDE_TONGUE_HEIGHT)));
			return Solids.union(parts);
		});
}
