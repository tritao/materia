package machinekit.robotics;

import machinekit.assembly.MachineAssembly;
import machinekit.pneumatic.PneumaticCylinder;
import machinekit.structural.BlockBracket;
import machinekit.component.Solids;
import materia.assembly.AssemblyFrames;

/** A complete, assumed double-acting pneumatic parallel gripper. One cylinder moves
 * the left jaw; the second jaw follows through a mirrored mechanical coupling. */
class ParallelGripper extends MachineAssembly {
	public static inline var FINGER_LEFT:String = "fingerLeft";
	public static inline var FINGER_RIGHT:String = "fingerRight";
	public final spec:ParallelGripperSpec;

	public function new(spec:ParallelGripperSpec) {
		super();
		if (spec == null) throw "Parallel gripper assembly needs one assumed size specification";
		this.spec = spec;
		var body = new ParallelGripperBody(spec);
		var left = new ParallelGripperJaw(spec, -1);
		var right = new ParallelGripperJaw(spec, 1);
		var cylinder:PneumaticCylinder = spec.cylinder();
		var cushion = cylinder.endStopCompliance;
		addComponent("body", body);
		addComponent("jawLeft", left);
		addComponent("jawRight", right);
		addComponent("rod", cylinder.movingRod());
		addComponent(FINGER_LEFT, new ParallelGripperFinger(spec, -1));
		addComponent(FINGER_RIGHT, new ParallelGripperFinger(spec, 1));
		addMateOnAxis("jaw-left-slide", "prismatic", "body", "leftGuide", "jawLeft", "start",
			{x: 0, y: 1, z: 0}, 0,
			{lower: 0, upper: spec.jawTravel(), velocity: spec.cylinderSpeed, effort: null, overtravel: cushion});
		addMateOnAxis("jaw-right-slide", "prismatic", "body", "rightGuide", "jawRight", "start",
			{x: 0, y: 1, z: 0}, 0,
			{lower: -spec.jawTravel(), upper: 0, velocity: spec.cylinderSpeed, effort: null, overtravel: cushion});
		addCoupling("jaw-mirror", "jaw-left-slide", "jaw-right-slide", -1);
		addMate("finger-left-mount", "fixed", "jawLeft", "fingerSeat", FINGER_LEFT, "mount");
		addMate("finger-right-mount", "fixed", "jawRight", "fingerSeat", FINGER_RIGHT, "mount");
		var cylinderPose = AssemblyFrames.compose(AssemblyFrames.translation(
			spec.cylinderStartX(), 0, spec.cylinderAxisZ()),
			AssemblyFrames.turnY(Math.PI / 2));
		addComponent("cylinder", cylinder, cylinderPose);
		// A side bracket carries the barrel without blocking its underside air ports.
		var supportZ = spec.cylinderAxisZ() - cylinder.bodyWidth / 2;
		var outsideY = spec.supportSideReach();
		var bracket = new BlockBracket("gripper cylinder support", [{
			min: [spec.cylinderStartX(), spec.bodyDepth / 2, supportZ],
			max: [-spec.bodyWidth / 2 + ParallelGripperSpec.CYLINDER_BRACKET_LAP, outsideY, spec.bodyLength - spec.jawHeight - ParallelGripperSpec.GUIDE_TONGUE_HEIGHT]
		}, {
			min: [spec.cylinderStartX(), cylinder.bodyWidth / 2, supportZ],
			max: [spec.cylinderStartX() + cylinder.bodyLength, outsideY, spec.cylinderAxisZ() + 1]
		}]);
		addComponent("cylinderBracket", bracket);
		var bracketSeat = Solids.axial(-spec.bodyWidth / 2 + ParallelGripperSpec.CYLINDER_BRACKET_LAP / 2, spec.bodyDepth / 2, supportZ + ParallelGripperSpec.CYLINDER_BRACKET_LAP);
		addMemberConnector("body", "cylinderBracketSeat", bracketSeat);
		addMemberConnector("cylinderBracket", "mount", bracketSeat);
		addMemberConnector("cylinderBracket", "cylinderSeat", body.connector("cylinderMount").frame);
		addMate("cylinder-bracket-mount", "fixed", "body", "cylinderBracketSeat", "cylinderBracket", "mount");
		addMate("cylinder-mount", "fixed", "cylinderBracket", "cylinderSeat", "cylinder", "mount");
		addMate("cylinder-rod-guide", "fixed", "jawLeft", "rodMount", "rod", "mount");
		// Two routed services are connected by the valve in the tool that mounts this assembly.
		exposePort("airA", "cylinder", "A");
		exposePort("airB", "cylinder", "B");
		exposePort("open", "body", "open");
		exposePort("close", "body", "close");
		exposeConnector("mount", "body", "mount");
		exposeConnector("tcp", "body", "tcp");
		addSwitch("jaw-open", "jaw-left-slide", {lower: -cushion,
			upper: ParallelGripperSpec.SWITCH_EDGE_CLEARANCE,
			velocity: null, effort: null}, 0.25);
		addSwitch("jaw-closed-empty", "jaw-left-slide", {
			lower: spec.jawTravel() - ParallelGripperSpec.SWITCH_EDGE_CLEARANCE,
			upper: spec.jawTravel() + cushion, velocity: null, effort: null}, 0.25);
		var gripWindow = spec.grippedTravelWindow();
		addSwitch("gripped", "jaw-left-slide", {lower: gripWindow.lower, upper: gripWindow.upper,
			velocity: null, effort: null}, 0.25);
	}
}
