package machinekit.screwdriving;

import machinekit.assembly.MachineAssembly;
import machinekit.component.MachineComponent;
import machinekit.component.ComponentDetail;
import machinekit.component.Solids;
import machinekit.standard.SocketHeadCapScrew;
import cadkit.modeling.Part;
import materia.assembly.AssemblyFrames;

/** Hand-loaded single-size presenter, with a sliding escapement and a physical presence volume. */
class ScrewPresenter extends MachineAssembly {
	public final screw:SocketHeadCapScrew;

	public function new(screw:SocketHeadCapScrew) {
		super(); this.screw = screw;
		addComponent("nest", new PresenterNest(screw));
		addComponent("escapement", new PresenterGate());
		addMateOnAxis("escapement-slide", "prismatic", "nest", "gate", "escapement", "mount", {x: 1, y: 0, z: 0}, 0,
			{lower: 0, upper: 15, velocity: 50, effort: 10});
		addComponent("screw", screw);
		addMate("presented-screw", "fixed", "nest", "seat", "screw", "head");
		exposeConnector("mount", "nest", "mount"); exposeConnector("pick", "screw", "head");
		addPresence("part-present", "nest/sense", screw.spec.headHeight + 2);
		exposePort("escapement", "escapement", "control");
	}
}
private class PresenterNest extends MachineComponent {
	final screw:SocketHeadCapScrew;

	public function new(screw:SocketHeadCapScrew) {
		super("CUSTOM-SCREW-PRESENTER-NEST-" + screw.designation, "Hand-loaded screw presenter nest", "aluminium 6061", true);
		this.screw = screw;
		addConnector("mount", Mount, Solids.axial(0, 0, 0));
		addConnector("gate", Mount, AssemblyFrames.translation(20, 0, 10));
		addConnector("seat", Mount, Solids.axial(0, 0, screw.length + 15));
		addConnector("sense", Mount, AssemblyFrames.translation(0, 0, screw.length + 15));
	}
	override public function hasGeometry():Bool return true;
	override public function geometry(detail:ComponentDetail = Preview):Part
		return Solids.cut(Part.box(50, 50, screw.length + 15), [Part.cylinderSpan(screw.diameter / 2 + 0.3, 14, screw.length + 15.1)]);
}
private class PresenterGate extends MachineComponent {
	public function new() {
		super("CUSTOM-SCREW-PRESENTER-GATE", "Presenter escapement", "steel", true);
		addConnector("mount", Mount, AssemblyFrames.translation(0, 0, 0));
		addPort({name: "control", kind: Signal, role: Consumer, iface: ScrewdrivingInterfaces.control(), required: true});
	}
	override public function hasGeometry():Bool return true;
	override public function geometry(detail:ComponentDetail = Preview):Part return Part.box(10, 20, 5);
}
