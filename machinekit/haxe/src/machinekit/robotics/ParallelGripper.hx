package machinekit.robotics;

import cadkit.modeling.Part;
import machinekit.component.ComponentDetail;
import machinekit.component.Dimension;
import machinekit.component.MachineComponent;
import machinekit.component.PortInterface;
import machinekit.component.PortKind;
import machinekit.component.PortRole;
import machinekit.component.Solids;

/** Generic two-jaw pneumatic gripper envelope; stroke is the total jaw travel. */
class ParallelGripper extends MachineComponent {
	public final width:Float;
	public final depth:Float;
	public final length:Float;
	public final stroke:Float;

	public function new(width:Float, depth:Float, length:Float, stroke:Float) {
		if (!Math.isFinite(width) || width <= 0 || !Math.isFinite(depth) || depth <= 0 ||
			!Math.isFinite(length) || length <= 0 || !Math.isFinite(stroke) || stroke <= 0)
			throw "Parallel gripper needs positive dimensions and stroke";
		super('GRIPPER-${Dimension.format(width)}-${Dimension.format(stroke)}',
			"Generic pneumatic parallel gripper", "aluminium 6061", true);
		this.width = width;
		this.depth = depth;
		this.length = length;
		this.stroke = stroke;
		addConnector("mount", Mount, Solids.axial(0, 0, 0));
		addConnector("tcp", Face, Solids.axial(0, 0, length));
		addPort({name: "open", kind: Pneumatic, role: Consumer, iface: PushIn(6), required: true});
		addPort({name: "close", kind: Pneumatic, role: Consumer, iface: PushIn(6), required: true});
		addCapability(Grip(stroke, null, "open", "close"));
	}

	override public function hasGeometry():Bool return true;

	override public function geometry(detail:ComponentDetail = Preview):Part
		return Part.box(detail == Envelope ? width + stroke : width, depth, length);
}
