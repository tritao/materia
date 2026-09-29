package machinekit.robotics;

import cadkit.modeling.Part;
import machinekit.component.ComponentDetail;
import machinekit.component.Dimension;
import machinekit.component.MachineComponent;
import machinekit.component.Solids;

/** Simple rectangular robot tooling bar along local +Z. */
class FrameBar extends MachineComponent {
	public final width:Float;
	public final depth:Float;
	public final length:Float;

	public function new(width:Float, depth:Float, length:Float) {
		if (!Math.isFinite(width) || width <= 0 || !Math.isFinite(depth) || depth <= 0 ||
			!Math.isFinite(length) || length <= 0)
			throw "Frame bar needs positive dimensions";
		super('FRAME-BAR-${Dimension.format(width)}-${Dimension.format(depth)}-${Dimension.format(length)}',
			"Generic end-effector frame bar", "aluminium 6061", true);
		this.width = width;
		this.depth = depth;
		this.length = length;
		addConnector("base", Mount, Solids.axial(0, 0, 0));
		addConnector("end", Mount, Solids.axial(0, 0, length));
	}

	override public function hasGeometry():Bool return true;

	override public function geometry(detail:ComponentDetail = Preview):Part
		return Part.box(width, depth, length);
}
