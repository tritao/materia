package machinekit.picking;

import cadkit.modeling.Align;
import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.component.ComponentDetail;
import machinekit.component.ConnectorRole;
import machinekit.component.MachineComponent;
import machinekit.component.Solids;

/** Simplified pick indicator housing, light, quantity display, and confirmation target. */
class PickIndicator extends MachineComponent {
	public final width:Float;
	public final depth:Float;
	public final height:Float;
	public final confirmationDiameter:Float;

	public function new(width:Float = 70, depth:Float = 24, height:Float = 48,
		confirmationDiameter:Float = 18) {
		if (!positive(width) || !positive(depth) || !positive(height) || !positive(confirmationDiameter))
			throw "Pick indicator dimensions must be finite and positive";
		this.width = width;
		this.depth = depth;
		this.height = height;
		this.confirmationDiameter = confirmationDiameter;
		super('PICK-INDICATOR-${fmt(width)}',
			'Pick indicator housing with light, quantity display, and confirm target');
		addConnector("mount", ConnectorRole.Mount, Solids.axial(0, 0, 0));
		addConnector("confirmation", ConnectorRole.Face, Solids.axial(0, -depth / 2, height * 0.25));
	}

	override public function geometry(detail:ComponentDetail = Preview):Part {
		var housing = Part.box(width, depth, height, Align.Center, Align.Center, Align.Min);
		if (detail == Envelope) return housing;
		var display = Part.box(width * 0.62, 2, height * 0.20,
			Align.Center, Align.Center, Align.Min).translated(new Vector(0, -depth / 2 - 1, height * 0.48));
		var light = Part.cylinderAlongY(width * 0.12, -depth / 2 - 3, -depth / 2,
			0, height * 0.78);
		var target = Part.cylinderAlongY(confirmationDiameter / 2, -depth / 2 - 3, -depth / 2,
			0, height * 0.22);
		return Solids.union([housing, display, light, target]);
	}

	static function positive(value:Float):Bool return Math.isFinite(value) && value > 0;
	static function fmt(value:Float):String return Std.string(Math.round(value * 10) / 10);
}
