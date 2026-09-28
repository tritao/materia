package pickingstation;

import cadkit.modeling.Align;
import cadkit.modeling.Axis;
import cadkit.modeling.Location;
import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.component.ComponentDetail;
import machinekit.component.ConnectorRole;
import machinekit.component.MachineComponent;
import machinekit.component.Solids;

/** Flat sheet shelf with separate supports and a front retaining lip. */
class ShelfAssembly extends MachineComponent {
	public final width:Float;
	public final depth:Float;
	public final thickness:Float;
	public final inclinationDegrees:Float;
	public static inline var RETAINING_LIP_HEIGHT:Float = 55;
	public static inline var RETAINING_LIP_THICKNESS:Float = 18;

	public function new(width:Float, depth:Float, thickness:Float = 18, inclinationDegrees:Float = 0) {
		if (!positive(width) || !positive(depth) || !positive(thickness))
			throw "Shelf dimensions must be finite and positive";
		if (!Math.isFinite(inclinationDegrees) || Math.abs(inclinationDegrees) > 30)
			throw "Shelf inclination must be between -30 and 30 degrees";
		this.width = width;
		this.depth = depth;
		this.thickness = thickness;
		this.inclinationDegrees = inclinationDegrees;
		super('SHELF-${fmt(width)}x${fmt(depth)}',
			'Sheet shelf with supports and retaining lip', "birch plywood");
		addConnector("support", ConnectorRole.Mount, Solids.axial(0, 0, 0));
	}

	override public function geometry(detail:ComponentDetail = Preview):Part {
		var panel = Part.box(width, depth, thickness, Align.Center, Align.Center, Align.Min);
		if (detail == Envelope) return incline(panel);
		var supportWidth = Math.min(40, width / 4);
		var supportHeight = Math.max(20, thickness * 1.5);
		var supportDepth = Math.max(20, depth - 30);
		var supportA = Part.box(supportWidth, supportDepth, supportHeight,
			Align.Center, Align.Center, Align.Min).translated(new Vector(-width / 2 + supportWidth, 0, -supportHeight));
		var supportB = Part.box(supportWidth, supportDepth, supportHeight,
			Align.Center, Align.Center, Align.Min).translated(new Vector(width / 2 - supportWidth, 0, -supportHeight));
		var lip = Part.box(width, RETAINING_LIP_THICKNESS, RETAINING_LIP_HEIGHT,
			Align.Center, Align.Center, Align.Min).translated(new Vector(0,
			-depth / 2 + RETAINING_LIP_THICKNESS / 2, thickness));
		return incline(Solids.union([panel, supportA, supportB, lip]));
	}

	function incline(part:Part):Part {
		if (inclinationDegrees == 0) return part;
		var rotated = part.placed(Location.rotation(Axis.X(), inclinationDegrees * Math.PI / 180));
		part.close();
		return rotated;
	}

	static function positive(value:Float):Bool return Math.isFinite(value) && value > 0;
	static function fmt(value:Float):String return Std.string(Math.round(value * 10) / 10);
}
