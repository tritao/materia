package pickingstation;

import cadkit.modeling.Align;
import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.component.ComponentDetail;
import machinekit.component.ConnectorRole;
import machinekit.component.MachineComponent;
import machinekit.component.Solids;

typedef StorageBinEnvelope = {
	var width:Float;
	var depth:Float;
	var height:Float;
}

/** Open-top storage bin. The Envelope detail is the external package envelope. */
class StorageBin extends MachineComponent {
	public final width:Float;
	public final depth:Float;
	public final height:Float;
	public final wallThickness:Float;

	public function new(width:Float, depth:Float, height:Float, wallThickness:Float = 3) {
		if (!positive(width) || !positive(depth) || !positive(height) || !positive(wallThickness))
			throw "Storage bin dimensions must be finite and positive";
		if (2 * wallThickness >= Math.min(width, Math.min(depth, height)))
			throw "Storage bin wall thickness is too large";
		this.width = width;
		this.depth = depth;
		this.height = height;
		this.wallThickness = wallThickness;
		super('BIN-${fmt(width)}x${fmt(depth)}x${fmt(height)}',
			'Open-top storage bin ${fmt(width)} x ${fmt(depth)} x ${fmt(height)} mm');
		addConnector("base", ConnectorRole.Mount, Solids.axial(0, 0, 0));
		addConnector("pick", ConnectorRole.Face, Solids.axial(0, -depth / 2, height));
	}

	public function externalEnvelope():StorageBinEnvelope return {width: width, depth: depth, height: height};

	override public function geometry(detail:ComponentDetail = Preview):Part {
		if (detail == Envelope) return Part.box(width, depth, height, Align.Center, Align.Center, Align.Min);
		var bottom = Part.box(width, depth, wallThickness, Align.Center, Align.Center, Align.Min);
		var left = Part.box(wallThickness, depth, height, Align.Center, Align.Center, Align.Min)
			.translated(new Vector(-(width - wallThickness) / 2, 0, 0));
		var right = Part.box(wallThickness, depth, height, Align.Center, Align.Center, Align.Min)
			.translated(new Vector((width - wallThickness) / 2, 0, 0));
		var front = Part.box(width - 2 * wallThickness, wallThickness, height,
			Align.Center, Align.Center, Align.Min).translated(new Vector(0, -(depth - wallThickness) / 2, 0));
		var back = Part.box(width - 2 * wallThickness, wallThickness, height,
			Align.Center, Align.Center, Align.Min).translated(new Vector(0, (depth - wallThickness) / 2, 0));
		return Solids.union([bottom, left, right, front, back]);
	}

	static function positive(value:Float):Bool return Math.isFinite(value) && value > 0;
	static function fmt(value:Float):String return Std.string(Math.round(value * 10) / 10);
}
