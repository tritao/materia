package machinekit.motion;

import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.component.ComponentDetail;
import machinekit.component.ConnectorRole;
import machinekit.component.Dimension;
import machinekit.component.MachineComponent;
import machinekit.component.Solids;
import machinekit.motion.LinearRailProfile.LinearRailProfileSpec;
import materia.project.AssemblyFrames;

/** A cut length of profile rail. The preview is a nominal envelope; mounting-hole locations
 * are exposed as named connectors and kept in the catalog-backed guide system. CAD frame: the
 * rail cross-section is centred on x=0, spans y=-railHeight..0, and runs along +Z. */
class LinearRail extends MachineComponent {
	public final spec:LinearRailProfileSpec;
	public final length:Float;
	public final holePositions:Array<Float>;

	public function new(spec:LinearRailProfileSpec, length:Float) {
		if (!(length > 2 * spec.railEndMargin)) throw 'Linear rail length must exceed twice the end margin';
		if (!(spec.railWidth > 0) || !(spec.railHeight > 0) || !(spec.railHolePitch > 0) || !(spec.railEndMargin > 0))
			throw 'Linear rail profile "${spec.designation}" has invalid rail dimensions';
		var lengthText = Dimension.format(length);
		super('${spec.family}-${spec.designation}-RAIL-L$lengthText',
			'${spec.family} ${spec.designation} profile rail, $lengthText mm long', "steel");
		this.spec = spec;
		this.length = length;
		holePositions = [];
		var position = spec.railEndMargin;
		while (position <= length - spec.railEndMargin + 1e-9) {
			holePositions.push(position);
			position += spec.railHolePitch;
		}
		addConnector("axis", Axis, Solids.axial(0, 0, 0));
		for (i in 0...holePositions.length)
			addConnector('mount${i + 1}', Mount,
				AssemblyFrames.alongY(0, 0, holePositions[i], 0, 1, 0));
	}

	override public function geometry(detail:ComponentDetail = Preview):Part {
		return Solids.prism([
			new Vector(-spec.railWidth / 2, -spec.railHeight),
			new Vector(spec.railWidth / 2, -spec.railHeight),
			new Vector(spec.railWidth / 2, 0),
			new Vector(-spec.railWidth / 2, 0),
		], 0, length);
	}
}
