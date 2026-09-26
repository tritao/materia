package machinekit.motion;

import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.component.ComponentDetail;
import machinekit.component.ConnectorRole;
import machinekit.component.MachineComponent;
import machinekit.component.Solids;
import machinekit.motion.LinearRailProfile.LinearRailProfileSpec;
import materia.project.AssemblyFrames;

/** Matching carriage block for a profile rail. Geometry is a nominal exterior envelope; the
 * two mounting locations and rail axis are exact interface data from the profile catalog. CAD
 * frame: the block is centred on z=0, with its rail-facing side at y=0. */
class LinearRailBlock extends MachineComponent {
	public final spec:LinearRailProfileSpec;

	public function new(spec:LinearRailProfileSpec) {
		if (!(spec.blockWidth > 0) || !(spec.blockHeight > 0) || !(spec.blockLength > 0) ||
			!(spec.blockHoleSpacing > 0) || spec.blockHoleSpacing > spec.blockLength)
			throw 'Linear rail profile "${spec.designation}" has invalid block dimensions';
		super('${spec.family}-${spec.designation}-BLOCK',
			'${spec.family} ${spec.designation} carriage block', "steel");
		this.spec = spec;
		addConnector("rail", Axis, Solids.axial(0, 0, 0));
		addConnector("axis", Axis, Solids.axial(0, 0, 0));
		for (i in 0...2) {
			var z = (i == 0 ? -1 : 1) * spec.blockHoleSpacing / 2;
			addConnector('mount${i + 1}', Mount, AssemblyFrames.alongY(0, spec.blockHeight, z, 0, 1, 0));
		}
	}

	override public function geometry(detail:ComponentDetail = Preview):Part {
		return Solids.prism([
			new Vector(-spec.blockWidth / 2, 0), new Vector(spec.blockWidth / 2, 0),
			new Vector(spec.blockWidth / 2, spec.blockHeight), new Vector(-spec.blockWidth / 2, spec.blockHeight),
		], -spec.blockLength / 2, spec.blockLength / 2);
	}
}
