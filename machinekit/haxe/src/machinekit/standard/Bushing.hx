package machinekit.standard;

import cadkit.modeling.Part;
import machinekit.component.ComponentDetail;
import machinekit.component.ConnectorRole;
import machinekit.component.MachineComponent;
import machinekit.component.Solids;

/** Plain sleeve bushing, sized by bore (shaft) diameter; outer diameter and length are derived
 * proportionally unless given explicitly.
 * CAD frame: axis along +Z, front face at z=0, back face at z=length, matching
 * `DeepGrooveBearing`. Connectors: `front`, `back` (faces) and `axis` (mid-length), all with +Y
 * along +Z.
 */
class Bushing extends MachineComponent {
	public final boreDiameter:Float;
	public final outerDiameter:Float;
	public final length:Float;

	public function new(boreDiameter:Float, ?outerDiameter:Float, ?length:Float) {
		if (!(boreDiameter > 0)) throw "Bushing needs a positive bore diameter";
		var wall = Math.max(1.5, boreDiameter * 0.15);
		var od = outerDiameter == null ? boreDiameter + 2 * wall : outerDiameter;
		var len = length == null ? boreDiameter * 1.5 : length;
		if (!(od > boreDiameter)) throw "Bushing outer diameter must be larger than the bore";
		if (!(len > 0)) throw "Bushing needs a positive length";
		super('BUSHING-${boreDiameter}x${od}x${len}', 'Plain bushing ${boreDiameter}x${od}x${len}', "bronze");
		this.boreDiameter = boreDiameter;
		this.outerDiameter = od;
		this.length = len;
		addConnector("front", Face, Solids.axial(0, 0, 0));
		addConnector("axis", Axis, Solids.axial(0, 0, this.length / 2));
		addConnector("back", Face, Solids.axial(0, 0, this.length));
	}

	override public function geometry(detail:ComponentDetail = Preview):Part
		return Solids.cut(Solids.cylinder(outerDiameter / 2, 0, length),
			[Solids.cylinder(boreDiameter / 2, -0.1, length + 0.1)]);
}
