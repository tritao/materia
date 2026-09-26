package machinekit.motion;

import cadkit.modeling.Part;
import machinekit.component.ComponentDetail;
import machinekit.component.ConnectorRole;
import machinekit.component.MachineComponent;
import machinekit.component.Solids;
import machinekit.standard.SocketHeadCapScrew;

/** Rigid sleeve coupling joining two shaft ends, with an independent bore on each side (a
 * reducer coupling when they differ). Set screws are semantic (size only, not modelled),
 * matching the project's thread convention.
 * CAD frame: axis along +Z, side A face at z=0, side B face at z=length, matching
 * `DeepGrooveBearing`. Connectors: `sideA`, `sideB` (faces) and `axis` (mid-length), all with
 * +Y along +Z.
 */
class ShaftCoupling extends MachineComponent {
	public final boreA:Float;
	public final boreB:Float;
	public final outerDiameter:Float;
	public final length:Float;
	public final setScrew:String;

	public function new(boreA:Float, boreB:Float, ?outerDiameter:Float, ?length:Float) {
		if (!(boreA > 0) || !(boreB > 0)) throw "Shaft coupling needs positive bore diameters";
		var maxBore = Math.max(boreA, boreB);
		var wall = Math.max(3, maxBore * 0.4);
		var od = outerDiameter == null ? maxBore + 2 * wall : outerDiameter;
		var len = length == null ? maxBore * 3 : length;
		if (!(od > maxBore)) throw "Shaft coupling outer diameter must be larger than both bores";
		if (!(len > 0)) throw "Shaft coupling needs a positive length";
		super('COUPLING-${boreA}x${boreB}-${od}x${len}', 'Shaft coupling ${boreA} to ${boreB} mm, ${od}x${len}',
			"aluminium 6061");
		this.boreA = boreA;
		this.boreB = boreB;
		this.outerDiameter = od;
		this.length = len;
		setScrew = this.outerDiameter <= 20 ? "M3" : this.outerDiameter <= 30 ? "M4" : "M5";
		addConnector("sideA", Face, Solids.axial(0, 0, 0));
		addConnector("axis", Axis, Solids.axial(0, 0, this.length / 2));
		addConnector("sideB", Face, Solids.axial(0, 0, this.length));
	}

	override public function geometry(detail:ComponentDetail = Preview):Part {
		var body = Solids.cylinder(outerDiameter / 2, 0, length);
		var boreATool = Solids.cylinder(boreA / 2, -0.1, length / 2 + 0.1);
		var boreBTool = Solids.cylinder(boreB / 2, length / 2 - 0.1, length + 0.1);
		return Solids.cut(body, [boreATool, boreBTool]);
	}

	/** Set screw that clamps the coupling to a shaft. */
	public function setScrewPart(length:Float):SocketHeadCapScrew
		return SocketHeadCapScrew.metric(setScrew, length);
}
