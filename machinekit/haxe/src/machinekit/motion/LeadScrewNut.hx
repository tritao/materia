package machinekit.motion;

import cadkit.modeling.Part;
import machinekit.component.ComponentDetail;
import machinekit.component.ConnectorRole;
import machinekit.component.MachineComponent;
import machinekit.component.Solids;
import machinekit.standard.ClearanceFit;
import machinekit.standard.SocketHeadCapScrew;

/** ACME/trapezoidal lead screw nut: a flanged block with a bore matching the screw diameter and
 * a mounting bolt pattern on the flange face, for driving a carriage. The thread itself is
 * semantic (screw diameter and lead only, not modelled), matching the project's convention.
 * `travelPerRevolution()`/`rotationFor()` convert between screw rotation and nut travel.
 * CAD frame: axis along +Z, body from z=0 to z=bodyLength, flange from there to
 * z=bodyLength+flangeThickness. Connectors: `bore` (axis, mid-body) and `mount1`..`mountN` (on
 * the flange face), all with +Y along +Z.
 */
class LeadScrewNut extends MachineComponent {
	public final screwDiameter:Float;
	public final lead:Float;
	public final bodyDiameter:Float;
	public final bodyLength:Float;
	public final flangeDiameter:Float;
	public final flangeThickness:Float;
	public final boltCircleDiameter:Float;
	public final boltCount:Int;
	public final mountScrew:String;

	public function new(screwDiameter:Float, lead:Float, boltCount:Int = 4) {
		if (!(screwDiameter > 0)) throw "Lead screw nut needs a positive screw diameter";
		if (!(lead > 0)) throw "Lead screw nut needs a positive lead";
		if (boltCount < 3) throw "Lead screw nut needs at least 3 mounting bolts";
		var bodyDia = screwDiameter * 1.8;
		var bodyLen = screwDiameter * 2;
		var flangeDia = screwDiameter * 3;
		var flangeThick = Math.max(3, screwDiameter * 0.3);
		var mountScrewSize = flangeDia <= 20 ? "M3" : flangeDia <= 35 ? "M4" : "M5";
		super('LEADNUT-D${screwDiameter}-L${lead}', 'Lead screw nut, ${screwDiameter} mm screw, ${lead} mm lead', "bronze");
		this.screwDiameter = screwDiameter;
		this.lead = lead;
		bodyDiameter = bodyDia;
		bodyLength = bodyLen;
		flangeDiameter = flangeDia;
		flangeThickness = flangeThick;
		boltCircleDiameter = flangeDia * 0.75;
		this.boltCount = boltCount;
		mountScrew = mountScrewSize;
		addConnector("bore", Axis, Solids.axial(0, 0, bodyLength / 2));
		var i = 1;
		for (point in boltPattern())
			addConnector('mount${i++}', Mount, Solids.axial(point.x, point.y, bodyLength + flangeThickness));
	}

	/** Mount bolt centres on the flange face, counter-clockwise from angle 0. */
	public function boltPattern():Array<{x:Float, y:Float}> {
		var r = boltCircleDiameter / 2;
		return [for (i in 0...boltCount) {
			var angle = 2 * Math.PI * i / boltCount;
			{x: r * Math.cos(angle), y: r * Math.sin(angle)};
		}];
	}

	/** Linear travel for one screw revolution. */
	public function travelPerRevolution():Float
		return lead;

	/** Screw rotation, in radians, needed to travel `distance`. */
	public function rotationFor(distance:Float):Float
		return distance / lead * 2 * Math.PI;

	/** Screw that fits the nut's mounting holes. */
	public function mountScrewPart(length:Float):SocketHeadCapScrew
		return SocketHeadCapScrew.metric(mountScrew, length);

	override public function geometry(detail:ComponentDetail = Preview):Part {
		var body = Solids.cylinder(bodyDiameter / 2, 0, bodyLength);
		var flange = Solids.cylinder(flangeDiameter / 2, bodyLength, bodyLength + flangeThickness);
		var solidPart = Solids.union([body, flange]);
		var boreTool = Solids.cylinder(screwDiameter / 2, -0.1, bodyLength + flangeThickness + 0.1);
		if (detail == Envelope) return Solids.cut(solidPart, [boreTool]);
		var screw = mountScrewPart(10);
		var tools = [boreTool];
		for (point in boltPattern())
			tools.push(Solids.cylinder(screw.clearanceDiameter(Medium) / 2, bodyLength - 0.1,
				bodyLength + flangeThickness + 0.1, point.x, point.y));
		return Solids.cut(solidPart, tools);
	}
}
