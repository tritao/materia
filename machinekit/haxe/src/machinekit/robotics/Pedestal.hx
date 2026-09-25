package machinekit.robotics;

import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.component.ComponentDetail;
import machinekit.component.ConnectorRole;
import machinekit.component.MachineComponent;
import machinekit.component.Solids;
import machinekit.standard.ClearanceFit;
import machinekit.standard.SocketHeadCapScrew;

/** Column stand carrying a `RobotFlange` mount on top and its own floor bolt pattern at the
 * base, for mounting a robot arm above a machine bed.
 * CAD frame: floor face at z=0, top mounting face at z=height. Connectors: `floor`, `top` (the
 * flange mount), and `floorBolt1`..`floorBoltN`, all with +Y along +Z.
 */
class Pedestal extends MachineComponent {
	public final flange:RobotFlange;
	public final height:Float;
	public final columnDiameter:Float;
	public final baseDiameter:Float;
	public final baseThickness:Float;
	public final floorBoltCircleDiameter:Float;
	public final floorBoltCount:Int;
	public final floorMountScrew:String;

	public function new(flange:RobotFlange, height:Float, ?columnDiameter:Float, floorBoltCount:Int = 4) {
		if (!(height > flange.pilotHeight + 5)) throw "Pedestal height must clear the flange's pilot boss";
		if (floorBoltCount < 3) throw "Pedestal needs at least 3 floor bolts";
		var column = columnDiameter == null ? flange.flangeDiameter : columnDiameter;
		if (!(column > 0)) throw "Pedestal needs a positive column diameter";
		super('PEDESTAL-${flange.flangeDiameter}-${height}', 'Pedestal for ${flange.designation}, ${height} mm tall', "steel");
		this.flange = flange;
		this.height = height;
		this.columnDiameter = column;
		baseDiameter = column * 1.6;
		baseThickness = Math.max(10, column * 0.2);
		floorBoltCircleDiameter = baseDiameter * 0.8;
		this.floorBoltCount = floorBoltCount;
		floorMountScrew = baseDiameter <= 100 ? "M8" : baseDiameter <= 160 ? "M10" : "M12";
		addConnector("floor", Face, Solids.axial(0, 0, 0));
		addConnector("top", Mount, Solids.axial(0, 0, this.height));
		var i = 1;
		for (point in floorBoltPattern()) addConnector('floorBolt${i++}', Mount, Solids.axial(point.x, point.y, 0));
	}

	/** Floor bolt centres, counter-clockwise from angle 0. */
	public function floorBoltPattern():Array<{x:Float, y:Float}> {
		var r = floorBoltCircleDiameter / 2;
		return [for (i in 0...floorBoltCount) {
			var angle = 2 * Math.PI * i / floorBoltCount;
			{x: r * Math.cos(angle), y: r * Math.sin(angle)};
		}];
	}

	/** Screw that fits the pedestal's floor bolt circle. */
	public function floorMountScrewPart(length:Float):SocketHeadCapScrew
		return SocketHeadCapScrew.metric(floorMountScrew, length);

	override public function geometry(detail:ComponentDetail = Preview):Part {
		var base = Solids.cylinder(baseDiameter / 2, 0, baseThickness);
		var column = Solids.cylinder(columnDiameter / 2, 0, height);
		var body = Solids.union([base, column]);
		if (detail == Envelope) return body;
		var screw = floorMountScrewPart(10);
		var tools = [for (point in floorBoltPattern())
			Solids.cylinder(screw.clearanceDiameter(Medium) / 2, -0.1, baseThickness + 0.1, point.x, point.y)];
		var topCutDepth = flange.pilotHeight + 5;
		var topCut = flange.mountingCutout(topCutDepth);
		try {
			var placedTopCut = topCut.translated(new Vector(0, 0, height - topCutDepth));
			topCut.close();
			tools.push(placedTopCut);
		} catch (error:Dynamic) {
			topCut.close();
			throw error;
		}
		return Solids.cut(body, tools);
	}
}
