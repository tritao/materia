package machinekit.robotics;

import cadkit.modeling.Part;
import machinekit.component.ComponentDetail;
import machinekit.component.ConnectorRole;
import machinekit.component.MachineComponent;
import machinekit.component.Solids;
import machinekit.standard.ClearanceFit;
import machinekit.standard.SocketHeadCapScrew;

/** Adapter plate between a robot's `RobotFlange` and a tool: one face mates to the flange's
 * mounting pattern, the other exposes its own bolt circle for a gripper or tool.
 * CAD frame: robot-side face at z=0, tool-side face at z=thickness. Connectors: `robot` (the
 * flange mount), `tool` (the tool mount), and `toolBolt1`..`toolBoltN`, all with +Y along +Z.
 */
class EndEffectorPlate extends MachineComponent {
	public final flange:RobotFlange;
	public final thickness:Float;
	public final toolBoltCircleDiameter:Float;
	public final toolBoltCount:Int;
	public final toolMountScrew:String;

	public function new(flange:RobotFlange, ?thickness:Float, ?toolBoltCircleDiameter:Float,
			toolBoltCount:Int = 4, toolMountScrew:String = "M5") {
		var actualThickness = thickness == null ? flange.thickness : thickness;
		var actualBoltCircle = toolBoltCircleDiameter == null ? flange.boltCircleDiameter * 0.6 : toolBoltCircleDiameter;
		if (!(actualThickness > flange.pilotHeight)) throw "End effector plate must be thicker than the flange's pilot boss";
		if (toolBoltCount < 3) throw "End effector plate needs at least 3 tool bolts";
		super('EOAT-${flange.flangeDiameter}-${actualThickness}', 'End-effector adapter plate for ${flange.designation}',
			"aluminium 6061");
		this.flange = flange;
		this.thickness = actualThickness;
		this.toolBoltCircleDiameter = actualBoltCircle;
		this.toolBoltCount = toolBoltCount;
		this.toolMountScrew = toolMountScrew;
		addConnector("robot", Mount, Solids.axial(0, 0, 0));
		addConnector("tool", Mount, Solids.axial(0, 0, this.thickness));
		var i = 1;
		for (point in toolBoltPattern()) addConnector('toolBolt${i++}', Mount, Solids.axial(point.x, point.y, this.thickness));
	}

	/** Tool-side bolt centres, counter-clockwise from angle 0. */
	public function toolBoltPattern():Array<{x:Float, y:Float}> {
		var r = toolBoltCircleDiameter / 2;
		return [for (i in 0...toolBoltCount) {
			var angle = 2 * Math.PI * i / toolBoltCount;
			{x: r * Math.cos(angle), y: r * Math.sin(angle)};
		}];
	}

	override public function geometry(detail:ComponentDetail = Preview):Part {
		var body = Solids.cylinder(flange.flangeDiameter / 2, 0, thickness);
		if (detail == Envelope) return body;
		var toolScrew = SocketHeadCapScrew.metric(toolMountScrew, 10);
		var tools = [flange.mountingCutout(thickness)];
		for (point in toolBoltPattern())
			tools.push(Solids.cylinder(toolScrew.clearanceDiameter(Medium) / 2, -0.1, thickness + 0.1, point.x, point.y));
		return Solids.cut(body, tools);
	}
}
