package machinekit.robotics;

import cadkit.modeling.Location;
import cadkit.modeling.Part;
import cadkit.modeling.Plane;
import cadkit.modeling.Vector;
import machinekit.component.ComponentDetail;
import machinekit.component.ConnectorRole;
import machinekit.component.Dimension;
import machinekit.component.MachineComponent;
import machinekit.component.Solids;
import machinekit.standard.ClearanceFit;
import machinekit.standard.SocketHeadCapScrew;
import materia.project.AssemblyFrames;

/** Column stand carrying a `RobotFlange` mount on top and its own floor bolt pattern at the
 * base, for mounting a robot arm above a machine bed.
 *
 * The column must be wider than the flange's bolt circle plus one screw head, so the top bolt
 * holes and heads stay on the column; it defaults to the flange diameter. The floor bolt circle
 * puts each floor screw head half a head diameter clear of the column, and the base extends half
 * a head diameter past the heads.
 *
 * CAD frame: floor face at z=0, top mounting face at z=height. Connectors: `floor` and
 * `floorBolt1`..`floorBoltN` at z=0 with +Y along +Z, and `top` (the flange mount) at z=height
 * with +Y along -Z, pointing into the pedestal like every flange `Mount` connector. Mating the
 * flange's `face` to `top` therefore turns the flange over: its plate sits above the pedestal and
 * its pilot boss drops into the top recess.
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
	/** Depth of the top mounting cutout: the pilot recess plus two screw diameters of thread. */
	public final topCutDepth:Float;

	public function new(flange:RobotFlange, height:Float, ?columnDiameter:Float, floorBoltCount:Int = 4) {
		var flangeScrew = flange.mountScrewPart(10);
		var cutDepth = flange.pilotRecessDepth() + 2 * flangeScrew.diameter;
		if (!(height > cutDepth)) throw "Pedestal height must clear the flange's pilot boss and bolt holes";
		if (floorBoltCount < 3) throw "Pedestal needs at least 3 floor bolts";
		var column:Float = columnDiameter == null ? flange.flangeDiameter : columnDiameter;
		if (!(column >= flange.boltCircleDiameter + flangeScrew.spec.headDiameter))
			throw "Pedestal column must be wider than the flange bolt circle plus a screw head";
		var floorScrew = column <= 60 ? "M8" : column <= 120 ? "M10" : "M12";
		var head = SocketHeadCapScrew.metric(floorScrew, 10).spec.headDiameter;
		super('PEDESTAL-${Dimension.format(flange.boltCircleDiameter)}-D${Dimension.format(column)}x${Dimension.format(height)}',
			'Pedestal for ${flange.designation}, ${Dimension.format(column)} mm column, ${Dimension.format(height)} mm tall',
			"steel");
		this.flange = flange;
		this.height = height;
		this.columnDiameter = column;
		topCutDepth = cutDepth;
		floorMountScrew = floorScrew;
		floorBoltCircleDiameter = column + 2 * head;
		baseDiameter = floorBoltCircleDiameter + 2 * head;
		baseThickness = Math.max(10, column * 0.2);
		this.floorBoltCount = floorBoltCount;
		addConnector("floor", Face, Solids.axial(0, 0, 0));
		addConnector("top", Mount, AssemblyFrames.alongY(0, 0, this.height, 0, 0, -1));
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
		// The cutout is built in the mated part's frame (face at z=0, material toward +Z); the
		// `top` connector turns it over (x kept, y and z reversed) onto the top face.
		var topCut = flange.mountingCutout(topCutDepth);
		try {
			var placedTopCut = topCut.placed(new Location(new Plane(new Vector(0, 0, height), Vector.X(), Vector.Z().scale(-1))));
			topCut.close();
			tools.push(placedTopCut);
		} catch (error:Dynamic) {
			topCut.close();
			for (tool in tools) tool.close();
			body.close();
			throw error;
		}
		return Solids.cut(body, tools);
	}
}
