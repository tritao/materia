package machinekit.robotics;

import machinekit.component.ComponentType;
import machinekit.component.ComponentValues;
import machinekit.component.ComponentValue.*;
import machinekit.component.ComponentRecipeSupport;
import machinekit.component.Dimension;
import materia.project.MaterialLibrary;

import cadkit.modeling.Part;
import machinekit.component.ComponentDetail;
import machinekit.component.ConnectorRole;
import machinekit.component.Dimension;
import machinekit.component.MachineComponent;
import machinekit.component.Solids;
import machinekit.catalog.CatalogIndex;
import machinekit.catalog.FilteredCatalogIndex;
import machinekit.standard.ClearanceFit;
import machinekit.standard.SocketHeadCapScrew;

/** Adapter plate between a robot's `RobotFlange` and a tool: one face mates to the flange's
 * mounting pattern (a blind recess for its pilot boss, bolt and pin clearance holes), the other
 * exposes its own bolt circle for a gripper or tool.
 *
 * The default tool bolt circle lies outside the flange's bolt circle, far enough that tool and
 * flange screw heads clear each other; the plate grows past the flange diameter when needed to
 * carry it. A supplied tool bolt circle must keep every tool hole clear of the pilot recess, the
 * flange bolt and pin holes, and its neighbours.
 * CAD frame: robot-side face at z=0, tool-side face at z=thickness. Connectors: `robot` (the
 * flange mount, mate it to the flange's `face`), `tool` (the tool mount), and
 * `toolBolt1`..`toolBoltN`, all with +Y along +Z.
 */
class EndEffectorPlate extends MachineComponent {
	static inline var MIN_WEB:Float = 1.0;
	static inline var MIN_FLOOR:Float = 2.0;

	public final flange:RobotFlange;
	public final diameter:Float;
	public final thickness:Float;
	public final toolBoltCircleDiameter:Float;
	public final toolBoltCount:Int;
	public final toolMountScrew:String;

	public function new(flange:RobotFlange, ?thickness:Float, ?toolBoltCircleDiameter:Float,
			toolBoltCount:Int = 4, toolMountScrew:String = "M5") {
		var actualThickness:Float = thickness == null ? flange.thickness : thickness;
		if (!(actualThickness > flange.pilotRecessDepth() + MIN_FLOOR))
			throw "End effector plate must be thicker than the flange's pilot boss recess plus a 2 mm floor";
		if (toolBoltCount < 3) throw "End effector plate needs at least 3 tool bolts";
		var toolScrew = SocketHeadCapScrew.metric(toolMountScrew, 10);
		var flangeScrew = flange.mountScrewPart(10);
		var actualBoltCircle:Float = toolBoltCircleDiameter == null
			? flange.boltCircleDiameter + flangeScrew.spec.headDiameter + toolScrew.spec.headDiameter + 2 * MIN_WEB
			: toolBoltCircleDiameter;
		if (!(actualBoltCircle > 0)) throw "End effector plate needs a positive tool bolt circle";
		checkToolPattern(flange, actualBoltCircle, toolBoltCount, toolScrew.clearanceDiameter(Medium),
			flangeScrew.clearanceDiameter(Medium));
		var boltCircleText = Dimension.format(actualBoltCircle);
		super('EOAT-${Dimension.format(flange.boltCircleDiameter)}-${Dimension.format(actualThickness)}' +
			'-${toolBoltCount}x$toolMountScrew-PCD$boltCircleText',
			'End-effector adapter plate for ${flange.designation}, ${toolBoltCount}x$toolMountScrew tool bolts on ' +
			'$boltCircleText PCD', "aluminium 6061");
		this.flange = flange;
		this.thickness = actualThickness;
		this.toolBoltCircleDiameter = actualBoltCircle;
		this.toolBoltCount = toolBoltCount;
		this.toolMountScrew = toolMountScrew;
		diameter = Math.max(flange.flangeDiameter, actualBoltCircle + toolScrew.spec.headDiameter + 2 * MIN_WEB);
		addConnector("robot", Mount, Solids.axial(0, 0, 0));
		addConnector("tool", Mount, Solids.axial(0, 0, this.thickness));
		var i = 1;
		for (point in toolBoltPattern()) addConnector('toolBolt${i++}', Mount, Solids.axial(point.x, point.y, this.thickness));
	}

	/** Tool-side bolt centres, counter-clockwise from angle 0. */
	public function toolBoltPattern():Array<{x:Float, y:Float}>
		return circle(toolBoltCircleDiameter, toolBoltCount);

	override public function geometry(detail:ComponentDetail = Preview):Part {
		var ownedParts:Array<Part> = [];
		return Solids.building(ownedParts, tracked -> {
			var body = Part.cylinderSpan(diameter / 2, 0, thickness);
			tracked.push(body);
			if (detail == Envelope) return body;
			var toolScrew = SocketHeadCapScrew.metric(toolMountScrew, 10);
			var flangeTool = flange.mountingCutout(thickness);
			var tools = [flangeTool];
			tracked.push(flangeTool);
			for (point in toolBoltPattern()) {
				var tool = Part.cylinderSpan(toolScrew.clearanceDiameter(Medium) / 2, -0.1, thickness + 0.1, point.x, point.y);
				tracked.push(tool);
				tools.push(tool);
			}
			var result = Solids.cut(body, tools);
			tracked.push(result);
			return result;
		});
	}

	static function circle(diameter:Float, count:Int):Array<{x:Float, y:Float}> {
		var r = diameter / 2;
		return [for (i in 0...count) {
			var angle = 2 * Math.PI * i / count;
			{x: r * Math.cos(angle), y: r * Math.sin(angle)};
		}];
	}

	/** Throws unless every tool hole clears the pilot recess, the flange's bolt and pin holes (as
	 * `RobotFlange.mountingCutout` cuts them with its default clearances), and its neighbours.
	 */
	static function checkToolPattern(flange:RobotFlange, boltCircle:Float, count:Int, toolHole:Float,
			flangeHole:Float):Void {
		var pilotClearance = 0.2;
		var recessRadius = (flange.pilotDiameter + pilotClearance) / 2;
		var pinHole = flange.pinDiameter + pilotClearance;
		var pin = flange.pinPoint();
		var toolPoints = circle(boltCircle, count);
		function distance(a:{x:Float, y:Float}, b:{x:Float, y:Float}):Float
			return Math.sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
		if (!(boltCircle / 2 - toolHole / 2 >= recessRadius + MIN_WEB))
			throw "End effector tool bolt circle must clear the flange's pilot recess";
		if (!(distance(toolPoints[0], toolPoints[1]) >= toolHole + MIN_WEB))
			throw "End effector tool bolts are too close together";
		for (point in toolPoints) {
			if (!(distance(point, pin) >= (toolHole + pinHole) / 2 + MIN_WEB))
				throw "End effector tool bolt circle must clear the flange's pin hole";
			for (bolt in flange.boltPattern())
				if (!(distance(point, bolt) >= (toolHole + flangeHole) / 2 + MIN_WEB))
					throw "End effector tool bolt circle must clear the flange's bolt holes";
		}
	}

	private static var recipeTypeCache:Null<ComponentType>;
	private static var toolScrewCatalogCache:Null<CatalogIndex>;

	static function toolScrewCatalog():CatalogIndex {
		if (toolScrewCatalogCache == null) {
			var screws = SocketHeadCapScrew.catalog();
			var compatible:Array<String> = [];
			for (designation in screws.designations()) {
				try {
					new EndEffectorPlate(new RobotFlange(40, 4), 12, 65, 4, designation);
					compatible.push(designation);
				} catch (_:Dynamic) {}
			}
			toolScrewCatalogCache = new FilteredCatalogIndex(screws, compatible);
		}
		return toolScrewCatalogCache;
	}

	public static function recipeType():ComponentType {
		if (recipeTypeCache == null)
			recipeTypeCache = new ComponentType("machinekit.robotics.end-effector-plate",
			[ComponentRecipeSupport.length("flangePitchCircle", 40), ComponentRecipeSupport.count("flangeBoltCount", 4), ComponentRecipeSupport.length("thickness", 12),
				ComponentRecipeSupport.length("toolBoltCircleDiameter", 65), ComponentRecipeSupport.count("toolBoltCount", 4),
				ComponentRecipeSupport.catalog("toolMountScrew", toolScrewCatalog(), "M5")],
			v -> new EndEffectorPlate(new RobotFlange(v.number("flangePitchCircle"), v.integer("flangeBoltCount")),
				v.number("thickness"), v.number("toolBoltCircleDiameter"), v.integer("toolBoltCount"), v.token("toolMountScrew")));
		return recipeTypeCache;
	}

	override public function componentType():Null<ComponentType> return recipeType();

	override public function values():ComponentValues {
		return new ComponentValues().setNumber("flangePitchCircle", this.flange.spec.pitchCircle)
				.setInteger("flangeBoltCount", this.flange.boltCount)
				.setNumber("thickness", this.thickness)
				.setNumber("toolBoltCircleDiameter", this.toolBoltCircleDiameter)
				.setInteger("toolBoltCount", this.toolBoltCount)
				.setToken("toolMountScrew", this.toolMountScrew).setToken("material", materialSpec());
	}

}
