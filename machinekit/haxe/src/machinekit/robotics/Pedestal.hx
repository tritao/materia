package machinekit.robotics;

import machinekit.component.ComponentType;
import machinekit.component.ComponentValues;
import machinekit.component.ComponentValue.*;
import machinekit.component.ComponentRecipeSupport;
import machinekit.component.Dimension;
import materia.project.MaterialLibrary;

import cadkit.modeling.Location;
import cadkit.modeling.Axis;
import cadkit.modeling.Part;
import cadkit.modeling.Plane;
import cadkit.modeling.Sketch;
import cadkit.modeling.Vector;
import machinekit.component.Bom;
import machinekit.component.ComponentDetail;
import machinekit.component.ConnectorRole;
import machinekit.component.Dimension;
import machinekit.component.MachineComponent;
import machinekit.component.Solids;
import machinekit.standard.ClearanceFit;
import machinekit.standard.SocketHeadCapScrew;
import materia.assembly.AssemblyFrames;

/** Optional structural and service details for a pedestal. Dimensions are in millimetres. */
typedef PedestalDetail = {
	var ?baseThickness:Float;
	var ?anchorCircleDiameter:Float;
	var ?gussetHeight:Float;
	var ?gussetThickness:Float;
	var ?gussetCount:Int;
	var ?levelingFootDiameter:Float;
	var ?levelingFootHeight:Float;
	var ?cablePathDiameter:Float;
}

/** Column stand carrying a `RobotFlange` mount on top and its own floor bolt pattern at the
 * base, for mounting a robot arm above a machine bed.
 *
 * The column must be wider than the flange's bolt circle plus one screw head, so the top bolt
 * holes and heads stay on the column; it defaults to the flange diameter. The floor bolt circle
 * puts each floor screw head half a head diameter clear of the column, and the base extends half
 * a head diameter past the heads.
 *
 * CAD frame: floor face at z=0, top mounting face at z=height. Connectors: `floor` and
 * `floorBolt1`/`anchor1` through `floorBoltN`/`anchorN` at the floor plane with +Y along +Z,
 * and `top` (the flange mount) at z=height with +Y along -Z, pointing into the pedestal like
 * every flange `Mount` connector. Mating the flange's `face` to `top` therefore turns the flange
 * over: its plate sits above the pedestal and its pilot boss drops into the top recess.
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
	public final anchorHoleDiameter:Float;
	public final gussetHeight:Float;
	public final gussetThickness:Float;
	public final gussetCount:Int;
	public final levelingFootDiameter:Float;
	public final levelingFootHeight:Float;
	public final cablePathDiameter:Float;
	/** Depth of the top mounting cutout: the pilot recess plus two screw diameters of thread. */
	public final topCutDepth:Float;

	public function new(flange:RobotFlange, height:Float, ?columnDiameter:Float, floorBoltCount:Int = 4,
			?detail:PedestalDetail) {
		var flangeScrew = flange.mountScrewPart(10);
		var cutDepth = flange.pilotRecessDepth() + 2 * flangeScrew.diameter;
		if (!(height > cutDepth)) throw "Pedestal height must clear the flange's pilot boss and bolt holes";
		if (floorBoltCount < 3) throw "Pedestal needs at least 3 floor bolts";
		var column:Float = columnDiameter == null ? flange.flangeDiameter : columnDiameter;
		if (!(column >= flange.boltCircleDiameter + flangeScrew.spec.headDiameter))
			throw "Pedestal column must be wider than the flange bolt circle plus a screw head";
		var floorScrew = column <= 60 ? "M8" : column <= 120 ? "M10" : "M12";
		var floorPart = SocketHeadCapScrew.metric(floorScrew, 10);
		var head = floorPart.spec.headDiameter;
		var resolvedDetail:PedestalDetail = detail == null ? emptyDetail() : detail;
		var actualBaseThickness = resolvedDetail.baseThickness == null ? Math.max(10, column * 0.2) : resolvedDetail.baseThickness;
		var actualAnchorCircle = resolvedDetail.anchorCircleDiameter == null ? column + 2 * head : resolvedDetail.anchorCircleDiameter;
		var actualGussetHeight = resolvedDetail.gussetHeight == null ? 0 : resolvedDetail.gussetHeight;
		var actualGussetThickness = resolvedDetail.gussetThickness == null ? Math.max(3, column * 0.08) : resolvedDetail.gussetThickness;
		var actualGussetCount = resolvedDetail.gussetCount == null ? 4 : resolvedDetail.gussetCount;
		var actualFootDiameter = resolvedDetail.levelingFootDiameter == null ? 0 : resolvedDetail.levelingFootDiameter;
		var actualFootHeight = resolvedDetail.levelingFootHeight == null ? Math.max(4, actualFootDiameter * 0.2) : resolvedDetail.levelingFootHeight;
		var actualCablePath = resolvedDetail.cablePathDiameter == null ? 0 : resolvedDetail.cablePathDiameter;
		if (!Math.isFinite(actualBaseThickness) || !(actualBaseThickness > 0) || actualBaseThickness >= height)
			throw "Pedestal base thickness must be positive and below its height";
		if (!Math.isFinite(actualAnchorCircle) || !(actualAnchorCircle >= column + head + 2))
			throw "Pedestal anchor circle must clear the column and screw heads";
		if (!Math.isFinite(actualGussetHeight) || actualGussetHeight < 0 || actualGussetHeight >= height)
			throw "Pedestal gusset height must be below the pedestal height";
		if (!Math.isFinite(actualGussetThickness) || !Math.isFinite(actualGussetCount) ||
			(actualGussetHeight > 0 && (!(actualGussetThickness > 0) || actualGussetThickness >= column || actualGussetCount < 3)))
			throw "Pedestal gussets need a positive thickness, a valid count, and room around the column";
		if (!Math.isFinite(actualFootDiameter) || !Math.isFinite(actualFootHeight) || actualFootDiameter < 0 ||
			(actualFootDiameter > 0 && !(actualFootHeight > 0)))
			throw "Pedestal leveling feet need positive dimensions";
		if (!Math.isFinite(actualCablePath) || actualCablePath < 0 || actualCablePath > flange.pilotDiameter)
			throw "Pedestal cable path must not exceed the flange pilot recess diameter";
		var detailSuffix = detailDesignation({baseThickness: actualBaseThickness,
			anchorCircleDiameter: actualAnchorCircle, gussetHeight: actualGussetHeight,
			gussetThickness: actualGussetThickness, gussetCount: actualGussetCount,
			levelingFootDiameter: actualFootDiameter,
			levelingFootHeight: actualFootDiameter > 0 ? actualFootHeight : 0,
			cablePathDiameter: actualCablePath});
		var hasDetails = actualBaseThickness != Math.max(10, column * 0.2) ||
			actualAnchorCircle != column + 2 * head || actualGussetHeight > 0 ||
			actualFootDiameter > 0 || actualCablePath > 0;
		super('PEDESTAL-${Dimension.format(flange.boltCircleDiameter)}-D${Dimension.format(column)}x${Dimension.format(height)}$detailSuffix',
			'Pedestal for ${flange.designation}, ${Dimension.format(column)} mm column, ${Dimension.format(height)} mm tall' +
			(hasDetails ? ", detailed base and services" : ""),
			"steel");
		this.flange = flange;
		this.height = height;
		this.columnDiameter = column;
		topCutDepth = cutDepth;
		floorMountScrew = floorScrew;
		anchorHoleDiameter = floorPart.clearanceDiameter(Medium);
		floorBoltCircleDiameter = actualAnchorCircle;
		baseDiameter = floorBoltCircleDiameter + 2 * head;
		baseThickness = actualBaseThickness;
		this.floorBoltCount = floorBoltCount;
		gussetHeight = actualGussetHeight;
		gussetThickness = actualGussetThickness;
		gussetCount = actualGussetCount;
		levelingFootDiameter = actualFootDiameter;
		levelingFootHeight = actualFootDiameter > 0 ? actualFootHeight : 0;
		cablePathDiameter = actualCablePath;
		var floorZ = -levelingFootHeight;
		addConnector("floor", Face, Solids.axial(0, 0, floorZ));
		// Match the flange's locating-pin datum while facing its boss downward.
		addConnector("top", Mount, AssemblyFrames.compose(
			AssemblyFrames.alongY(0, 0, this.height, 0, 0, -1),
			AssemblyFrames.turnY(Math.PI / flange.boltCount)));
		var i = 1;
		for (point in floorBoltPattern()) {
			var frame = Solids.axial(point.x, point.y, floorZ);
			var index = i++;
			addConnector('floorBolt$index', Mount, frame);
			addConnector('anchor$index', Mount, frame);
		}
		if (cablePathDiameter > 0) addConnector("cablePath", Axis, Solids.axial(0, 0, height / 2));
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

	/** Pedestal BOM, including one semantic floor anchor screw for each anchor hole. */
	public function billOfMaterials(floorScrewLength:Float = 40):Bom {
		var result = new Bom();
		result.addComponent(this);
		result.addComponent(floorMountScrewPart(floorScrewLength), floorBoltCount);
		return result;
	}

	override public function hasGeometry():Bool return true;

	override public function geometry(detail:ComponentDetail = Preview):Part {
		var ownedParts:Array<Part> = [];
		return Solids.building(ownedParts, tracked -> {
			var parts:Array<Part> = [];
			var base = Part.cylinderSpan(baseDiameter / 2, 0, baseThickness);
			tracked.push(base);
			var column = Part.cylinderSpan(columnDiameter / 2, 0, height);
			tracked.push(column);
			parts.push(base);
			parts.push(column);
			if (levelingFootDiameter > 0)
				for (point in floorBoltPattern()) {
					var foot = Part.cylinderSpan(levelingFootDiameter / 2, -levelingFootHeight, 0, point.x, point.y);
					tracked.push(foot);
					parts.push(foot);
				}
			if (gussetHeight > 0)
				for (i in 0...gussetCount) {
					var support = gusset(2 * Math.PI * (i + 0.5) / gussetCount);
					tracked.push(support);
					parts.push(support);
				}
			var body = Solids.union(parts);
			tracked.push(body);
			if (detail == Envelope) return body;
			var screw = floorMountScrewPart(10);
			var anchorStart = -levelingFootHeight - 0.1;
			var tools:Array<Part> = [];
			for (point in floorBoltPattern()) {
				var tool = Part.cylinderSpan(screw.clearanceDiameter(Medium) / 2, anchorStart, baseThickness + 0.1, point.x, point.y);
				tracked.push(tool);
				tools.push(tool);
			}
			if (cablePathDiameter > 0) {
				var cable = Part.cylinderSpan(cablePathDiameter / 2, -0.1, height + 0.1);
				tracked.push(cable);
				tools.push(cable);
			}
			// The cutout is built in the mated part's frame (face at z=0, material toward +Z); the
			// `top` connector turns it over (x kept, y and z reversed) onto the top face.
			var topCut = flange.mountingCutout(topCutDepth);
			tracked.push(topCut);
			var placedTopCut = topCut.placed(new Location(new Plane(new Vector(0, 0, height), Vector.X(), Vector.Z().scale(-1))));
			topCut.close();
			tracked.push(placedTopCut);
			tools.push(placedTopCut);
			var result = Solids.cut(body, tools);
			tracked.push(result);
			return result;
		});
	}

	function gusset(angle:Float):Part {
		var root = columnDiameter / 2 - 0.1, outer = baseDiameter / 2 - 0.5;
		var plane = Plane.XZ().offset(-gussetThickness / 2);
		var sketch = Sketch.polygon([new Vector(root, 0), new Vector(outer, 0), new Vector(root, gussetHeight)], plane);
		var local:Part;
		try {
			local = sketch.extrude(gussetThickness);
		} catch (error:Dynamic) {
			sketch.close();
			throw error;
		}
		sketch.close();
		try {
			var result = local.placed(Location.rotation(Axis.Z(), angle));
			local.close();
			return result;
		} catch (error:Dynamic) {
			local.close();
			throw error;
		}
	}

	static function emptyDetail():PedestalDetail
		return {baseThickness: null, anchorCircleDiameter: null, gussetHeight: null, gussetThickness: null,
			gussetCount: null, levelingFootDiameter: null, levelingFootHeight: null, cablePathDiameter: null};

	static function detailDesignation(detail:PedestalDetail):String {
		return '-B${Dimension.format(optionalFloat(detail.baseThickness))}-A${Dimension.format(optionalFloat(detail.anchorCircleDiameter))}' +
			'-G${Dimension.format(optionalFloat(detail.gussetHeight))}x${Dimension.format(optionalFloat(detail.gussetThickness))}x${detail.gussetCount == null ? 0 : detail.gussetCount}' +
			'-F${Dimension.format(optionalFloat(detail.levelingFootDiameter))}x${Dimension.format(optionalFloat(detail.levelingFootHeight))}' +
			'-C${Dimension.format(optionalFloat(detail.cablePathDiameter))}';
	}

	static function optionalFloat(value:Null<Float>):Float return value == null ? 0 : value;

	private static var recipeTypeCache:Null<ComponentType>;

	public static function recipeType():ComponentType {
		if (recipeTypeCache == null)
			recipeTypeCache = new ComponentType("machinekit.robotics.pedestal",
			[ComponentRecipeSupport.length("flangePitchCircle", 40), ComponentRecipeSupport.count("flangeBoltCount", 4), ComponentRecipeSupport.length("height", 150),
				ComponentRecipeSupport.length("columnDiameter", 60), ComponentRecipeSupport.count("floorBoltCount", 4), ComponentRecipeSupport.length("baseThickness", 12),
				ComponentRecipeSupport.length("anchorCircleDiameter", 90), ComponentRecipeSupport.length("gussetHeight", 0), ComponentRecipeSupport.length("gussetThickness", 4),
				ComponentRecipeSupport.count("gussetCount", 4), ComponentRecipeSupport.length("levelingFootDiameter", 0),
				ComponentRecipeSupport.length("levelingFootHeight", 4), ComponentRecipeSupport.length("cablePathDiameter", 0)],
			v -> new Pedestal(new RobotFlange(v.number("flangePitchCircle"), v.integer("flangeBoltCount")),
				v.number("height"), v.number("columnDiameter"), v.integer("floorBoltCount"),
				{baseThickness: v.number("baseThickness"), anchorCircleDiameter: v.number("anchorCircleDiameter"),
				gussetHeight: v.number("gussetHeight"), gussetThickness: v.number("gussetThickness"),
				gussetCount: v.integer("gussetCount"), levelingFootDiameter: v.number("levelingFootDiameter"),
				levelingFootHeight: v.number("levelingFootHeight"), cablePathDiameter: v.number("cablePathDiameter")}));
		return recipeTypeCache;
	}

	override public function componentType():Null<ComponentType> return recipeType();

	override public function values():ComponentValues {
		return new ComponentValues().setNumber("flangePitchCircle", this.flange.spec.pitchCircle)
				.setInteger("flangeBoltCount", this.flange.boltCount)
				.setNumber("height", this.height)
				.setNumber("columnDiameter", this.columnDiameter)
				.setInteger("floorBoltCount", this.floorBoltCount)
				.setNumber("baseThickness", this.baseThickness)
				.setNumber("anchorCircleDiameter", this.floorBoltCircleDiameter)
				.setNumber("gussetHeight", this.gussetHeight)
				.setNumber("gussetThickness", this.gussetThickness)
				.setInteger("gussetCount", this.gussetCount)
				.setNumber("levelingFootDiameter", this.levelingFootDiameter)
				.setNumber("levelingFootHeight", this.levelingFootHeight)
				.setNumber("cablePathDiameter", this.cablePathDiameter).setToken("material", materialSpec());
	}

}
