package machinekit.motion;

import haxe.Json;
import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import materia.assembly.AssemblyFrames;
import machinekit.component.Bom;
import machinekit.component.ComponentDetail;
import machinekit.component.ConnectorRole;
import machinekit.component.Dimension;
import machinekit.component.MachineComponent;
import machinekit.component.Solids;
import machinekit.standard.SocketHeadCapScrew;
import machinekit.component.ComponentRecipeSupport;
import machinekit.component.ComponentType;

/** One radial tapped set-screw hole in a shaft coupling. `z` is the axial location and `angle`
 * is measured in radians from local +X toward +Y. The thread flanks are not modelled. */
typedef ShaftCouplingSetScrew = {
	var z:Float;
	var angle:Float;
}

/** Rigid sleeve coupling joining two shaft ends, with an independent bore on each side (a
 * reducer coupling when they differ). Preview includes configurable radial tapped holes for
 * the semantic set screws; Envelope keeps the turned sleeve and bores only.
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
	public final setScrews:Array<ShaftCouplingSetScrew>;
	public final setScrewHoleDiameter:Float;

	public function new(boreA:Float, boreB:Float, ?outerDiameter:Float, ?length:Float,
			?setScrews:Array<ShaftCouplingSetScrew>) {
		if (!(boreA > 0) || !(boreB > 0)) throw "Shaft coupling needs positive bore diameters";
		var maxBore = Math.max(boreA, boreB);
		var wall = Math.max(3, maxBore * 0.4);
		var od = outerDiameter == null ? maxBore + 2 * wall : outerDiameter;
		var len = length == null ? maxBore * 3 : length;
		if (!(od > maxBore)) throw "Shaft coupling outer diameter must be larger than both bores";
		if (!(len > 0)) throw "Shaft coupling needs a positive length";
		var a = Dimension.format(boreA), b = Dimension.format(boreB), size = '${Dimension.format(od)}x${Dimension.format(len)}';
		super('COUPLING-${a}x$b-$size', 'Shaft coupling $a to $b mm, $size', "aluminium 6061");
		this.boreA = boreA;
		this.boreB = boreB;
		this.outerDiameter = od;
		this.length = len;
		setScrew = this.outerDiameter <= 20 ? "M3" : this.outerDiameter <= 30 ? "M4" : "M5";
		var screw = setScrewPart(10);
		setScrewHoleDiameter = screw.spec.tapDrill;
		var resolvedScrews = setScrews == null ? defaultSetScrews(len) : setScrews.copy();
		validateSetScrews(resolvedScrews);
		this.setScrews = resolvedScrews;
		addConnector("sideA", Face, Solids.axial(0, 0, 0));
		addConnector("axis", Axis, Solids.axial(0, 0, this.length / 2));
		addConnector("sideB", Face, Solids.axial(0, 0, this.length));
		var i = 1;
		for (hole in this.setScrews) {
			var x = this.outerDiameter / 2 * Math.cos(hole.angle), y = this.outerDiameter / 2 * Math.sin(hole.angle);
			addConnector('setScrew${i++}', Mount, AssemblyFrames.alongY(x, y, hole.z,
				Math.cos(hole.angle), Math.sin(hole.angle), 0));
		}
	}

	static var recipeTypeCache:Null<ComponentType>;

	public static function recipeType():ComponentType {
		if (recipeTypeCache == null) recipeTypeCache = new ComponentType("machinekit.motion.shaft-coupling", [ComponentRecipeSupport.length("boreA", 8), ComponentRecipeSupport.length("boreB", 8),
			ComponentRecipeSupport.length("outerDiameter", 20), ComponentRecipeSupport.length("length", 30), ComponentRecipeSupport.text("setScrews", '[{"z":7.5,"angle":0},{"z":22.5,"angle":0}]')],
			v -> new ShaftCoupling(v.number("boreA"), v.number("boreB"), v.number("outerDiameter"),
				v.number("length"), ShaftCoupling.setScrewsFromText(v.token("setScrews"))), true);
		return recipeTypeCache;
	}

	/** Subclasses must declare their own recipe and saved values. */
	override public function componentType():Null<ComponentType>
		return Std.isExactType(this, ShaftCoupling) ? recipeType() : null;

	override public function values():machinekit.component.ComponentValues return new machinekit.component.ComponentValues().setNumber("boreA", boreA).setNumber("boreB", boreB).setNumber("outerDiameter", outerDiameter).setNumber("length", length).setToken("setScrews", haxe.Json.stringify(setScrews)).setToken("material", materialSpec());

	override public function hasGeometry():Bool return true;

	override public function geometry(detail:ComponentDetail = Preview):Part {
		var body = Solids.named(Part.cylinderSpan(outerDiameter / 2, 0, length), "body");
		var boreATool = Solids.named(Part.cylinderSpan(boreA / 2, -0.1, length / 2 + 0.1), "bore.a");
		var boreBTool = Solids.named(Part.cylinderSpan(boreB / 2, length / 2 - 0.1, length + 0.1), "bore.b");
		if (detail == Envelope) return Solids.cut(body, [boreATool, boreBTool]);
		var tools = [boreATool, boreBTool];
		var screwRadius = setScrewHoleDiameter / 2;
		for (hole in setScrews) {
			var radial = new Vector(Math.cos(hole.angle), Math.sin(hole.angle), 0);
			var start = radial.scale(outerDiameter / 2 + 0.1).add(new Vector(0, 0, hole.z));
			tools.push(Solids.named(Part.cylinderAlong(screwRadius, start, radial.scale(-1),
				(outerDiameter - Math.min(boreA, boreB)) / 2 + 0.2), "setscrew" + (setScrews.indexOf(hole) + 1)));
		}
		return Solids.cut(body, tools);
	}

	/** Set screw that clamps the coupling to a shaft. */
	public function setScrewPart(length:Float):SocketHeadCapScrew
		return SocketHeadCapScrew.metric(setScrew, length);

	/** Coupling BOM, optionally including one semantic set screw for each radial hole. */
	public function billOfMaterials(screwLength:Float = 10):Bom {
		var result = new Bom();
		result.addComponent(this);
		if (setScrews.length > 0) result.addComponent(setScrewPart(screwLength), setScrews.length);
		return result;
	}

	static function defaultSetScrews(length:Float):Array<ShaftCouplingSetScrew>
		return [{z: length * 0.25, angle: 0}, {z: length * 0.75, angle: 0}];

	function validateSetScrews(holes:Array<ShaftCouplingSetScrew>):Void {
		for (i in 0...holes.length) {
			var hole = holes[i];
			var endMargin = setScrewPart(10).diameter * 1.5;
			if (hole == null || hole.z < endMargin || hole.z > length - endMargin || !Math.isFinite(hole.z) || !Math.isFinite(hole.angle))
				throw "Shaft coupling set screw location must clear both ends by 1.5 screw diameters";
			for (j in 0...i)
				if (Math.abs(holes[j].z - hole.z) < 1e-6 && Math.abs(holes[j].angle - hole.angle) < 1e-6)
					throw "Shaft coupling set screw locations must be unique";
		}
	}

	public static function setScrewsFromText(text:String):Array<ShaftCouplingSetScrew> {
		var rows:Array<Dynamic> = Json.parse(text);
		return [for (row in rows) {z: cast Reflect.field(row, "z"), angle: cast Reflect.field(row, "angle")}];
	}
}
