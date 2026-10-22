package machinekit.motion;

import cadkit.InertiaTensor;
import cadkit.modeling.Align;
import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.assembly.RbkMotorMount;
import machinekit.component.SupplierCadComponent;
import machinekit.component.ComponentDetail;
import machinekit.component.ComponentRecipeSupport;
import machinekit.component.ComponentType;
import machinekit.component.ComponentValues;
import machinekit.component.MachineComponent;
import machinekit.component.Solids;
import machinekit.motion.VallderSupportData.ScrewSupportReference;
import materia.assembly.AssemblyFrames;

/**
 * A ball-screw end support from the BK/BF (foot-mounted) and FK/FF (flange-mounted) tables, with its bearing and
 * hardware: fixed (BK, FK, preloaded angular-contact pair) or floating (BF, FF). One recipe for every size, drawn
 * `nominal` (from the table) or `supplier` (the pinned Vallder CAD); the drawing changes neither the part number nor
 * the table's weight, which is its mass. Frame: the shaft runs along +Z through the origin; foot supports stand on
 * their base at y = -shaftHeight and are centred on z = 0; flange supports have their mounting flange from z = 0 to
 * its thickness.
 */
class ScrewSupportUnit extends MachineComponent {
	public static inline var NOMINAL = "nominal";
	public static inline var SUPPLIER = "supplier";

	public final spec:ScrewSupportReference;
	/** `NOMINAL` or `SUPPLIER`. */
	public final drawing:String;
	public final fixed:Bool;
	public final boreDiameter:Float;
	public final support:ScrewSupport;
	public final width:Float;
	public final height:Float;
	public final length:Float;

	public static function model(model:String, drawing:String = NOMINAL):ScrewSupportUnit
		return new ScrewSupportUnit(model, drawing);

	public static function bk12():ScrewSupportUnit return model("BK12");
	public static function bf12():ScrewSupportUnit return model("BF12");

	/**
	 * The foot-mounted support sized for a ball screw of nominal `screwDiameter`, by the usual pairing of screw and
	 * journal: SFU12 on BK10/BF10, SFU16 on BK12/BF12, SFU20 on BK15/BF15, SFU25 on BK20/BF20, SFU32 on BK25/BF25.
	 */
	public static function forScrew(screwDiameter:Float, fixed:Bool, drawing:String = NOMINAL):ScrewSupportUnit {
		var size = switch Math.round(screwDiameter) {
			case 12: "10";
			case 16: "12";
			case 20: "15";
			case 25: "20";
			case 32: "25";
			case _: throw 'No BK/BF support pairs with a ${screwDiameter} mm screw';
		};
		return model((fixed ? "BK" : "BF") + size, drawing);
	}

	function new(model:String, drawing:String) {
		if (VallderSupportData.models().indexOf(model) < 0) throw "Support needs interface review: " + model;
		if (drawing != NOMINAL && drawing != SUPPLIER) throw 'Unknown screw support drawing "$drawing"';
		var s = VallderSupportData.get(model);
		fixed = StringTools.startsWith(model, "BK") || StringTools.startsWith(model, "FK");
		super(model, '${fixed ? "Fixed" : "Floating"} screw support $model, ${machinekit.component.Dimension.format(s.bore)} mm journal, with bearing',
			"steel");
		spec = s;
		this.drawing = drawing;
		boreDiameter = s.bore;
		support = fixed ? Fixed : Simple;
		width = s.width;
		height = s.height;
		length = s.length;
		addConnector("axis", Axis, Solids.axial(0, 0, 0));
		addConnector("shaft", Axis, Solids.axial(0, 0, 0));
		for (datum in Vallder15SupportSeats.datums(model, s.sha256)) addConnector(datum.name, Axis, Solids.axial(0, 0, datum.z));
		if (s.base) {
			addConnector("base", Mount, AssemblyFrames.translation(0, -s.shaftHeight, 0));
			// The face it bolts down by: +Y (a connector's axis) down out of the foot.
			addConnector("mount", Mount, AssemblyFrames.alongY(0, -s.shaftHeight, 0, 0, -1, 0));
		} else {
			addConnector("flangeBack", Mount, Solids.axial(0, 0, 0));
			addConnector("flangeFront", Mount, Solids.axial(0, 0, s.height));
		}
		var index = 1;
		for (p in mountingPositions())
			addConnector('mount${index++}', Mount, s.base ? AssemblyFrames.alongY(p.x, -s.shaftHeight, p.z, 0, 1, 0) : Solids.axial(p.x, p.y, s.height));
		var box = envelopeBox();
		var dx = box.max.x - box.min.x, dy = box.max.y - box.min.y, dz = box.max.z - box.min.z, kg = s.massKg;
		declareMass(kg, new Vector((box.min.x + box.max.x) / 2, (box.min.y + box.max.y) / 2, (box.min.z + box.max.z) / 2),
			new InertiaTensor(kg * (dy * dy + dz * dz) / 12, 0, 0, kg * (dx * dx + dz * dz) / 12, 0, kg * (dx * dx + dy * dy) / 12));
	}

	/** Bolt axes: on the foot (x across, z along) or round the flange's pitch circle. */
	public function mountingPositions():Array<Vector> {
		var s = spec;
		if (s.base)
			return [for (x in [-s.pitch / 2, s.pitch / 2]) for (z in (s.longitudinalPitch == 0 ? [0.0] : [-s.longitudinalPitch / 2, s.longitudinalPitch / 2])) new Vector(x, 0, z)];
		var offset = s.pitch / Math.sqrt(8);
		return [for (x in [-offset, offset]) for (y in [-offset, offset]) new Vector(x, y, 0)];
	}

	/** Assumed catalogue-typical axial rigidity of the preloaded fixed pair, N/mm; floating supports carry no axial load. */
	public function axialStiffness():Null<Float> return fixed ? LeadScrew.SUPPORT_BEARING_STIFFNESS : null;

	override public function hasGeometry():Bool return true;

	override public function geometry(detail:ComponentDetail = Preview):Part {
		if (drawing == SUPPLIER) {
			var exterior = supplierCad();
			if (detail != Envelope) return exterior;
			var b = exterior.shape.bounds(), lo = b.get_min(), hi = b.get_max();
			exterior.close();
			return box(lo.get_x(), hi.get_x(), lo.get_y(), hi.get_y(), lo.get_z(), hi.get_z());
		}
		var e = envelopeBox();
		if (detail == Envelope) return box(e.min.x, e.max.x, e.min.y, e.max.y, e.min.z, e.max.z);
		return spec.base ? footDrawing() : flangeDrawing();
	}

	/** The table's envelope: the foot block, or the square flange and the boss behind it. */
	function envelopeBox():{min:Vector, max:Vector} {
		var s = spec;
		return s.base ? {min: new Vector(-s.width / 2, -s.shaftHeight, -s.length / 2), max: new Vector(s.width / 2, s.height - s.shaftHeight, s.length / 2)}
			: {min: new Vector(-s.width / 2, -s.width / 2, 0), max: new Vector(s.width / 2, s.width / 2, s.length)};
	}

	/** A foot support: a full-width foot up to the shaft, a narrower chamfered housing above, the bore, counterbored bolts. */
	function footDrawing():Part {
		var s = spec, foot = s.shaftHeight * 0.55, upper = s.width * 0.58, top = s.height - s.shaftHeight, c = upper * 0.12;
		return Solids.building([], owned -> {
			function keep(part:Part, tag:String):Part {
				var named = Solids.named(part, tag);
				owned.push(named);
				return named;
			}
			var body = Solids.union([
				keep(box(-s.width / 2, s.width / 2, -s.shaftHeight, -s.shaftHeight + foot, -s.length / 2, s.length / 2), "foot"),
				keep(Part.prism([new Vector(-upper / 2, -s.shaftHeight + foot - 0.01), new Vector(upper / 2, -s.shaftHeight + foot - 0.01),
					new Vector(upper / 2, top - c), new Vector(upper / 2 - c, top), new Vector(-upper / 2 + c, top), new Vector(-upper / 2, top - c)],
					-s.length / 2, s.length / 2), "housing")
			]);
			var tools = [keep(Part.cylinderSpan(s.bore / 2, -s.length / 2 - 0.1, s.length / 2 + 0.1), "bore")];
			for (p in mountingPositions()) {
				tools.push(keep(Part.cylinderAlongY(s.clearance / 2, -s.shaftHeight - 0.1, -s.shaftHeight + foot + 0.1, p.x, p.z), "bolt-hole"));
				tools.push(keep(Part.cylinderAlongY(Math.min(s.counterbore, s.length * 0.45) / 2, -s.shaftHeight + foot * 0.55, -s.shaftHeight + foot + 0.1,
					p.x, p.z), "counterbore"));
			}
			return Solids.cut(body, tools);
		});
	}

	/** A flange support: the square flange with chamfered corners and its bolt holes, a round boss behind, the bore. */
	function flangeDrawing():Part {
		var s = spec, half = s.width / 2, c = half * 0.25, boss = Math.min(s.width * 0.8, s.pitch - s.clearance) / 2;
		return Solids.building([], owned -> {
			function keep(part:Part, tag:String):Part {
				var named = Solids.named(part, tag);
				owned.push(named);
				return named;
			}
			var body = Solids.union([
				keep(Part.prism([new Vector(-half + c, -half), new Vector(half - c, -half), new Vector(half, -half + c), new Vector(half, half - c),
					new Vector(half - c, half), new Vector(-half + c, half), new Vector(-half, half - c), new Vector(-half, -half + c)], 0, s.height), "flange"),
				keep(Part.cylinderSpan(boss, s.height - 0.01, s.length), "boss")
			]);
			var tools = [keep(Part.cylinderSpan(s.bore / 2, -0.1, s.length + 0.1), "bore")];
			for (p in mountingPositions())
				tools.push(keep(Part.cylinderSpan(s.clearance / 2, -0.1, s.height + 0.1, p.x, p.y), "bolt-hole"));
			return Solids.cut(body, tools);
		});
	}

	/** The pinned Vallder CAD, checked against its digest. The caller owns it. */
	function supplierCad():Part {
		var imported = new SupplierCadComponent(RbkMotorMount.assetDirectory() + "/cad/assets/" + spec.sha256 + ".step", spec.sha256, designation,
			description);
		try {
			var result = imported.geometry();
			imported.close();
			return result;
		} catch (e:Dynamic) {
			imported.close();
			throw e;
		}
	}

	static function box(x0:Float, x1:Float, y0:Float, y1:Float, z0:Float, z1:Float):Part
		return Part.prism([new Vector(x0, y0), new Vector(x1, y0), new Vector(x1, y1), new Vector(x0, y1)], z0, z1);

	static var supportRecipe:Null<ComponentType>;

	public static function recipeType():ComponentType {
		if (supportRecipe == null) supportRecipe = new ComponentType("machinekit.motion.screw-support-unit",
			[ComponentRecipeSupport.choice("model", VallderSupportData.models(), "BK12"), ComponentRecipeSupport.choice("geometry", [NOMINAL, SUPPLIER], NOMINAL)],
			v -> new ScrewSupportUnit(v.token("model"), v.token("geometry")), true, false, ["geometry"]);
		return supportRecipe;
	}

	override public function componentType():Null<ComponentType> return recipeType();

	override public function values():ComponentValues return new ComponentValues()
		.setToken("model", spec.model).setToken("geometry", drawing).setToken("material", materialSpec());
}
