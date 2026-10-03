package machinekit.motion;

import machinekit.component.ComponentType;
import machinekit.component.ComponentValues;
import machinekit.component.ComponentValue.*;
import machinekit.component.ComponentRecipeSupport;
import machinekit.component.ToolSpec;
import machinekit.component.Dimension;
import materia.project.MaterialLibrary;

import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.catalog.Catalog;
import machinekit.catalog.CatalogMetadata.DimensionKind;
import machinekit.catalog.CatalogMetadata.Conformance;
import machinekit.component.ComponentDetail;
import machinekit.component.ConnectorRole;
import machinekit.component.Dimension;
import machinekit.component.MachineComponent;
import machinekit.component.Solids;
import materia.assembly.AssemblyDefinition.AssemblyActuator;
import machinekit.standard.ClearanceFit;
import machinekit.standard.SocketHeadCapScrew;

/** Stepper motor built from a NEMA mounting interface and a named motor variant.
 * CAD frame: mounting face at z=0, body toward -Z, shaft along +Z.
 */
class NemaStepper extends MachineComponent implements MotorDrive {
	static var frameTable:Null<Catalog<NemaFrameInterface>>;
	static var variantTable:Null<Catalog<StepperMotorVariant>>;
	static var ratingTable:Null<Catalog<StepperMotorRating>>;

	/** Frame mounting dimensions; kept as `spec` for existing callers. */
	public final spec:NemaFrameInterface;
	public final variant:StepperMotorVariant;
	public final bodyLength:Float;

	public static function catalog():Catalog<NemaFrameInterface> {
		if (frameTable == null)
			frameTable = new Catalog("NEMA frame", spec -> Std.string(spec.frame), [
				{frame: 17, face: 42.3, boltSpacing: 31.0, pilotDiameter: 22.0},
				{frame: 23, face: 56.4, boltSpacing: 47.14, pilotDiameter: 38.1},
				{frame: 34, face: 86.0, boltSpacing: 69.6, pilotDiameter: 73.0},
			], spec -> ({source: switch (spec.frame) {
				case 17 | 23: "https://www.nanotec.com/fileadmin/files/Katalog/linear-actuators-en.pdf";
				default: "https://www.nanotec.com/fileadmin/files/Baureihenuebersichten/Plug_Drive/Product_Overview_PD6-C.pdf";
			}, standard: null, standardEdition: null, dimensionKind: Mixed, conformance: NominalEnvelope,
				verifiedFields: ["face", "boltSpacing", "pilotDiameter"]}));
		return frameTable;
	}

	public static function variantCatalog():Catalog<StepperMotorVariant> {
		if (variantTable == null)
			variantTable = new Catalog("stepper motor variant", spec -> spec.designation, [
				{designation: "17HS19-1684S1", frame: 17, bodyFace: 42.0, bodyLength: 48.0,
					shaftDiameter: 5.0, shaftLength: 24.0, pilotHeight: 2.0,
					mountScrew: "M3", tappedMount: true, mountHoleDepth: 4.5},
				{designation: "23HS22-2804S", frame: 23, bodyFace: 57.3, bodyLength: 56.0,
					shaftDiameter: 6.35, shaftLength: 21.0, pilotHeight: 1.6,
					mountScrew: "M5", tappedMount: false, mountHoleDepth: 5.0},
				{designation: "34HS31-5504S", frame: 34, bodyFace: 86.0, bodyLength: 80.0,
					shaftDiameter: 14.0, shaftLength: 35.0, pilotHeight: 1.6,
					mountScrew: "M6", tappedMount: false, mountHoleDepth: 8.0},
			], spec -> ({source: switch (spec.designation) {
				case "17HS19-1684S1": "https://www.omc-stepperonline.com/nema-17-bipolar-1-8deg-45ncm-64oz-in-1-68a-2-8v-42x42x48mm-4-wires-17hs19-1684s1";
				case "23HS22-2804S": "https://www.omc-stepperonline.com/nema-23-bipolar-1-8deg-1-26nm-178-4oz-in-2-8a-2-5v-57x57x56mm-4-wires-23hs22-2804s";
				default: "https://www.omc-stepperonline.com/nema-34-cnc-stepper-motor-4-5nm-637-25oz-in-5-5a-86x86x80mm-key-way-shaft-34hs31-5504s";
			}, standard: null, standardEdition: null, dimensionKind: Unverified, conformance: NominalEnvelope,
				verifiedFields: ["bodyFace", "bodyLength", "shaftDiameter", "shaftLength"]}));
		return variantTable;
	}

	/**
	 * Ratings of the named variants. Holding torque and rated current are the ones the products are
	 * named by; inductance and rotor inertia are typical datasheet values, not checked against
	 * the linked pages.
	 */
	public static function ratingCatalog():Catalog<StepperMotorRating> {
		if (ratingTable == null)
			ratingTable = new Catalog("stepper motor rating", spec -> spec.designation, [
				{designation: "17HS19-1684S1", holdingTorque: 0.45, ratedCurrent: 1.68, phaseInductance: 2.8e-3,
					rotorInertia: 8.2e-6, stepAngle: 1.8},
				{designation: "23HS22-2804S", holdingTorque: 1.26, ratedCurrent: 2.8, phaseInductance: 2.5e-3,
					rotorInertia: 3.0e-5, stepAngle: 1.8},
				{designation: "34HS31-5504S", holdingTorque: 4.5, ratedCurrent: 5.5, phaseInductance: 3.2e-3,
					rotorInertia: 1.4e-4, stepAngle: 1.8},
			], spec -> ({source: variantCatalog().metadata(spec.designation).source, standard: null,
				standardEdition: null, dimensionKind: Unverified, conformance: NominalEnvelope,
				verifiedFields: ["holdingTorque", "ratedCurrent", "stepAngle"]}));
		return ratingTable;
	}

	/** Named manufacturer variant. */
	public static function model(designation:String):NemaStepper {
		var variant = variantCatalog().get(designation);
		return new NemaStepper(catalog().get(Std.string(variant.frame)), variant);
	}

	/** Build a motor from explicit frame and motor dimensions without a named catalog recipe. */
	public static function custom(spec:NemaFrameInterface, variant:StepperMotorVariant):NemaStepper
		return new NemaStepper(spec, variant, true);

	/** Default named variant for a frame. A length override creates a generic preview variant. */
	public static function frame(size:Int, ?bodyLength:Float):NemaStepper {
		var spec = catalog().get(Std.string(size));
		var name = switch (size) {
			case 17: "17HS19-1684S1";
			case 23: "23HS22-2804S";
			case 34: "34HS31-5504S";
			default: throw 'No default NEMA $size motor variant';
		};
		var base = variantCatalog().get(name);
		if (bodyLength == null) return new NemaStepper(spec, base);
		var custom:StepperMotorVariant = {
			designation: 'GENERIC-NEMA$size-L${Dimension.format(bodyLength)}', frame: size,
			bodyFace: base.bodyFace, bodyLength: bodyLength, shaftDiameter: base.shaftDiameter,
			shaftLength: base.shaftLength, pilotHeight: base.pilotHeight, mountScrew: base.mountScrew,
			tappedMount: base.tappedMount, mountHoleDepth: base.mountHoleDepth,
		};
		return new NemaStepper(spec, custom);
	}

	private function new(spec:NemaFrameInterface, variant:StepperMotorVariant, codeOnly:Bool = false) {
		if (spec.frame != variant.frame) throw "Stepper motor variant frame does not match its mounting interface";
		if (!(variant.bodyLength > variant.mountHoleDepth) || !(variant.bodyFace > spec.boltSpacing))
			throw 'NEMA ${spec.frame} motor body is too short or narrow';
		if (!(spec.boltSpacing > 0) || !(spec.face > spec.boltSpacing) || !(spec.pilotDiameter > variant.shaftDiameter)
			|| !(spec.pilotDiameter < spec.boltSpacing * Math.sqrt(2)) || !(variant.shaftDiameter > 0)
			|| !(variant.shaftLength > variant.pilotHeight) || !(variant.pilotHeight > 0))
			throw 'NEMA ${spec.frame} interface or motor variant has inconsistent dimensions';
		SocketHeadCapScrew.catalog().get(variant.mountScrew);
		var customName = '${variant.designation}-IF${Dimension.format(spec.face)}x${Dimension.format(spec.boltSpacing)}x${Dimension.format(spec.pilotDiameter)}' +
			'-B${Dimension.format(variant.bodyFace)}x${Dimension.format(variant.bodyLength)}-S${Dimension.format(variant.shaftDiameter)}x${Dimension.format(variant.shaftLength)}' +
			'-P${Dimension.format(variant.pilotHeight)}-M${variant.mountScrew}-${variant.tappedMount ? "T" : "C"}${Dimension.format(variant.mountHoleDepth)}';
		super(codeOnly ? customDesignation(customName) : variant.designation,
			'${variant.designation} NEMA ${spec.frame} stepper motor', null, codeOnly);
		this.spec = spec;
		this.variant = variant;
		bodyLength = variant.bodyLength;
		addConnector("mountFace", Mount, Solids.axial(0, 0, 0));
		addConnector("shaftAxis", Axis, Solids.axial(0, 0, 0));
		addConnector("shaftTip", Shaft, Solids.axial(0, 0, variant.shaftLength));
		var i = 1;
		for (point in boltPattern()) addConnector('bolt${i++}', Mount, Solids.axial(point.x, point.y, 0));
	}

	/** The motor's ratings, or null for a variant without them, such as a generic length. */
	public function rating():Null<StepperMotorRating>
		return ratingCatalog().exists(variant.designation) ? ratingCatalog().get(variant.designation) : null;

	/**
	 * Pull-out torque at shaft speed `speed` (rad/s) on a `volts` supply, in N m: the holding torque
	 * until the winding's inductance keeps its current below rated, then falling as 1 / speed. A
	 * first-order model: back EMF, the driver's current regulation and resonance are left out.
	 */
	public function pullOutTorque(speed:Float, volts:Float):Float {
		var rated = requireRating();
		var corner = cornerSpeed(rated, volts);
		var magnitude = Math.abs(speed);
		return magnitude <= corner ? rated.holdingTorque : rated.holdingTorque * corner / magnitude;
	}

	/**
	 * Torque and speed the motor can be relied on for on a `volts` supply: `margin` of its holding
	 * torque, up to the speed where its pull-out torque falls to that. Half is the usual margin
	 * that keeps a stepper from missing steps.
	 */
	public function usableTorque(margin:Float = 0.5):Float {
		if (!(margin > 0 && margin <= 1)) throw "Stepper torque margin must be in (0, 1]";
		return requireRating().holdingTorque * margin;
	}

	public function usableSpeed(volts:Float, margin:Float = 0.5):Float {
		usableTorque(margin);
		return cornerSpeed(requireRating(), volts) / margin;
	}

	/**
	 * The pull-out curve as torque-speed points (speed in rad/s, torque in N m, alternating): the
	 * holding torque to the corner speed, then falling as 1 / speed. Points are close enough to the
	 * hyperbola that joining them by lines overstates it by about 1% at most, and the curve ends at
	 * the usable speed or eight times the corner, whichever is higher.
	 */
	public function pullOutCurve(volts:Float, margin:Float = 0.5):Array<Float> {
		var rated = requireRating();
		var corner = cornerSpeed(rated, volts);
		var usable = 1.0 / margin;
		var last = Math.max(8.0, usable);
		var multiples = [1.0];
		for (multiple in [1.1, 1.25, 1.5, 1.75, 2.0, 2.5, 3.0, 4.0, 5.0, 6.0, 8.0, 10.0, 16.0, 32.0, 64.0])
			if (multiple <= last + 1e-9) multiples.push(multiple);
		// The usable speed is a point of its own, so the planner's number is exact on the curve.
		var present = false;
		for (multiple in multiples) if (Math.abs(multiple - usable) < 1e-9) present = true;
		if (!present && usable > 1.0) multiples.push(usable);
		multiples.sort((a, b) -> a < b ? -1 : a > b ? 1 : 0);
		var curve:Array<Float> = [0.0, rated.holdingTorque];
		for (multiple in multiples) {
			curve.push(multiple * corner);
			curve.push(rated.holdingTorque / multiple);
		}
		return curve;
	}

	public function actuator(id:String, joint:String, volts:Float, margin:Float, ?current:Float):AssemblyActuator {
		var rated = requireRating();
		var amps = current == null ? rated.ratedCurrent : current;
		if (!(amps > 0 && amps <= rated.ratedCurrent) || !Math.isFinite(amps))
			throw "Stepper driver current must be positive and at most the motor's rated current";
		var scale = amps / rated.ratedCurrent;
		// At lower current, holding torque scales with current and the reactance corner rises.
		// Reuse the rated-current curve at the equivalent voltage, then scale its torque points.
		var curve = pullOutCurve(volts / scale, margin);
		for (index in 0...Std.int(curve.length / 2)) curve[2 * index + 1] *= scale;
		var assumed = ["stepper inductance", "rotor inertia"];
		if (scale != 1) assumed.push("stepper current scaling");
		return {id: id, joint: joint, maxEffort: usableTorque(margin) * scale, maxRate: usableSpeed(volts / scale, margin),
			rotorInertia: rated.rotorInertia, fullStepsPerRevolution: 360.0 / rated.stepAngle, drive: "stepper",
			torqueSpeed: curve, holdingTorque: rated.holdingTorque * scale, assumed: assumed,
			assumptions: [{quantity: "inertia", label: "rotor inertia"},
				{quantity: "motor curve", label: "stepper inductance"},
				{quantity: "speed limit", label: "stepper inductance"}].concat(scale == 1 ? [] :
				[{quantity: "motor curve", label: "stepper current scaling"},
				 {quantity: "speed limit", label: "stepper current scaling"}])};
	}

	/** Shaft speed (rad/s) where the winding's reactance at rated current takes the whole supply. */
	static function cornerSpeed(rated:StepperMotorRating, volts:Float):Float {
		if (!(volts > 0) || !Math.isFinite(volts)) throw "Stepper supply voltage must be positive";
		// A two-phase hybrid stepper has 90 / step angle rotor teeth: its electrical frequency.
		var teeth = 90 / rated.stepAngle;
		return volts / (teeth * rated.phaseInductance * rated.ratedCurrent);
	}

	function requireRating():StepperMotorRating {
		var rated = rating();
		if (rated == null) throw '${variant.designation} has no torque rating';
		return rated;
	}

	/** Bolt centres on the mounting face, counter-clockwise from (+,+). */
	public function boltPattern():Array<{x:Float, y:Float}> {
		var h = spec.boltSpacing / 2;
		return [{x: h, y: h}, {x: -h, y: h}, {x: -h, y: -h}, {x: h, y: -h}];
	}

	override public function hasGeometry():Bool return true;

	override public function geometry(detail:ComponentDetail = Preview):Part {
		var ownedParts:Array<Part> = [];
		return Solids.building(ownedParts, tracked -> {
			var half = variant.bodyFace / 2;
			var parts:Array<Part> = [];
			if (detail == Envelope) {
				var body = Solids.named(Part.prism([new Vector(-half, -half), new Vector(half, -half),
					new Vector(half, half), new Vector(-half, half)], -bodyLength, 0), "body");
				tracked.push(body);
				parts.push(body);
			} else {
				var c = 0.08 * variant.bodyFace;
				var body = Part.prism([new Vector(-half + c, -half), new Vector(half - c, -half),
					new Vector(half, -half + c), new Vector(half, half - c), new Vector(half - c, half),
					new Vector(-half + c, half), new Vector(-half, half - c), new Vector(-half, -half + c)],
					-bodyLength, 0);
				body = Solids.named(body, "body");
				tracked.push(body);
				var screw = mountScrew(10);
				var holeDiameter = variant.tappedMount ? screw.spec.tapDrill : screw.clearanceDiameter(Medium);
				var tools:Array<Part> = [];
				var bolt = 1;
				for (point in boltPattern()) {
					// Bolt holes are named like the `bolt<k>` connectors: the pattern always has the same four.
					var tool = Solids.named(Part.cylinderSpan(holeDiameter / 2, -variant.mountHoleDepth, 0.1, point.x, point.y),
						"bolt" + bolt++);
					tracked.push(tool);
					tools.push(tool);
				}
				body = Solids.cut(body, tools);
				tracked.push(body);
				parts.push(body);
			}
			var pilot = Solids.named(Part.cylinderSpan(spec.pilotDiameter / 2, 0, variant.pilotHeight), "pilot");
			var shaft = Solids.named(Part.cylinderSpan(variant.shaftDiameter / 2, 0, variant.shaftLength), "shaft");
			tracked.push(pilot);
			tracked.push(shaft);
			parts.push(pilot);
			parts.push(shaft);
			var result = Solids.union(parts);
			tracked.push(result);
			return result;
		});
	}

	public function mountScrew(length:Float):SocketHeadCapScrew
		return SocketHeadCapScrew.metric(variant.mountScrew, length);

	/** Cutting tool for a plate in front of the face (z=0..thickness): pilot and bolt clearances. */
	public function mountingCutout(thickness:Float, pilotClearance:Float = 0.2, fit:ClearanceFit = Medium):Part {
		if (!(thickness > 0)) throw "Motor mounting plate needs a positive thickness";
		var radius = mountScrew(10).clearanceDiameter(fit) / 2;
		// Named like the motor's own bodies, so the faces a cutout leaves in a mating plate stay distinguishable.
		var tools = [Solids.named(Part.cylinderSpan((spec.pilotDiameter + pilotClearance) / 2, 0, thickness), "pilot")];
		var bolt = 1;
		for (point in boltPattern()) tools.push(Solids.named(Part.cylinderSpan(radius, 0, thickness, point.x, point.y), "bolt" + bolt++));
		return Solids.union(tools);
	}

	override public function toolSpecs():Array<ToolSpec> return [new ToolSpec("mountingCutout", [
		ComponentRecipeSupport.toolDepth(10), ComponentRecipeSupport.length("pilotClearance", 0.2),
		ComponentRecipeSupport.choice("fit", ["Fine", "Medium", "Coarse"], "Medium")])];

	override function buildTool(name:String, values:ComponentValues):Part {
		if (name == "mountingCutout") return mountingCutout(values.number("depth"),
			values.number("pilotClearance"), ComponentRecipeSupport.clearanceFit(values.token("fit")));
		return super.buildTool(name, values);
	}

	private static var namedRecipeTypeCache:Null<ComponentType>;

	public static function namedRecipeType():ComponentType {
		if (namedRecipeTypeCache == null)
			namedRecipeTypeCache = new ComponentType("machinekit.motion.nema-stepper",
			[ComponentRecipeSupport.catalog("model", NemaStepper.variantCatalog(), "17HS19-1684S1")],
			v -> NemaStepper.model(v.token("model")),
			true);
		return namedRecipeTypeCache;
	}

	private static var genericRecipeTypeCache:Null<ComponentType>;

	public static function genericRecipeType():ComponentType {
		if (genericRecipeTypeCache == null)
			genericRecipeTypeCache = new ComponentType("machinekit.motion.generic-nema-stepper",
			[ComponentRecipeSupport.choice("frame", ["17", "23", "34"], "17"), ComponentRecipeSupport.length("bodyLength", 48)],
			v -> NemaStepper.frame(Std.parseInt(v.token("frame")), v.number("bodyLength")));
		return genericRecipeTypeCache;
	}

	override public function componentType():Null<ComponentType>
		return codeOnly ? null : variant.designation.indexOf("GENERIC-NEMA") == 0 ? genericRecipeType() : namedRecipeType();

	override public function values():ComponentValues {
		if (variant.designation.indexOf("GENERIC-NEMA") == 0)
			return new ComponentValues().setToken("frame", Std.string(this.spec.frame))
				.setNumber("bodyLength", this.bodyLength).setToken("material", materialSpec());
		return new ComponentValues().setToken("model", this.variant.designation)
			.setToken("material", materialSpec());
	}

}
