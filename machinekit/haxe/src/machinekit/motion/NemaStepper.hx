package machinekit.motion;

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
import machinekit.standard.ClearanceFit;
import machinekit.standard.SocketHeadCapScrew;

/** Stepper motor built from a NEMA mounting interface and a named motor variant.
 * CAD frame: mounting face at z=0, body toward -Z, shaft along +Z.
 */
class NemaStepper extends MachineComponent {
	static var frameTable:Null<Catalog<NemaFrameInterface>>;
	static var variantTable:Null<Catalog<StepperMotorVariant>>;

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

	/** Named manufacturer variant. */
	public static function model(designation:String):NemaStepper {
		var variant = variantCatalog().get(designation);
		return new NemaStepper(catalog().get(Std.string(variant.frame)), variant);
	}

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

	public function new(spec:NemaFrameInterface, variant:StepperMotorVariant) {
		if (spec.frame != variant.frame) throw "Stepper motor variant frame does not match its mounting interface";
		if (!(variant.bodyLength > variant.mountHoleDepth) || !(variant.bodyFace > spec.boltSpacing))
			throw 'NEMA ${spec.frame} motor body is too short or narrow';
		if (!(spec.boltSpacing > 0) || !(spec.face > spec.boltSpacing) || !(spec.pilotDiameter > variant.shaftDiameter)
			|| !(spec.pilotDiameter < spec.boltSpacing * Math.sqrt(2)) || !(variant.shaftDiameter > 0)
			|| !(variant.shaftLength > variant.pilotHeight) || !(variant.pilotHeight > 0))
			throw 'NEMA ${spec.frame} interface or motor variant has inconsistent dimensions';
		SocketHeadCapScrew.catalog().get(variant.mountScrew);
		super(variant.designation, '${variant.designation} NEMA ${spec.frame} stepper motor', null);
		this.spec = spec;
		this.variant = variant;
		bodyLength = variant.bodyLength;
		addConnector("mountFace", Mount, Solids.axial(0, 0, 0));
		addConnector("shaftAxis", Axis, Solids.axial(0, 0, 0));
		addConnector("shaftTip", Shaft, Solids.axial(0, 0, variant.shaftLength));
		var i = 1;
		for (point in boltPattern()) addConnector('bolt${i++}', Mount, Solids.axial(point.x, point.y, 0));
	}

	/** Bolt centres on the mounting face, counter-clockwise from (+,+). */
	public function boltPattern():Array<{x:Float, y:Float}> {
		var h = spec.boltSpacing / 2;
		return [{x: h, y: h}, {x: -h, y: h}, {x: -h, y: -h}, {x: h, y: -h}];
	}

	override public function geometry(detail:ComponentDetail = Preview):Part {
		var half = variant.bodyFace / 2;
		var parts:Array<Part> = [];
		if (detail == Envelope) {
			parts.push(Solids.prism([new Vector(-half, -half), new Vector(half, -half),
				new Vector(half, half), new Vector(-half, half)], -bodyLength, 0));
		} else {
			var c = 0.08 * variant.bodyFace;
			var body = Solids.prism([new Vector(-half + c, -half), new Vector(half - c, -half),
				new Vector(half, -half + c), new Vector(half, half - c), new Vector(half - c, half),
				new Vector(-half + c, half), new Vector(-half, half - c), new Vector(-half, -half + c)],
				-bodyLength, 0);
			var screw = mountScrew(10);
			var holeDiameter = variant.tappedMount ? screw.spec.tapDrill : screw.clearanceDiameter(Medium);
			parts.push(Solids.cut(body, [for (point in boltPattern())
				Solids.cylinder(holeDiameter / 2, -variant.mountHoleDepth, 0.1, point.x, point.y)]));
		}
		parts.push(Solids.cylinder(spec.pilotDiameter / 2, 0, variant.pilotHeight));
		parts.push(Solids.cylinder(variant.shaftDiameter / 2, 0, variant.shaftLength));
		return Solids.union(parts);
	}

	public function mountScrew(length:Float):SocketHeadCapScrew
		return SocketHeadCapScrew.metric(variant.mountScrew, length);

	/** Cutting tool for a plate in front of the face (z=0..thickness): pilot and bolt clearances. */
	public function mountingCutout(thickness:Float, pilotClearance:Float = 0.2, fit:ClearanceFit = Medium):Part {
		if (!(thickness > 0)) throw "Motor mounting plate needs a positive thickness";
		var radius = mountScrew(10).clearanceDiameter(fit) / 2;
		var tools = [Solids.cylinder((spec.pilotDiameter + pilotClearance) / 2, 0, thickness)];
		for (point in boltPattern()) tools.push(Solids.cylinder(radius, 0, thickness, point.x, point.y));
		return Solids.union(tools);
	}
}
