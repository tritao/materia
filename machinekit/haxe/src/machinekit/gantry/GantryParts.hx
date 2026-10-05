package machinekit.gantry;

import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import cadkit.modeling.Location;
import cadkit.modeling.Plane;
import machinekit.component.MachineComponent;
import machinekit.component.ComponentDetail;
import machinekit.component.Dimension;
import machinekit.component.Solids;
import machinekit.structural.TSlotExtrusion;
import machinekit.motion.NemaStepper;
import machinekit.motion.LeadScrewNut;
import machinekit.standard.ClearanceFit;
import materia.assembly.AssemblyFrames;
import materia.assembly.AssemblyRecord.AssemblyFrame;

/** Cut extrusion, centred in its cross-section and extending along local +Z. */
class GantryExtrusion extends MachineComponent {
	public final profile:TSlotExtrusion;
	public final length:Float;
	public function new(profile:TSlotExtrusion, length:Float) {
		if (!Math.isFinite(length) || length <= 0) throw "Gantry extrusion length must be positive";
		super(profile.designation + "-L" + Dimension.format(length), profile.description + ", cut length " + Dimension.format(length),
			"aluminium 6061", true);
		this.profile = profile; this.length = length;
	}
	override public function hasGeometry():Bool return true;
	override public function geometry(detail:ComponentDetail = Preview):Part {
		if (detail == Envelope) return Part.box(profile.size, profile.height, length);
		return profile.geometry(length);
	}
}

/** Mounting plate, centred in X/Y with its bottom at Z=0. */
class GantryPlate extends MachineComponent {
	public final width:Float;
	public final depth:Float;
	public final height:Float;
	public final motor:Null<NemaStepper>;
	public final motorFace:Null<AssemblyFrame>;
	public function new(name:String, width:Float, depth:Float, height:Float,
			?motor:NemaStepper, ?motorFace:AssemblyFrame) {
		for (value in [width, depth, height])
			if (!Math.isFinite(value) || value <= 0) throw "Gantry plate dimensions must be positive";
		if ((motor == null) != (motorFace == null)) throw "A motor mounting plate needs its motor face";
		var mountIdentity = motor == null || motorFace == null ? "" : "-" + motor.designation + "-AT" +
			[motorFace.x, motorFace.y, motorFace.z, motorFace.qx, motorFace.qy, motorFace.qz, motorFace.qw].join(",");
		super("GANTRY-" + name + "-" + Dimension.format(width) + "x" + Dimension.format(depth) + "x" + Dimension.format(height) + mountIdentity,
			"Gantry " + name, "aluminium 6061", true);
		this.width = width; this.depth = depth; this.height = height;
		this.motor = motor; this.motorFace = motorFace;
	}
	override public function hasGeometry():Bool return true;
	override public function geometry(detail:ComponentDetail = Preview):Part {
		var mounted = motor, face = motorFace;
		if (mounted == null || face == null || detail == Envelope) return Part.box(width, depth, height);
		return Solids.building([], owned -> {
			var block = Part.box(width, depth, height);
			owned.push(block);
			var cutout = mounted.mountingCutout(width + depth + height);
			owned.push(cutout);
			var x = AssemblyFrames.transformVector(face, 1, 0, 0), z = AssemblyFrames.transformVector(face, 0, 0, 1);
			var tool = cutout.placed(new Location(new Plane(new Vector(face.x, face.y, face.z),
				new Vector(x.x, x.y, x.z), new Vector(z.x, z.y, z.z))));
			owned.push(tool); cutout.close();
			return Solids.cut(block, [tool]);
		});
	}
}

/** Stationary journal supporting a belt idler's bore. */
class GantryIdlerPin extends MachineComponent {
	public final diameter:Float;
	public final length:Float;
	public function new(diameter:Float, length:Float) {
		if (!Math.isFinite(diameter) || !Math.isFinite(length) || diameter <= 0 || length <= 0) throw "Idler pin dimensions must be positive";
		super("GANTRY-IDLER-PIN-D" + Dimension.format(diameter) + "-L" + Dimension.format(length),
			"Gantry idler pin", "steel", true);
		this.diameter = diameter; this.length = length;
	}
	override public function hasGeometry():Bool return true;
	override public function geometry(detail:ComponentDetail = Preview):Part
		return Part.cylinderSpan(diameter / 2, -6, length);
}

/** Belt jaw with a through slot around the belt's actual width and band thickness. */
class GantryBeltClamp extends MachineComponent {
	public final beltWidth:Float;
	public final bandThickness:Float;
	public function new(beltWidth:Float, bandThickness:Float) {
		for (value in [beltWidth, bandThickness])
			if (!Math.isFinite(value) || value <= 0) throw "Belt clamp dimensions must be positive";
		super("GANTRY-BELT-CLAMP-W" + Dimension.format(beltWidth) + "-T" + Dimension.format(bandThickness), "Gantry belt clamp", "aluminium 6061", true);
		this.beltWidth = beltWidth; this.bandThickness = bandThickness;
		// The path is defined on the belt's lower face; the jaw is centred on its width.
		addConnector("belt", machinekit.component.ConnectorRole.Mount, AssemblyFrames.translation(0, 0, -beltWidth / 2));
	}
	override public function hasGeometry():Bool return true;
	override public function geometry(detail:ComponentDetail = Preview):Part {
		return Solids.building([], owned -> {
			var block = Part.box(20, 12, beltWidth + 4);
			owned.push(block);
			var body = block.translated(new Vector(0, 0, -(beltWidth + 4) / 2));
			owned.push(body); block.close();
			var slot = Part.box(22, bandThickness + 0.4, beltWidth + 0.4);
			owned.push(slot);
			var cut = slot.translated(new Vector(0, 0, -(beltWidth + 0.4) / 2));
			owned.push(cut); slot.close();
			return Solids.cut(body, [cut]);
		});
	}
}

/** Mounting plate seated against the nut flange; bore and bolt holes come from the nut. */
class GantryNutMount extends MachineComponent {
	public final nut:LeadScrewNut;
	public final size:Float;
	public static inline var THICKNESS:Float = 6;
	public function new(nut:LeadScrewNut) {
		super("GANTRY-NUT-MOUNT-" + nut.designation, "Gantry screw nut mounting plate", "aluminium 6061", true);
		this.nut = nut; size = nut.flangeDiameter + 8;
	}
	override public function hasGeometry():Bool return true;
	override public function geometry(detail:ComponentDetail = Preview):Part {
		return Solids.building([], owned -> {
			var body = Part.box(size, size, THICKNESS);
			owned.push(body);
			var tools = [Part.cylinderSpan((nut.screwDiameter + 1) / 2, -0.1, THICKNESS + 0.1)];
			owned.push(tools[0]);
			var radius = nut.mountScrewPart(10).clearanceDiameter(ClearanceFit.Medium) / 2;
			for (point in nut.boltPattern()) {
				var hole = Part.cylinderSpan(radius, -0.1, THICKNESS + 0.1, point.x, point.y);
				owned.push(hole); tools.push(hole);
			}
			return Solids.cut(body, tools);
		});
	}
}

/** Connected bracket sections with a bored mounting interface. */
class GantryBoredBracket extends GantryPlate {
	public final boreFace:AssemblyFrame;
	public final clearanceDiameter:Float;
	public final sections:Array<Array<Float>>;
	public function new(name:String, width:Float, depth:Float, height:Float,
			boreFace:AssemblyFrame, clearanceDiameter:Float, ?sections:Array<Array<Float>>) {
		if (!Math.isFinite(clearanceDiameter) || clearanceDiameter <= 0)
			throw "Bracket needs a positive bore diameter";
		super(name + "-BORE" + Dimension.format(clearanceDiameter), width, depth, height);
		this.boreFace = boreFace; this.clearanceDiameter = clearanceDiameter;
		this.sections = sections == null ? [[-width / 2, -depth / 2, 0.0, width / 2, depth / 2, height]] : [for (section in sections) section.copy()];
		if (this.sections.length == 0) throw "Bracket needs a solid section";
		addFacet(machinekit.component.CollisionHullFacet.fromBoxes(this.sections));
		for (section in this.sections) {
			if (section.length != 6) throw "Bracket section needs six bounds";
			for (value in section) if (!Math.isFinite(value)) throw "Bracket bounds must be finite";
			for (axis in 0...3) if (section[axis + 3] <= section[axis]) throw "Bracket section must have positive volume";
		}
	}
	override public function geometry(detail:ComponentDetail = Preview):Part {
		return Solids.building([], owned -> {
			var parts:Array<Part> = [];
			for (section in sections) {
				var block = Part.box(section[3] - section[0], section[4] - section[1], section[5] - section[2]);
				owned.push(block);
				var placed = block.translated(new Vector((section[0] + section[3]) / 2, (section[1] + section[4]) / 2, section[2]));
				owned.push(placed); block.close(); parts.push(placed);
			}
			var body = parts.length == 1 ? parts[0] : Solids.union(parts);
			if (parts.length != 1) owned.push(body);
			var extent = width + depth + height;
			var bore = Part.cylinderSpan(clearanceDiameter / 2, -extent, extent);
			owned.push(bore);
			var x = AssemblyFrames.transformVector(boreFace, 1, 0, 0), z = AssemblyFrames.transformVector(boreFace, 0, 0, 1);
			var tool = bore.placed(new Location(new Plane(new Vector(boreFace.x, boreFace.y, boreFace.z),
				new Vector(x.x, x.y, x.z), new Vector(z.x, z.y, z.z))));
			owned.push(tool); bore.close();
			return Solids.cut(body, [tool]);
		});
	}
}

/** Frame/carriage bridge with clearance for the actual motor shaft or pinion. */
class GantryMotorSupport extends GantryBoredBracket {
	public final mountedMotor:NemaStepper;
	public final shaftFace:AssemblyFrame;
	public function new(name:String, width:Float, depth:Float, height:Float, motor:NemaStepper,
			shaftFace:AssemblyFrame, ?sections:Array<Array<Float>>, ?clearanceDiameter:Float) {
		var bore = clearanceDiameter == null ? motor.variant.shaftDiameter + 0.5 : clearanceDiameter;
		if (!Math.isFinite(bore) || bore < motor.variant.shaftDiameter + 0.5)
			throw "Motor support clearance must cover its shaft";
		super(name, width, depth, height, shaftFace, bore, sections);
		mountedMotor = motor; this.shaftFace = shaftFace;
	}
}
