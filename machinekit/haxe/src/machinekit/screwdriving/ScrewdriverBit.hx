package machinekit.screwdriving;

import cadkit.modeling.Part;
import machinekit.component.ComponentDetail;
import machinekit.component.MachineComponent;
import machinekit.component.Solids;
import machinekit.standard.ScrewDrive;

/** Replaceable bit and vacuum sleeve. The drive states its fit; the preview is its envelope. */
class ScrewdriverBit extends MachineComponent {
	static inline var TIP_LENGTH:Float = 8;
	static inline var STEM_RADIUS:Float = 3;
	static inline var SLEEVE_BORE:Float = 3.5;
	static inline var SLEEVE_FLOOR:Float = 3;
	public final drive:ScrewDrive;
	public final length:Float;
	public final sleeveRadius:Float;
	/** Authored manufacturing clearance across flats; nominal drive identity stays unchanged. */
	public final fitClearance:Float;

	public function new(drive:ScrewDrive, length:Float = 45, sleeveRadius:Float = 6, fitClearance:Float = 0) {
		if (!Math.isFinite(fitClearance) || fitClearance < 0) throw "Bit fit clearance must be finite and non-negative";
		if (drive == null || !Math.isFinite(length) || length <= TIP_LENGTH+SLEEVE_FLOOR || !Math.isFinite(sleeveRadius) || sleeveRadius <= SLEEVE_BORE)
			throw "Screwdriver bit needs a drive and positive finite dimensions";
		super("CUSTOM-SCREWDRIVER-BIT-" + Std.string(drive) + "-L" + machinekit.component.Dimension.format(length)
			+ "-R" + machinekit.component.Dimension.format(sleeveRadius)
			+ (fitClearance == 0 ? "" : "-C" + machinekit.component.Dimension.format(fitClearance)), "Replaceable screwdriver bit and vacuum sleeve", "steel", true);
		this.drive = drive; this.length = length; this.sleeveRadius = sleeveRadius; this.fitClearance = fitClearance;
		var data = driveData();
		if (data.size.length == 0) throw "Screwdriver bit needs a drive size";
		switch drive {
			case HexSocket(size), HexHead(size), Slot(size): if (!Math.isFinite(size) || size <= 0) throw "Screwdriver bit needs a positive drive size";
			case Cross(size): if (size < 0) throw "Screwdriver bit needs a non-negative cross size";
			case Hexalobular(size): if (size.length == 0) throw "Screwdriver bit needs a hexalobular size";
		}
		switch drive {
			case HexSocket(size): if (fitClearance >= size) throw "Bit fit clearance must leave a positive hex profile";
			case _: if (fitClearance != 0) throw "Bit fit clearance requires an authored hex-socket profile";
		}
		addConnector("mount", Mount, Solids.axial(0, 0, 0));
		addConnector("tcp", Mount, materia.assembly.AssemblyFrames.translation(0, 0, length));
		addPort({name: "vacuum", kind: Vacuum, role: Consumer, iface: ScrewdrivingInterfaces.vacuum(), required: true});
		var pieces = [machinekit.component.CollisionHullFacet.cylinder(STEM_RADIUS,0,length-TIP_LENGTH),
			machinekit.component.CollisionHullFacet.cylinder(sleeveRadius,0,SLEEVE_FLOOR)];
		var sleeve = sleeveGeometry(), tip = tipGeometry();
		try {
			pieces = pieces.concat(machinekit.component.CollisionHullFacet.fromExtrusion(sleeve,SLEEVE_FLOOR,length-TIP_LENGTH).hulls);
			pieces = pieces.concat(machinekit.component.CollisionHullFacet.fromExtrusion(tip,length-TIP_LENGTH,length).hulls);
		} catch (error:Dynamic) { sleeve.close(); tip.close(); throw error; }
		sleeve.close(); tip.close();
		addFacet(new machinekit.component.CollisionHullFacet(pieces));
	}
	public function matches(other:ScrewDrive):Bool return switch drive {
		case HexSocket(a): switch other { case HexSocket(b): a == b; default: false; }
		case Hexalobular(a): switch other { case Hexalobular(b): a == b; default: false; }
		case Cross(a): switch other { case Cross(b): a == b; default: false; }
		case HexHead(a): switch other { case HexHead(b): a == b; default: false; }
		case Slot(a): switch other { case Slot(b): a == b; default: false; }
	};

	public function driveData():materia.project.SceneArtifact.SceneArtifactScrewdriverBit return switch drive {
		case HexSocket(size): {kind: "hex-socket", size: Std.string(size)};
		case Hexalobular(size): {kind: "hexalobular", size: size};
		case Cross(size): {kind: "cross", size: Std.string(size)};
		case HexHead(size): {kind: "hex-head", size: Std.string(size)};
		case Slot(size): {kind: "slot", size: Std.string(size)};
	};
	override public function hasGeometry():Bool return true;
	override public function geometry(detail:ComponentDetail = Preview):Part {
		// Hex socket bits retain their actual across-flats tip. Symbolic drives use
		// an envelope until their manufacturing profiles are specified.
		return Solids.union([Solids.named(Part.cylinderSpan(STEM_RADIUS,0,length-TIP_LENGTH), "shank"),
			Solids.named(tipGeometry(), "bit"), Solids.named(sleeveGeometry(), "vacuum-sleeve")]);
	}
	function tipGeometry():Part return switch drive {
		case HexSocket(size):
			if (fitClearance >= size) throw "Bit fit clearance must leave a positive hex profile";
			Part.prism(cadkit.modeling.Polygon.regular(6,size-fitClearance),length-TIP_LENGTH,length);
		case _: Part.cylinderSpan(STEM_RADIUS,length-TIP_LENGTH,length);
	};
	function sleeveGeometry():Part return Solids.cut(Part.cylinderSpan(sleeveRadius,0,length-TIP_LENGTH),
		[Part.cylinderSpan(SLEEVE_BORE,SLEEVE_FLOOR,length-TIP_LENGTH+0.1)]);
}
