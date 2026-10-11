package machinekit.screwdriving;

import cadkit.modeling.Part;
import machinekit.component.ComponentDetail;
import machinekit.component.MachineComponent;
import machinekit.component.Solids;
import machinekit.standard.ScrewDrive;

/** Replaceable bit and vacuum sleeve. The drive states its fit; the preview is its envelope. */
class ScrewdriverBit extends MachineComponent {
	public final drive:ScrewDrive;
	public final length:Float;
	public final sleeveRadius:Float;

	public function new(drive:ScrewDrive, length:Float = 45, sleeveRadius:Float = 6) {
		if (drive == null || !Math.isFinite(length) || length <= 8 || !Math.isFinite(sleeveRadius) || sleeveRadius <= 3.5)
			throw "Screwdriver bit needs a drive and positive finite dimensions";
		super("CUSTOM-SCREWDRIVER-BIT-" + Std.string(drive) + "-L" + machinekit.component.Dimension.format(length)
			+ "-R" + machinekit.component.Dimension.format(sleeveRadius), "Replaceable screwdriver bit and vacuum sleeve", "steel", true);
		this.drive = drive; this.length = length; this.sleeveRadius = sleeveRadius;
		var data = driveData();
		if (data.size.length == 0) throw "Screwdriver bit needs a drive size";
		switch drive {
			case HexSocket(size), HexHead(size), Slot(size): if (!Math.isFinite(size) || size <= 0) throw "Screwdriver bit needs a positive drive size";
			case Cross(size): if (size < 0) throw "Screwdriver bit needs a non-negative cross size";
			case Hexalobular(size): if (size.length == 0) throw "Screwdriver bit needs a hexalobular size";
		}
		addConnector("mount", Mount, Solids.axial(0, 0, 0));
		addConnector("tcp", Mount, materia.assembly.AssemblyFrames.translation(0, 0, length));
		addPort({name: "vacuum", kind: Vacuum, role: Consumer, iface: ScrewdrivingInterfaces.vacuum(), required: true});
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
		var tip = switch drive {
			case HexSocket(size): Part.prism(cadkit.modeling.Polygon.regular(6, size), length - 8, length);
			case _: Part.cylinderSpan(3, length - 8, length);
		};
		var sleeve = Solids.cut(Part.cylinderSpan(sleeveRadius, 0, length - 8),
			[Part.cylinderSpan(3.5, 3, length - 7.9)]);
		return Solids.union([Solids.named(Part.cylinderSpan(3, 0, length - 8), "shank"),
			Solids.named(tip, "bit"), Solids.named(sleeve, "vacuum-sleeve")]);
	}
}
