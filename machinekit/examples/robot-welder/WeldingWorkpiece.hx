import cadkit.modeling.Part;
import machinekit.assembly.MachineAssembly;
import machinekit.component.ComponentDetail;
import machinekit.component.Dimension;
import machinekit.component.MachineComponent;
import machinekit.component.Solids;
import machinekit.structural.FrameAssembly;
import machinekit.structural.RectTube;
import machinekit.welding.WeldMetal;
import machinekit.welding.Weldment;

/** One tube of a `FrameAssembly`, as a member of its own: the frame's geometry for that member, in the frame's own coordinates. */
class FrameMemberComponent extends MachineComponent {
	final frame:FrameAssembly;
	final memberName:String;

	public function new(frame:FrameAssembly, memberName:String, profile:RectTube) {
		super('FRAME-${memberName.toUpperCase()}-${profile.designation}-${Dimension.format(frame.cutLength(memberName))}',
			'Tube frame member $memberName, ${profile.description}, cut ${Dimension.format(frame.cutLength(memberName))}', "steel", true);
		this.frame = frame;
		this.memberName = memberName;
		addConnector("origin", Mount, Solids.axial(0, 0, 0));
	}

	override public function hasGeometry():Bool return true;

	override public function geometry(detail:ComponentDetail = Preview):Part return frame.geometry(memberName);
}

/**
 * The workpiece the cell welds: a base plate with an upright standing on it (a T-joint, two fillet seams), and
 * beside it a small tube frame, a beam on the table, a little wider than the posts, with two posts standing on it (a fillet around each post).
 *
 * Its frame is the base plate's: the plate's centre on its underside, +Z up. The tube frame sits `FRAME_OFFSET`
 * along +X, with the beam on the same level as the plate. Which joints are welded is declared by `weldment()`;
 * where the seams are is not: they come from the members' faces.
 */
class WeldingWorkpiece extends MachineAssembly {
	public static inline var PLATE_LENGTH:Float = 180;
	public static inline var PLATE_WIDTH:Float = 150;
	public static inline var PLATE_THICKNESS:Float = 10;
	public static inline var UPRIGHT_THICKNESS:Float = 8;
	public static inline var UPRIGHT_HEIGHT:Float = 80;
	/** The frame's origin, on the table beside the plate, measured from the plate's centre. */
	public static inline var FRAME_OFFSET:Float = 290;
	public static inline var TUBE_WALL:Float = 3;
	/** How much wider than the posts the beam is. */
	public static inline var BEAM_EXTRA:Float = 20;
	public static inline var POST_HEIGHT:Float = 60;
	/** Distance from the beam's centre to each post's centre. */
	public static inline var POST_SPACING:Float = 80;
	public static inline var LEG_SIZE:Float = 5;

	public final uprightLength:Float;
	public final tubeSize:Float;
	public final frame = new FrameAssembly();

	public function new(uprightLength:Float = PLATE_LENGTH, tubeSize:Float = 40) {
		super();
		this.uprightLength = uprightLength;
		this.tubeSize = tubeSize;
		addComponent("basePlate", new ArmBlock(PLATE_LENGTH, PLATE_WIDTH, PLATE_THICKNESS, "steel", "Base plate"));
		addComponent("upright", new ArmBlock(uprightLength, UPRIGHT_THICKNESS, UPRIGHT_HEIGHT, "steel", "Upright"));
		addMate("upright-mate", "fixed", "basePlate", "top", "upright", "base");
		// The weld metal rides on the workpiece, at its origin, and carries the bead the welder lays.
		addComponent("weldMetal", new WeldMetal());
		addMate("weld-metal-mate", "fixed", "basePlate", "base", "weldMetal", "base");

		// The tube frame: a beam lying on the table, its two posts standing on its top.
		var tube = new RectTube(tubeSize, tubeSize, TUBE_WALL);
		// The beam is wider than the posts, so a fillet runs round each post's foot rather than a flush edge.
		var beamTube = new RectTube(tubeSize + BEAM_EXTRA, tubeSize, TUBE_WALL);
		var beamHalf = POST_SPACING + tubeSize;
		frame.point("beamStart", -beamHalf, 0, tubeSize / 2);
		frame.point("beamEnd", beamHalf, 0, tubeSize / 2);
		frame.member("beam", "beamStart", "beamEnd", beamTube);
		for (side in [-1, 1]) {
			var name = side < 0 ? "postLeft" : "postRight";
			frame.point(name + "Foot", side * POST_SPACING, 0, tubeSize);
			frame.point(name + "Top", side * POST_SPACING, 0, tubeSize + POST_HEIGHT);
			frame.member(name, name + "Foot", name + "Top", tube);
		}
		addMemberConnector("basePlate", "frameSeat", Solids.axial(FRAME_OFFSET, 0, 0));
		for (name in ["beam", "postLeft", "postRight"]) {
			addComponent(name, new FrameMemberComponent(frame, name, name == "beam" ? beamTube : tube));
			addMate(name + "-mate", "fixed", "basePlate", "frameSeat", name, "origin");
		}
	}

	/** The welded joints: both sides of the upright to the plate, and each post to the beam. */
	public function weldment():Weldment
		return new Weldment("basePlate", ["basePlate", "upright", "beam", "postLeft", "postRight"])
			.join("basePlate", "upright", LEG_SIZE, 1)
			.join("beam", "postLeft", LEG_SIZE, 1)
			.join("beam", "postRight", LEG_SIZE, 1);
}
