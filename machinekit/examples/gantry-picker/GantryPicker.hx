import machinekit.gantry.Gantry;
import machinekit.gantry.GantrySpec;
import machinekit.gantry.GantrySpec.GantryDrive;
import machinekit.gantry.GantrySpec.GantryHead;
import machinekit.transmission.TimingBeltProfile;
import machinekit.robotics.RobotFlange;
import machinekit.robotics.SuctionTool;
import machinekit.component.MachineComponent;
import machinekit.component.ComponentDetail;
import machinekit.component.Dimension;
import machinekit.component.Solids;
import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import materia.assembly.AssemblyFrames;

/** Six cartons move from the infeed to a two-row pallet pattern. */
class GantryPicker extends Gantry {
	public static inline var TABLE_TOP:Float = 200;
	public static inline var PAD_HEIGHT:Float = 10;
	public static inline var BOX_WIDTH:Float = 70;
	public static inline var BOX_HEIGHT:Float = 50;
	public static inline var BOX_COUNT:Int = 6;
	public final tool:SuctionTool;

	public function new() {
		var belt:GantryDrive = Belt(TimingBeltProfile.GT2, 20, 9);
		// The suction tool projects in front of the slide. Keep the front crossbar
		// beyond its vertical sweep, including the guide's homing overtravel.
		super(new GantrySpec(1500, 1000, 500, belt, belt, belt, true, "MGN12C", 23,
			"HFS5-4040", "HFS5-4040", false, GantryHead.None, 0.5, 24, 16, 200, 100, 50, 60));
		var flange:RobotFlange = cast component("flange");
		tool = new SuctionTool(flange);
		include("tool", tool);
		addMate("tool-mount", "fixed", "flange", "face", "tool/plate", "robot");
		exposeConnector("toolContact", "tool/cup", "contact");
		exposePort("compressedAir", "tool/ejector", "air");
		addComponent("infeed", new PickerTable(400, 300, TABLE_TOP), AssemblyFrames.translation(250, 200, 0));
		addComponent("palletTable", new PickerTable(500, 350, TABLE_TOP), AssemblyFrames.translation(1100, 650, 0));
		var pad = new PickerBlock(90, 90, PAD_HEIGHT, "rubber", "seat");
		var box = new PickerBlock(BOX_WIDTH, BOX_WIDTH, BOX_HEIGHT, "birch plywood", "carton");
		for (index in 0...BOX_COUNT) {
			var column = index % 3, row = Std.int(index / 3);
			addComponent("box" + index, box, AssemblyFrames.translation(150 + 100 * column, 150 + 100 * row, TABLE_TOP + PAD_HEIGHT));
			addComponent("infeedSeat" + index, pad, AssemblyFrames.translation(150 + 100 * column, 150 + 100 * row, TABLE_TOP));
			addComponent("slot" + index, pad, AssemblyFrames.translation(1000 + 100 * column, 600 + 100 * row, TABLE_TOP));
		}
	}
}

/** Table top and four physically connected legs; all dimensions are authored assumptions. */
class PickerTable extends MachineComponent {
	final width:Float;
	final depth:Float;
	final height:Float;
	public function new(width:Float, depth:Float, height:Float) {
		super("PICKER-TABLE-" + Dimension.format(width) + "x" + Dimension.format(depth) + "x" + Dimension.format(height),
			"Gantry picker table", "steel", true);
		this.width = width; this.depth = depth; this.height = height;
	}
	override public function hasGeometry():Bool return true;
	override public function geometry(detail:ComponentDetail = Preview):Part {
		return Solids.building([], owned -> {
			var parts:Array<Part> = [];
			var top = Part.box(width, depth, 20); owned.push(top);
			var slab = top.translated(new Vector(0, 0, height - 20)); owned.push(slab); top.close(); parts.push(slab);
			for (sx in [-1, 1]) for (sy in [-1, 1]) {
				var leg = Part.box(30, 30, height - 20); owned.push(leg);
				var placed = leg.translated(new Vector(sx * (width / 2 - 30), sy * (depth / 2 - 30), 0));
				owned.push(placed); leg.close(); parts.push(placed);
			}
			return Solids.union(parts);
		});
	}
}

/** Carton or landing pad, standing on local Z=0 with its grasp/seat at the top. */
class PickerBlock extends MachineComponent {
	final width:Float;
	final depth:Float;
	final height:Float;
	public function new(width:Float, depth:Float, height:Float, material:String, name:String) {
		super("PICKER-" + name + "-" + Dimension.format(width) + "x" + Dimension.format(depth) + "x" + Dimension.format(height), name, material, true);
		this.width = width; this.depth = depth; this.height = height;
		addConnector("base", machinekit.component.ConnectorRole.Mount, Solids.axial(0, 0, 0));
		addConnector("top", machinekit.component.ConnectorRole.Mount, Solids.axial(0, 0, height));
	}
	override public function hasGeometry():Bool return true;
	override public function geometry(detail:ComponentDetail = Preview):Part return Part.box(width, depth, height);
}
