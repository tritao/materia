package machinekit.pressbrake;

import cadkit.modeling.Part;
import machinekit.component.ComponentDetail;
import machinekit.component.ComponentRecipeSupport;
import machinekit.component.ComponentType;
import machinekit.component.ComponentValues;
import machinekit.component.Dimension;
import machinekit.component.MachineComponent;
import machinekit.component.Solids;

/** Reference tooling clamp. Dimensions are an assumed envelope, with no inferred holding rating. */
class ToolClamp extends MachineComponent {
	public final mode:ToolClampMode;
	public var control(get, never):ToolClampControl;
	public final width:Float;
	public final depth:Float;
	public final height:Float;
	static var recipe:Null<ComponentType>;

	public function new(mode:ToolClampMode, width:Float = 50, depth:Float = 40, height:Float = 60) {
		if (mode == null || !Math.isFinite(width) || !Math.isFinite(depth) || !Math.isFinite(height)
			|| !(width > 0 && depth > 0 && height > 0)) throw "Tool clamp needs a mode and positive dimensions";
		super('TOOL-CLAMP-$mode-${Dimension.format(width)}x${Dimension.format(depth)}x${Dimension.format(height)}',
			"Reference press-brake tool clamp", "steel", true);
		this.mode = mode;
		this.width = width;
		this.depth = depth;
		this.height = height;
		addConnector("mount", Mount, Solids.axial(0, 0, height));
		addConnector("tool", Face, Solids.axial(0, 0, 0));
		switch mode {
			case Manual:
			case Powered:
				addPort({name: "lock", kind: Signal, role: Consumer, iface: Plug("generic", 3), required: true});
				addPort({name: "unlock", kind: Signal, role: Consumer, iface: Plug("generic", 3), required: true});
				addPort({name: "locked", kind: Signal, role: Supply, iface: Plug("generic", 3), required: true});
				addPort({name: "air", kind: Pneumatic, role: Consumer, iface: PushIn(6), required: true});
		}
	}

	function get_control():ToolClampControl return switch mode {
		case Manual: ToolClampControl.Manual;
		case Powered: ToolClampControl.Powered("lock", "unlock", "locked", "air");
	};

	override public function hasGeometry():Bool return true;
	override public function geometry(detail:ComponentDetail = Preview):Part return Part.box(width, depth, height);

	public static function recipeType():ComponentType {
		if (recipe == null) recipe = new ComponentType("machinekit.pressbrake.tool-clamp", [
			ComponentRecipeSupport.choice("mode", ["Manual", "Powered"], "Manual"),
			ComponentRecipeSupport.length("width", 50), ComponentRecipeSupport.length("depth", 40), ComponentRecipeSupport.length("height", 60)
		], v -> {
			var mode = switch v.token("mode") {
				case "Manual": ToolClampMode.Manual;
				case "Powered": ToolClampMode.Powered;
				case value: throw 'Unknown tool clamp mode "$value"';
			};
			return new ToolClamp(mode, v.number("width"), v.number("depth"), v.number("height"));
		});
		return recipe;
	}

	override public function componentType():Null<ComponentType> return recipeType();
	override public function values():ComponentValues return new ComponentValues().setToken("mode", Std.string(mode))
		.setNumber("width", width).setNumber("depth", depth).setNumber("height", height).setToken("material", materialSpec());
}
