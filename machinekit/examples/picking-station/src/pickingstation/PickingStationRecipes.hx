package pickingstation;

import machinekit.component.ComponentRecipeSupport;
import machinekit.component.ComponentType;
import machinekit.component.ComponentValues;
import machinekit.component.MachineKitComponents;

/** Application recipes used by saved picking-station assemblies. */
class PickingStationRecipes {
	static var types:Null<Array<ComponentType>>;
	static function length(name:String, value:Float):machinekit.component.ComponentParameter
		return ComponentRecipeSupport.length(name, value);

	public static function all():Array<ComponentType> {
		if (types == null) {
			types = [
				new ComponentType("pickingstation.rack-frame", [length("width", 900), length("depth", 600),
					length("height", 1800), length("footHeight", 25),
					ComponentRecipeSupport.count("shelfCount", 2), length("shelfSpacing", 300)],
					v -> new RackFrame(v.number("width"), v.number("depth"), v.number("height"),
						v.number("footHeight"), v.integer("shelfCount"), v.number("shelfSpacing")), true),
				new ComponentType("pickingstation.shelf", [length("width", 800), length("depth", 400),
					length("thickness", 18), ComponentRecipeSupport.scalar("inclinationDegrees", 0, -30, 30)],
					v -> new ShelfAssembly(v.number("width"), v.number("depth"), v.number("thickness"),
						v.number("inclinationDegrees")), true),
				new ComponentType("pickingstation.bin", [length("width", 220), length("depth", 300),
					length("height", 180), length("wallThickness", 3)],
					v -> new StorageBin(v.number("width"), v.number("depth"), v.number("height"),
						v.number("wallThickness")), true),
				new ComponentType("pickingstation.indicator", [length("width", 70), length("depth", 24),
					length("height", 48), length("confirmationDiameter", 18)],
					v -> new PickIndicator(v.number("width"), v.number("depth"), v.number("height"),
						v.number("confirmationDiameter")), true),
				new ComponentType("pickingstation.adjustable-foot", [length("diameter", 35), length("height", 25)],
					v -> new AdjustableFoot(v.number("diameter"), v.number("height")), true)
			];
			for (type in types) MachineKitComponents.defaultRegistry().register(type);
		}
		return types.copy();
	}

	public static function rackFrame():ComponentType return all()[0];
	public static function shelf():ComponentType return all()[1];
	public static function bin():ComponentType return all()[2];
	public static function indicator():ComponentType return all()[3];
	public static function foot():ComponentType return all()[4];

	public static function values(component:machinekit.component.MachineComponent):ComponentValues {
		var result = new ComponentValues();
		if (Std.isOfType(component, RackFrame)) {
			var part:RackFrame = cast component;
			result.setNumber("width", part.width).setNumber("depth", part.depth)
				.setNumber("height", part.height).setNumber("footHeight", part.footHeight)
				.setInteger("shelfCount", part.shelfCount).setNumber("shelfSpacing", part.shelfSpacing);
		} else if (Std.isOfType(component, ShelfAssembly)) {
			var part:ShelfAssembly = cast component;
			result.setNumber("width", part.width).setNumber("depth", part.depth)
				.setNumber("thickness", part.thickness).setNumber("inclinationDegrees", part.inclinationDegrees);
		} else if (Std.isOfType(component, StorageBin)) {
			var part:StorageBin = cast component;
			result.setNumber("width", part.width).setNumber("depth", part.depth)
				.setNumber("height", part.height).setNumber("wallThickness", part.wallThickness);
		} else if (Std.isOfType(component, PickIndicator)) {
			var part:PickIndicator = cast component;
			result.setNumber("width", part.width).setNumber("depth", part.depth)
				.setNumber("height", part.height).setNumber("confirmationDiameter", part.confirmationDiameter);
		} else if (Std.isOfType(component, AdjustableFoot)) {
			var part:AdjustableFoot = cast component;
			result.setNumber("diameter", part.diameter).setNumber("height", part.height);
		} else throw 'Unknown picking-station component "${component.designation}"';
		return result.setToken("material", component.materialSpec());
	}
}
