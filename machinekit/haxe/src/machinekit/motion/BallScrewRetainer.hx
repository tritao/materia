package machinekit.motion;

import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.component.MachineComponent;
import machinekit.component.ComponentDetail;
import machinekit.component.Solids;
import machinekit.component.ComponentType;
import machinekit.component.ComponentValues;

/** Generic notched retaining nut and washer, assumed dimensions, not a selected bearing locknut.
 * Local Z=0 is the motor-side face; the washer bears at Z=5. */
class BallScrewRetainer extends MachineComponent {
	public static inline var STATION:Float = 20;
	static var recipe:Null<ComponentType>;
	public static function recipeType():ComponentType {
		if (recipe == null) recipe = new ComponentType("machinekit.motion.ballscrew-retainer", [], _ -> new BallScrewRetainer());
		return recipe;
	}
	override public function componentType():Null<ComponentType> return recipeType();
	public function new() {
		super("BALLSCREW-D12-RETAINER-REFERENCE", "12 mm screw retaining nut and washer (assumed)", "steel", false, new ComponentValues());
		addConnector("axis", Axis, Solids.axial(0, 0, 0));
		addConnector("bearingFace", Mount, Solids.axial(0, 0, 5));
	}
	override public function hasGeometry():Bool return true;
	override public function geometry(detail:ComponentDetail = Preview):Part {
		var body = Solids.union([
			Solids.named(Part.cylinderSpan(10, 0, 4), "retaining-nut"),
			Solids.named(Part.cylinderSpan(10.5, 4, 5), "washer")]);
		var tools = [Part.cylinderSpan(6.1, -0.1, 5.1)];
		if (detail == Preview) for (axis in 0...4) {
			var angle = axis * Math.PI / 2;
			var tool = Part.box(axis % 2 == 0 ? 4 : 3, axis % 2 == 0 ? 3 : 4, 4.2);
			var placed = tool.translated(new Vector(10 * Math.cos(angle), 10 * Math.sin(angle), -0.1));
			tool.close(); tools.push(placed);
		}
		return Solids.cut(body, tools);
	}
}
