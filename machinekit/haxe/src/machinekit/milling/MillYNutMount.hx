package machinekit.milling;

import cadkit.modeling.Part;
import machinekit.component.*;
import machinekit.motion.BallNut;

/** Compact front saddle plate. Native +Z faces forward, +Y is up.
 * Nut body and flange enter the saddle's underside tunnel. Nominal M5 nut
 * and M6 saddle interfaces; no tall bridge between screw and saddle.
 */
class MillYNutMount extends MachineComponent {
	static var recipe:Null<ComponentType>;
	public static function recipeType():ComponentType {
		if (recipe == null) recipe = new ComponentType("machinekit.milling.y-nut-mount", [], _ -> new MillYNutMount());
		return recipe;
	}
	public function new() {
		super("MILL-Y-NUT-MOUNT", "Compact Y nut saddle mounting plate", "steel", false, new ComponentValues());
		addConnector("mount", Mount, Solids.axial(0,0,0));
	}
	override public function componentType():Null<ComponentType> return recipeType();
	override public function hasGeometry():Bool return true;
	override public function geometry(detail:ComponentDetail = Preview):Part {
		var tools = [Part.cylinderSpan(9,-1,13)];
		for (p in new BallNut().boltPattern()) tools.push(Part.cylinderSpan(2.75,-1,13,p.x,p.y));
		for (x in [-32.0,32.0]) tools.push(Part.cylinderSpan(3.3,-1,13,x,12));
		return Solids.cut(Part.box(84,54,12),tools);
	}
}
