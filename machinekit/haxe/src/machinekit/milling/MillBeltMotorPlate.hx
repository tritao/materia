package machinekit.milling;
import cadkit.modeling.Part;
import machinekit.component.*;

/** X return-drive motor plate, across X, shaft along Y, bottom Z=0.
 * Fits a machined saddle recess and the catalog 60 mm Leadshine flange.
 */
class MillBeltMotorPlate extends MachineComponent {
	static var recipe:Null<ComponentType>;
	public static function recipeType():ComponentType {
		if (recipe == null) recipe = new ComponentType("machinekit.milling.belt-motor-plate", [], _ -> new MillBeltMotorPlate());
		return recipe;
	}
	public function new() {
		super("MILL-X-RETURN-MOTOR-PLATE", "Recessed Leadshine 60 mm X motor plate", "steel", false, new ComponentValues());
		addConnector("mount", Mount, Solids.axial(0, 0, 0));
	}
	override public function componentType():Null<ComponentType> return recipeType();
	override public function hasGeometry():Bool return true;
	override public function geometry(detail:ComponentDetail = Preview):Part {
		var holes = [Part.cylinderAlongY(8.5, -7, 7, 0, 40)];
		var m=machinekit.motion.LeadshineServo.model("ELM2H-0400LA60F").mounting();
		if(m==null)throw "X motor requires catalog mounting dimensions";
		holes.push(Part.cylinderAlongY((m.pilotDiameter+0.2)/2,-7,-6+m.pilotHeight+0.2,0,40));
		for (p in m.holes) holes.push(Part.cylinderAlongY(m.holeDiameter/2,-7,7,p.x,40+p.y));
		for (x in [-32.0, 32.0]) for (z in [10.0, 70.0]) holes.push(Part.cylinderAlongY(3.3, -7, 7, x, z));
		return Solids.cut(Part.box(86, 12, 80), holes);
	}
}
