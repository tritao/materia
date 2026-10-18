package machinekit.milling;

import cadkit.modeling.Part;
import machinekit.component.*;

/** BT30 packaging study. Ø90 body and Ø125 flange follow the DAIYA BT30SP
 * envelope; axial interfaces, bearing/shaft internals and mounting holes are
 * explicit project assumptions, not a supplier interchangeable cartridge. */
class Bt30Cartridge extends MachineComponent {
	public static inline var DIAMETER:Float = 90;
	public static inline var SEAT:Float = 60;
	public static inline var REAR:Float = 242;
	static var recipe:Null<ComponentType>;
	public static function recipeType():ComponentType {
		if (recipe == null) recipe = new ComponentType("machinekit.milling.bt30-cartridge-study",[],v -> new Bt30Cartridge());
		return recipe;
	}
	public function new() {
		super("BT30-CARTRIDGE-STUDY","BT30 stationary housing — assumed interfaces","steel");
		for (row in [{id:"gaugeLine",z:0.0},{id:"mount",z:SEAT},{id:"reaction",z:REAR},{id:"pulley",z:250.0}])
			addConnector(row.id,Mount,Solids.axial(0,0,row.z));
	}
	override public function componentType():Null<ComponentType> return recipeType();
	override public function values():ComponentValues return new ComponentValues().setToken("material",materialSpec());
	override public function hasGeometry():Bool return true;
	override public function geometry(detail:ComponentDetail = Preview):Part {
		var body = Solids.union([Part.cylinderSpan(45,SEAT,REAR),Part.cylinderSpan(62.5,40,SEAT)]);
		// The nose, receiving taper and drawbar spring seat belong to the rotor.
		var cuts=[Part.cylinderSpan(26,39,REAR+1)];
		for (i in 0...6) { var a=i*Math.PI/3; cuts.push(Part.cylinderSpan(3.3,39,61,53.5*Math.cos(a),53.5*Math.sin(a))); }
		for (p in [[-37.5,0.0],[37.5,0.0],[0.0,-37.5],[0.0,37.5]]) cuts.push(Part.cylinderSpan(2.1,231,243,p[0],p[1]));
		return Solids.cut(body,cuts);
	}
}
