package machinekit.milling;

import cadkit.modeling.Part;
import machinekit.component.*;
import machinekit.component.ComponentRecipeSupport as P;

/** Two-bolt removable cover across the front of the open motor stand. */
class MillZCouplingGuard extends MachineComponent {
	static var recipe:Null<ComponentType>;
	public static function parameters():Array<ComponentParameter> return [P.length("width",100),P.length("height",16.5)];
	public static function recipeType():ComponentType {
		if (recipe==null) recipe=new ComponentType("machinekit.milling.z-coupling-guard",parameters(),v->new MillZCouplingGuard(v)); return recipe;
	}
	public function new(?values:ComponentValues) {
		var v=P.resolve(parameters(),values);
		if (v.number("width")<50 || v.number("height")<12) throw "Coupling guard needs mounting edge clearance";
		super("MILL-Z-COUPLING-GUARD","Removable Z coupling guard","painted steel",false,v);
		addConnector("mount",Mount,Solids.axial(0,0,0));
	}
	override public function componentType():Null<ComponentType> return recipeType();
	override public function hasGeometry():Bool return true;
	override public function geometry(detail:ComponentDetail = Preview):Part {
		var w=parameterNumber("width"); return Solids.cut(Part.box(w,2,parameterNumber("height")),[
			Part.cylinderAlongY(2.25,-2,2,-(w-12)/2,8),Part.cylinderAlongY(2.25,-2,2,(w-12)/2,8)]);
	}
}
