package machinekit.milling;

import cadkit.modeling.Part;
import machinekit.component.*;
import machinekit.component.ComponentRecipeSupport as P;

/** Visible flat cutter envelope, origin at tool tip. The buried shank is not
 * represented; stickout is measured from the collet's front face. */
class FlatEndMill extends MachineComponent {
	static var recipe:Null<ComponentType>;
	public static function parameters():Array<ComponentParameter> return [P.length("diameter",6),P.length("fluteLength",22),P.length("stickout",35)];
	public static function recipeType():ComponentType {
		if (recipe==null) recipe=new ComponentType("machinekit.milling.flat-end-mill",parameters(),v->new FlatEndMill(v)); return recipe;
	}
	public function new(?values:ComponentValues) {
		var v=P.resolve(parameters(),values);
		if (v.number("diameter")<=0 || v.number("fluteLength")<=0 || v.number("stickout")<v.number("fluteLength")) throw "Cutter needs positive dimensions and flutes within stickout";
		super("FLAT-END-MILL","Flat end mill — nominal visible envelope","steel",false,v);
		addConnector("tip",Mount,Solids.axial(0,0,0));
	}
	override public function componentType():Null<ComponentType> return recipeType();
	override public function hasGeometry():Bool return true;
	override public function geometry(detail:ComponentDetail = Preview):Part {
		var radius=parameterNumber("diameter")/2,flute=parameterNumber("fluteLength"),stickout=parameterNumber("stickout");
		if (stickout==flute) return Part.cylinderSpan(radius,0,flute);
		return Solids.union([Part.cylinderSpan(radius,0,flute),Part.cylinderSpan(radius*0.9333333333,flute,stickout)]);
	}
}
