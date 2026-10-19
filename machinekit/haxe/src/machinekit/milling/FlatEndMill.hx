package machinekit.milling;

import cadkit.modeling.Part;
import cadkit.modeling.Curve;
import cadkit.modeling.Plane;
import cadkit.modeling.Vector;
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
		var blank=stickout==flute ? Part.cylinderSpan(radius,0,flute) :
			Solids.union([Part.cylinderSpan(radius,0,flute),Part.cylinderSpan(radius*0.9333333333,flute,stickout)]);
		if(detail==Envelope)return blank;
		// Two assumed 30-degree helical gullets make orientation readable at preview fidelity.
		// Keep the packaging envelope available separately from these nominal cutting details.
		var cuts:Array<Part> =[];
		try {
			for(phase in [0.0,Math.PI]) {
				var sections:Array<Curve> =[];
				try {
					var twist=flute*Math.tan(Math.PI/6)/radius;
					var steps=Std.int(Math.min(96,Math.max(8,Math.ceil(twist/(Math.PI/12)))));
					for(i in 0...steps+1) {
						var z=-0.02+(flute+0.04)*i/steps,a=phase+z*Math.tan(Math.PI/6)/radius;
						sections.push(Curve.circle(radius*0.48,new Plane(new Vector(radius*0.88*Math.cos(a),radius*0.88*Math.sin(a),z),Vector.X(),Vector.Z())));
					}
					cuts.push(Part.loft(sections));
					for(section in sections)section.close();
				}catch(error:Dynamic){for(section in sections)section.close();throw error;}
			}
			return Solids.cut(blank,cuts);
		}catch(error:Dynamic){blank.close();for(cut in cuts)cut.close();throw error;}
	}
}
