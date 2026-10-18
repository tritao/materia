package machinekit.milling;

import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import cadkit.modeling.Sketch;
import cadkit.modeling.Plane;
import machinekit.component.*;

/** Removable belt enclosure study. The motor carrier closes its rear top;
 * explicit clearances surround reaction posts, carrier webs and fasteners. */
class MillSpindleBeltGuard extends MachineComponent {
	static var recipe:Null<ComponentType>;
	public static function recipeType():ComponentType {
		if (recipe==null) recipe=new ComponentType("machinekit.milling.spindle-belt-guard",[],v->new MillSpindleBeltGuard()); return recipe;
	}
	public function new() {
		super("MILL-SPINDLE-BELT-GUARD","Removable spindle belt guard — concept","painted steel");
		addConnector("mount",Mount,Solids.axial(0,0,0));
	}
	override public function componentType():Null<ComponentType> return recipeType();
	override public function values():ComponentValues return new ComponentValues().setToken("material",materialSpec());
	override public function hasGeometry():Bool return true;
	/** Analytic capsule; display tessellation must not become CAD topology. */
	static function capsule(radius:Float, bottom:Float, top:Float):Part {
		var distance=Math.sqrt(150*150+78*78);
		return slot(distance+2*radius,2*radius,new Plane(new Vector(75,39,bottom),new Vector(150,78,0),Vector.Z()),top-bottom);
	}
	static function slot(length:Float,width:Float,plane:Plane,height:Float):Part {
		var profile=Sketch.slot(length,width,plane);
		try { var part=profile.extrude(height); profile.close(); return part; }
		catch (error:Dynamic) { profile.close(); throw error; }
	}

	override public function geometry(detail:ComponentDetail = Preview):Part {
		var wall=Solids.cut(capsule(48,251,272),[capsule(46,250,273)]);
		var top=capsule(48,270,272),cuts=[Part.box(162,102,4).translated(new Vector(125,78,269))];
		for (p in [[0.0,0.0],[150.0,78.0]]) cuts.push(Part.cylinderSpan(12,250,273,p[0],p[1]));
		for (x in [-60.0,60.0]) for (y in [-35.0,35.0]) cuts.push(Part.cylinderSpan(8,250,273,x,y));
		for (y in [44.0,92.0]) cuts.push(Part.box(26,16,23).translated(new Vector(80,y,250)));
		for (x in [120.0,180.0]) for (y in [48.0,108.0]) cuts.push(slot(28,12,new Plane(new Vector(x,y,263),Vector.X(),Vector.Z()),10));
		var shell=Solids.cut(Solids.union([wall,top]),cuts);
		var joined=Solids.union([shell,Part.box(16,16,2).translated(new Vector(96,105,268)),Part.box(20,14,2).translated(new Vector(165,121,268))]);
		var finished=Solids.cut(joined,[Part.cylinderSpan(2.25,267,271,94,108),Part.cylinderSpan(2.25,267,271,165,121),Part.cylinderSpan(4.5,263,268,94,108),Part.cylinderSpan(4.5,263,268,165,121)]);
		try { var result=finished.refined(); finished.close(); return result; }
		catch (error:Dynamic) { finished.close(); throw error; }
	}
}
