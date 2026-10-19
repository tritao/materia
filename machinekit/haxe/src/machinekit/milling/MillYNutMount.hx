package machinekit.milling;

import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.component.ComponentRecipeSupport as P;
import machinekit.component.*;
import machinekit.motion.BallNut;

/** Removable front saddle adapter. Native +Z faces forward, +Y is up.
 * Nut body and flange enter the saddle's underside tunnel. Nominal M5 nut
 * and M6 saddle interfaces; no tall bridge between screw and saddle.
 */
class MillYNutMount extends MachineComponent {
	public static inline var THICKNESS:Float = 12;
	public static inline var FLANGE_BOLT_LENGTH:Float = 16;
	public static inline var NUT_THREAD_DEPTH:Float = 8;
	public static inline var NUT_DRILL_DEPTH:Float = 10; // includes conservative drill-point envelope
	public static inline var SERVICE_WIDTH:Float = 14;
	public static inline var SERVICE_BOTTOM:Float = 24;
	public static inline var ANCHOR_SEAT:Float = 6;
	public static inline var ANCHOR_LENGTH:Float = 25;
	public static inline var BOSS_WIDTH:Float = 116;
	public static inline var BOSS_DEPTH:Float = 28;
	public static function anchors():Array<{x:Float,y:Float}> return [for(x in [-40.0,40.0]) for(y in [14.0,48.0]) {x:x,y:y}];
	public static function pilots():Array<{x:Float,y:Float}> return [{x:-32,y:32},{x:32,y:32}];
	public static function parameters():Array<ComponentParameter> return [P.flag("reinforced",false),P.flag("serviceAccess",false)];
	static var recipe:Null<ComponentType>;
	public static function recipeType():ComponentType {
		if (recipe == null) recipe = new ComponentType("machinekit.milling.y-nut-mount", parameters(), v -> new MillYNutMount(v));
		return recipe;
	}
	public function new(?v:ComponentValues) {
		v=P.resolve(parameters(),v);
		if(v.boolean("serviceAccess") && !v.boolean("reinforced")) throw "Nut service slot requires reinforced adapter";
		super("MILL-Y-NUT-MOUNT", v.boolean("reinforced")?"Registered Y nut saddle adapter":"Compact Y nut saddle mounting plate", "steel", false, v);
		addConnector("mount", Mount, Solids.axial(0,0,0));
	}
	override public function componentType():Null<ComponentType> return recipeType();
	override public function hasGeometry():Bool return true;
	override public function geometry(detail:ComponentDetail = Preview):Part {
		if (values().boolean("reinforced")) {
			var nut=new BallNut(), pieces=[Part.box(100,92,THICKNESS).translated(new Vector(0,18,0))];
			// Shallow rear lip locates the nut flange without moving its axial seat.
			pieces.push(Solids.cut(Part.cylinderSpan(nut.flangeDiameter/2+3.5,-1.5,1),
				[Part.cylinderSpan(nut.flangeDiameter/2+0.05,-2,2),
				 Part.box(values().boolean("serviceAccess")?SERVICE_WIDTH:10,12,3).translated(new Vector(0,28,-2))]));
			for(p in pilots()) pieces.push(Part.cylinderSpan(3,-8,1,p.x,p.y));
			var cuts=[Part.cylinderSpan((nut.screwDiameter+2)/2,-1,THICKNESS+1)];
			for(p in nut.boltPattern()) cuts.push(Part.cylinderSpan(2.1,-1,values().boolean("serviceAccess")?NUT_DRILL_DEPTH:10,p.x,p.y));
			if (values().boolean("serviceAccess")) {
				// Open, end-milled service slot, not a drilled hole tangent to an edge.
				cuts.push(Part.cylinderSpan(SERVICE_WIDTH/2,-2,THICKNESS+1,0,SERVICE_BOTTOM+SERVICE_WIDTH/2));
				cuts.push(Part.box(SERVICE_WIDTH,40,THICKNESS+3).translated(new Vector(0,SERVICE_BOTTOM+SERVICE_WIDTH/2+20,-2)));
			}
			for(p in anchors()) {
				cuts.push(Part.cylinderSpan(3.3,-1,THICKNESS+1,p.x,p.y));
				cuts.push(Part.cylinderSpan(5.5,ANCHOR_SEAT,THICKNESS+1,p.x,p.y));
			}
			var raw=Solids.cut(Solids.union(pieces),cuts);
			try { var result=raw.refined(); raw.close(); return result; } catch(e:Dynamic) { raw.close(); throw e; }
		}
		var tools = [Part.cylinderSpan(9,-1,13)];
		for (p in new BallNut().boltPattern()) tools.push(Part.cylinderSpan(2.75,-1,13,p.x,p.y));
		for (x in [-32.0,32.0]) tools.push(Part.cylinderSpan(3.3,-1,13,x,12));
		return Solids.cut(Part.box(84,54,12),tools);
	}
}
