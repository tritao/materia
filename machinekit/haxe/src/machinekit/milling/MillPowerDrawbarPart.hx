package machinekit.milling;

import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.component.*;
import machinekit.component.ComponentRecipeSupport as P;

/** Separately itemised study hardware, in the cartridge gauge frame. Spring
 * preload, gripper details and cylinder pressure/force remain selection inputs. */
class MillPowerDrawbarPart extends MachineComponent {
	static var recipe:Null<ComponentType>;
	static final roles = ["drawbar","springs","gripper","pullStud","holder","reactionFrame","cylinder","pistonRod","rotor","piston"];
	public static function parameters():Array<ComponentParameter> return [P.text("role","drawbar"),P.flag("compactCylinder",false)];
	public static function recipeType():ComponentType {
		if (recipe == null) recipe = new ComponentType("machinekit.milling.power-drawbar-study-part",parameters(),v -> new MillPowerDrawbarPart(v));
		return recipe;
	}
	public function new(?values:ComponentValues) {
		var v = P.resolve(parameters(),values);
		if (roles.indexOf(v.token("role")) < 0) throw "Unknown drawbar study part";
		super('BT30-STUDY-${v.token("role")}', 'BT30 ${v.token("role")} — design assumption',v.token("role")=="cylinder"?"aluminium 6061":"steel",false,v);
		addConnector("mount",Mount,Solids.axial(0,0,0));
		if (v.token("role") == "holder") addConnector("tool",Axis,Solids.axial(0,0,-35));
	}
	override public function componentType():Null<ComponentType> return recipeType();
	override public function hasGeometry():Bool return true;
	static function ring(r:Float,b:Float,lo:Float,hi:Float):Part return Solids.cut(Part.cylinderSpan(r,lo,hi),[Part.cylinderSpan(b,lo-1,hi+1)]);
	override public function geometry(detail:ComponentDetail = Preview):Part {
		var role=values().token("role");
		if (values().boolean("compactCylinder") && ["reactionFrame","cylinder","piston","pistonRod"].indexOf(role)>=0)
			return compactGeometry(role);
		return switch(role) {
		case "drawbar": Solids.union([Part.cylinderSpan(5,62,285),Part.cylinderSpan(11,222,227),Part.cylinderSpan(7,68,71)]);
		case "springs":
			var rows = []; for (i in 0...16) rows.push(ring(12.5,5.5,181+i*2.6,183+i*2.6)); Solids.union(rows);
		case "gripper": Solids.union([ring(9,4.2,48,51),ring(9,5.5,51,68)]);
		case "pullStud": Solids.union([Part.cylinderSpan(4,40,51),Part.cylinderSpan(5.4,51,55)]);
		case "holder":
			var taper = Part.revolve([{r:0.0,z:0.0},{r:15.875,z:0.0},{r:10.042,z:40.0},{r:0.0,z:40.0}]);
			Solids.cut(Solids.union([taper,Part.cylinderSpan(23,-10,0),Part.cylinderSpan(17,-35,-10)]),[Part.cylinderSpan(3,-36,-5)]);
		case "reactionFrame":
			var rows = [ring(70,22,242,250),Solids.cut(Part.box(140,120,8).translated(new Vector(0,0,322)),[Part.cylinderSpan(12,321,331)])];
			for (x in [-60.0,60.0]) for (y in [-35.0,35.0]) rows.push(Part.cylinderSpan(6,249,323,x,y));
			var cuts=[];
			for (x in [-50.0,50.0]) for (y in [-50.0,50.0]) cuts.push(Part.cylinderSpan(4.2,321,331,x,y));
			for (p in [[-37.5,0.0],[37.5,0.0],[0.0,-37.5],[0.0,37.5]]) {
				cuts.push(Part.cylinderSpan(2.75,241,251,p[0],p[1])); cuts.push(Part.cylinderSpan(5,244.6,251,p[0],p[1]));
			}
			Solids.cut(Solids.union(rows),cuts);
		case "cylinder":
			var rows=[Part.box(120,120,8).translated(new Vector(0,0,330)),Part.box(120,120,8).translated(new Vector(0,0,422)),ring(55,50,338,422)];
			for (x in [-45.0,45.0]) for (y in [-45.0,45.0]) rows.push(Part.cylinderSpan(3,337,423,x,y));
			var cuts=[Part.cylinderSpan(10.5,329,431)];
			for (x in [-50.0,50.0]) for (y in [-50.0,50.0]) cuts.push(Part.cylinderSpan(3.4,329,338,x,y));
			for (z in [345.0,415.0]) cuts.push(Part.cylinderAlongY(3,-56,-49,0,z));
			Solids.cut(Solids.union(rows),cuts);
		case "piston": ring(49.5,10,378,386);
		case "pistonRod": Part.cylinderSpan(10,287,331);
		case "rotor":
			var blank=Solids.union([Part.cylinderSpan(30,0,39),Part.cylinderSpan(25,38,242)]);
			var taper=Part.revolve([{r:0.0,z:-1.0},{r:16.041,z:-1.0},{r:9.479,z:44.0},{r:0.0,z:44.0}]);
			var tube=Solids.cut(blank,[taper,Part.cylinderSpan(13,43,243)]);
			Solids.union([tube,ring(25,5.5,178,181),ring(25,5.5,238,242),ring(10,5.5,242,275)]);
		default: throw "Unknown drawbar role";
		};
	}
	function compactGeometry(role:String):Part {
		var b=MillDrawbarLayout.CYLINDER_BOTTOM, top=b+MillDrawbarLayout.HEIGHT;
		var bridge=MillDrawbarLayout.BRIDGE_BOTTOM, piston=MillDrawbarLayout.PISTON_BOTTOM;
		var raw=switch(role) {
			case "reactionFrame":
				var rows=[ring(70,22,242,250),Part.box(140,100,MillDrawbarLayout.BRIDGE_THICKNESS).translated(new Vector(0,0,bridge))];
				for (x in [-60.0,60.0]) for (y in [-35.0,35.0]) rows.push(Part.cylinderSpan(6,249,bridge+1,x,y));
				var cuts=[Part.cylinderSpan(12,bridge-1,b+1)];
				for (x in [-MillDrawbarLayout.MOUNT_X,MillDrawbarLayout.MOUNT_X])
					for (y in [-MillDrawbarLayout.MOUNT_Y,MillDrawbarLayout.MOUNT_Y]) cuts.push(Part.cylinderSpan(3.3,bridge-1,b+1,x,y));
				for (p in [[-37.5,0.0],[37.5,0.0],[0.0,-37.5],[0.0,37.5]]) {
					cuts.push(Part.cylinderSpan(2.75,241,251,p[0],p[1])); cuts.push(Part.cylinderSpan(5,244.6,251,p[0],p[1]));
				}
				Solids.cut(Solids.union(rows),cuts);
			case "cylinder":
				var rows=[Part.box(MillDrawbarLayout.WIDTH,MillDrawbarLayout.WIDTH,MillDrawbarLayout.CAP).translated(new Vector(0,0,b)),
					Part.box(MillDrawbarLayout.WIDTH,MillDrawbarLayout.WIDTH,MillDrawbarLayout.CAP).translated(new Vector(0,0,top-MillDrawbarLayout.CAP)),
					ring(MillDrawbarLayout.BORE/2+5,MillDrawbarLayout.BORE/2,b+MillDrawbarLayout.CAP,top-MillDrawbarLayout.CAP)];
				for (x in [-34.0,34.0]) for (y in [-34.0,34.0]) rows.push(Part.cylinderSpan(3,b+MillDrawbarLayout.CAP-1,top-MillDrawbarLayout.CAP+1,x,y));
				var cuts=[Part.cylinderSpan(10.5,b-1,b+MillDrawbarLayout.CAP+1)];
				for (x in [-MillDrawbarLayout.MOUNT_X,MillDrawbarLayout.MOUNT_X])
					for (y in [-MillDrawbarLayout.MOUNT_Y,MillDrawbarLayout.MOUNT_Y]) cuts.push(Part.cylinderSpan(2.5,b-1,b+MillDrawbarLayout.CAP+1,x,y));
				for (z in [b+11,top-11]) cuts.push(Part.cylinderAlongY(3,-46,-39,0,z));
				Solids.cut(Solids.union(rows),cuts);
			case "piston": ring(MillDrawbarLayout.BORE/2-0.5,10,piston,piston+MillDrawbarLayout.PISTON_THICKNESS);
			case "pistonRod": Part.cylinderSpan(10,MillDrawbarLayout.ROD_TIP,piston+MillDrawbarLayout.PISTON_THICKNESS);
			default: throw "Unknown compact release role";
		};
		try { var result=raw.refined(); raw.close(); return result; } catch(e:Dynamic) { raw.close(); throw e; }
	}
}
