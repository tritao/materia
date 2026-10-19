package machinekit.milling;

import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.component.*;
import machinekit.component.ComponentRecipeSupport as P;

/** Braced motor carrier, with a centered two-support option. The legacy offset
 * recipe remains rebuildable; motor interfaces are explicit study assumptions. */
class MillSpindleMotorMount extends MachineComponent {
	static var recipe:Null<ComponentType>;
	public static function parameters():Array<ComponentParameter> return [P.length("height",80),P.flag("centered",false)];
	public static function recipeType():ComponentType {
		if (recipe == null) recipe = new ComponentType("machinekit.milling.spindle-motor-mount",parameters(),v -> new MillSpindleMotorMount(v)); return recipe;
	}
	public function new(?v:ComponentValues) {
		v = P.resolve(parameters(),v); if (v.number("height") < 30) throw "Motor carrier needs bracing space";
		super("MILL-SPINDLE-MOTOR-CARRIER","Braced adjustable spindle motor carrier","steel",false,v);
		addConnector("mount",Mount,Solids.axial(0,0,0));
	}
	override public function componentType():Null<ComponentType> return recipeType();
	override public function hasGeometry():Bool return true;
	override public function geometry(detail:ComponentDetail = Preview):Part {
		if (values().boolean("centered")) return centeredGeometry();
		var h = parameterNumber("height");
		var rows = [Part.box(188,100,12).translated(new Vector(139,78,h)),Part.box(30,70,10).translated(new Vector(60,68,0))];
		for (y in [44.0,92.0]) {
			rows.push(Part.box(12,12,h+1).translated(new Vector(75,y,0)));
			var rib = Part.prism([new Vector(75,8),new Vector(75,h+1),new Vector(85,h+1)],-6,6);
			rows.push(machinekit.assembly.PosedParts.moved(rib,materia.assembly.AssemblyFrames.fromRotationMatrix(0,y,0,[1,0,0, 0,0,-1, 0,1,0]))); rib.close();
		}
		rows.push(Part.box(12,20,16).translated(new Vector(225,78,h+12)));
		var tools = [Part.cylinderSpan(8,h-1,h+13,60,35)];
		for (r in [12.0,20.0]) {
			var z=r==12?h-1:h+10;
			tools.push(Solids.union([Part.cylinderSpan(r,z,h+13,142,78),Part.cylinderSpan(r,z,h+13,158,78),Part.box(16,2*r,h+13-z).translated(new Vector(150,78,z))]));
		}
		var tap=Part.cylinderSpan(2.5,218,233);
		tools.push(machinekit.assembly.PosedParts.moved(tap,materia.assembly.AssemblyFrames.fromRotationMatrix(0,78,h+18,[0,0,1, 1,0,0, 0,1,0]))); tap.close();
		for (x in [120.0,180.0]) for (y in [48.0,108.0]) tools.push(Part.box(21,5.5,14).translated(new Vector(x,y,h-1)));
		for (y in [58.0,88.0]) tools.push(Part.cylinderSpan(3.3,-1,11,55,y));
		for (p in [[94.0,108.0],[165.0,121.0]]) tools.push(Part.cylinderSpan(1.65,h-1,h+8,p[0],p[1]));
		return Solids.cut(Solids.union(rows),tools);
	}
	function centeredGeometry():Part {
		var h=parameterNumber("height"), front=MillSpindleMotorLayout.PLATE_FRONT, rear=MillSpindleMotorLayout.PLATE_REAR;
		var pieces=[Part.box(188,rear-front,12).translated(new Vector(0,(front+rear)/2,h))];
		var tools:Array<Part> = [];
		for (side in [-1,1]) {
			pieces.push(Part.box(32,38,10).translated(new Vector(side*72,119,0)));
			pieces.push(Part.box(12,38,h+1).translated(new Vector(side*82,119,0)));
			for (y in [MillSpindleMotorLayout.FOOT_BOLT_FRONT,MillSpindleMotorLayout.FOOT_BOLT_REAR]) {
				tools.push(Part.cylinderSpan(3.3,-1,11,side*MillSpindleMotorLayout.FOOT_BOLT_X,y));
				tools.push(Part.cylinderSpan(5.5,h-1,h+13,side*MillSpindleMotorLayout.FOOT_BOLT_X,y));
			}
			tools.push(Part.cylinderSpan(8,h-1,h+13,side*60,35));
		}
		for (r in [12.0,20.0]) {
			var z=r==12?h-1:h+10;
			tools.push(Solids.union([Part.cylinderSpan(r,z,h+13,0,115),Part.cylinderSpan(r,z,h+13,0,125),
				Part.box(2*r,10,h+13-z).translated(new Vector(0,120,z))]));
		}
		for (x in [-30.0,30.0]) for (y in [90.0,150.0]) {
			tools.push(Solids.union([Part.cylinderSpan(2.75,h-1,h+13,x,y-5),Part.cylinderSpan(2.75,h-1,h+13,x,y+5),
				Part.box(5.5,10,14).translated(new Vector(x,y,h-1))]));
			tools.push(Solids.union([Part.cylinderSpan(4.75,h-1,h+6,x,y-5),Part.cylinderSpan(4.75,h-1,h+6,x,y+5),
				Part.box(9.5,10,7).translated(new Vector(x,y,h-1))]));
		}
		pieces.push(Part.box(20,12,16).translated(new Vector(MillSpindleMotorLayout.TENSION_X,42,h+12)));
		tools.push(Part.cylinderAlongY(2.5,35,49,MillSpindleMotorLayout.TENSION_X,h+18));
		for (x in [-MillSpindleMotorLayout.GUARD_BOLT_X,MillSpindleMotorLayout.GUARD_BOLT_X])
			tools.push(Part.cylinderSpan(1.65,h-1,h+8,x,MillSpindleMotorLayout.GUARD_BOLT_Y));
		var raw=Solids.cut(Solids.union(pieces),tools);
		try { var result=raw.refined(); raw.close(); return result; } catch(e:Dynamic) { raw.close(); throw e; }
	}

}
