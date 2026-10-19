package machinekit.milling;

import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import cadkit.modeling.Sketch;
import cadkit.modeling.Plane;
import machinekit.component.*;
import machinekit.component.ComponentRecipeSupport as P;
import machinekit.motion.ServoMotor;
import machinekit.motion.LeadshineServo;
import machinekit.transmission.TimingBelt;
import machinekit.transmission.TimingPulley;

/** Editable drive selection carried by the actual mounting plate recipe.
 * Geometry is a redesign study, not a supplier-qualified mount or enclosure.
 */
class MillSpindleDrivePart extends MachineComponent {
	static var recipe:Null<ComponentType>;
	static var guardRecipe:Null<ComponentType>;
	public static function parameters(role:String="carrier"):Array<ComponentParameter> return [
		P.choice("role",[role],role),
		P.choice("motor",["ASSUMED-BT30-MOTOR-2200W"].concat(LeadshineServo.catalog().designations()),"ELM2M-2900LD180FT-H"),
		P.count("motorTeeth",96),P.count("spindleTeeth",18),P.length("centres",180),
		P.length("faceZ",330),P.length("mountBottom",140)];
	public static function recipeType(role:String="carrier"):ComponentType {
		if(role=="guard") {
			if(guardRecipe==null)guardRecipe=new ComponentType("machinekit.milling.spindle-drive-study-guard",parameters("guard"),v->new MillSpindleDrivePart(v));
			return guardRecipe;
		}
		if (recipe==null) recipe=new ComponentType("machinekit.milling.spindle-drive-study-part",parameters(),v->new MillSpindleDrivePart(v));
		return recipe;
	}
	public function new(?v:ComponentValues) {
		var role=v!=null && v.get("role")!=null ? v.token("role") : "carrier";
		if(role!="carrier" && role!="guard")throw "Unknown drive part role";
		v=P.resolve(parameters(role),v);
		if (v.integer("motorTeeth")<8 || v.integer("motorTeeth")>160 || v.integer("spindleTeeth")<8 || v.integer("spindleTeeth")>160)
			throw "Drive study pulleys require 8–160 teeth";
		if (v.number("centres")<100 || v.number("centres")>400 || v.number("faceZ")-v.number("mountBottom")<42)
			throw "Drive study needs 100–400 mm centres and bracing space below its plate";
		super("MILL-SPINDLE-DRIVE-"+v.token("role"),"Selectable spindle drive "+v.token("role"),"steel",false,v);
		// Validate tooth counts against actual shaft bores, not just belt pitch.
		motorPulley(); spindlePulley();
		addConnector("mount",Mount,Solids.axial(0,0,0));
	}
	public function motor():ServoMotor return parameterToken("motor")=="ASSUMED-BT30-MOTOR-2200W"
		? new Bt30SpindleMotor() : LeadshineServo.model(parameterToken("motor"));
	public function motorTeeth():Int return parameterCount("motorTeeth");
	public function spindleTeeth():Int return parameterCount("spindleTeeth");
	public function speedRatio():Float return motorTeeth()/spindleTeeth();
	public function face():Float return parameterNumber("faceZ");
	public function motorFace():Float {
		var m=motor(); return Std.isOfType(m,LeadshineServo) ? cast(m,LeadshineServo).spec.face : 110;
	}
	public function flangeThickness():Float {
		var m=motor(); return Std.isOfType(m,LeadshineServo) ? cast(m,LeadshineServo).spec.flangeThickness : 10;
	}
	public function pilotDiameter():Float {
		var m=motor(); return Std.isOfType(m,LeadshineServo) ? cast(m,LeadshineServo).spec.pilotDiameter : 40;
	}
	public function pilotHeight():Float {
		var m=motor(); return Std.isOfType(m,LeadshineServo) ? cast(m,LeadshineServo).spec.pilotHeight : 2;
	}
	public function mountingHoles():Array<{x:Float,y:Float}> {
		var m=motor(); return Std.isOfType(m,LeadshineServo) ? cast(m,LeadshineServo).boltPattern()
			: [{x:-30,y:-30},{x:30,y:-30},{x:30,y:30},{x:-30,y:30}];
	}
	public function holeDiameter():Float {
		var m=motor(); return Std.isOfType(m,LeadshineServo) ? cast(m,LeadshineServo).spec.holeDiameter : 5.5;
	}
	public function screwSize():String {
		var d=holeDiameter(); return d<5?"M4":d<6?"M5":d<7?"M6":d<10?"M8":"M12";
	}
	public function motorPulley():TimingPulley return new TimingPulley(HTD5M,motorTeeth(),motor().rating.shaftDiameter,18);
	public function spindlePulley():TimingPulley return new TimingPulley(HTD5M,spindleTeeth(),20,18);
	/** Solve the exact unequal-pulley pitch path for the nearest whole-tooth belt. */
	public function centres():Float {
		var nominal=parameterNumber("centres");
		var target=Math.round(TimingBelt.twoPulley(HTD5M,spindleTeeth(),motorTeeth(),nominal,15).length/5)*5;
		var lo=nominal-5,hi=nominal+5;
		for (_ in 0...40) {
			var mid=(lo+hi)/2;
			if (TimingBelt.twoPulley(HTD5M,spindleTeeth(),motorTeeth(),mid,15).length<target) lo=mid; else hi=mid;
		}
		return (lo+hi)/2;
	}
	public function guardRadius():Float {
		var screw=machinekit.standard.SocketHeadCapScrew.metric(screwSize(),20),boltRadius=0.0;
		for(p in mountingHoles())boltRadius=Math.max(boltRadius,Math.sqrt(p.x*p.x+p.y*p.y));
		return Math.max(Math.max(96,Math.max(motorPulley().pitchDiameter,spindlePulley().pitchDiameter)/2+20),boltRadius+screw.spec.headDiameter/2+6);
	}
	public function guardBoltX():Float return guardRadius()+6;
	public function plateWidth():Float return Math.max(Math.max(188,motorFace()+24),2*(guardBoltX()+8));
	public function plateRear():Float return Math.max(180,parameterNumber("centres")+motorFace()/2+10);
	static function box(w:Float,d:Float,z0:Float,z1:Float,x:Float=0,y:Float=0):Part
		return Part.prism([new Vector(x-w/2,y-d/2),new Vector(x+w/2,y-d/2),new Vector(x+w/2,y+d/2),new Vector(x-w/2,y+d/2)],z0,z1);
	static function slot(radius:Float,y:Float,z0:Float,z1:Float,x:Float=0):Part {
		var profile=Sketch.slot(10+2*radius,2*radius,new Plane(new Vector(x,y,z0),Vector.Y(),Vector.Z()));
		try {var result=profile.extrude(z1-z0);profile.close();return result;}catch(e:Dynamic){profile.close();throw e;}
	}
	override public function componentType():Null<ComponentType> return recipeType(parameterToken("role"));
	override public function hasGeometry():Bool return true;
	override public function geometry(detail:ComponentDetail=Preview):Part {
		var owned:Array<Part> =[];
		return Solids.building(owned,tracked->{
			function keep(p:Part):Part {tracked.push(p);return p;}
			var pieces:Array<Part> =[],cuts:Array<Part> =[],front=34.0,rear=plateRear(),width=plateWidth();
			var bottom=parameterNumber("mountBottom"),plate=face()-12,nominal=parameterNumber("centres");
			if (parameterToken("role")=="carrier") {
				pieces.push(keep(box(width,rear-front,plate,face(),0,(front+rear)/2)));
				// Keep the release bridge and cylinder separate from the drive carrier.
				cuts.push(keep(box(144,104,plate-1,face()+1)));
				for (side in [-1,1]) {
					pieces.push(keep(box(32,38,bottom,bottom+10,side*72,119)));
					pieces.push(keep(box(12,38,bottom,plate+1,side*82,119)));
					for(y in [110.0,130.0]) {
						cuts.push(keep(Part.cylinderSpan(3.3,bottom-1,bottom+11,side*68,y)));
						cuts.push(keep(Part.cylinderSpan(5.5,plate-1,face()+1,side*68,y)));
					}
					cuts.push(keep(Part.cylinderSpan(1.65,plate-1,plate+8,side*guardBoltX(),112)));
					cuts.push(keep(Part.cylinderSpan(8,plate-1,face()+1,side*60,35)));
				}
				cuts.push(keep(slot((motor().rating.shaftDiameter+2)/2,nominal,plate-1,face()+1)));
				cuts.push(keep(slot((pilotDiameter()+0.2)/2,nominal,face()-pilotHeight()-0.2,face()+1)));
				var screw=machinekit.standard.SocketHeadCapScrew.metric(screwSize(),20);
				for (p in mountingHoles()) {
					cuts.push(keep(slot(holeDiameter()/2,nominal+p.y,plate-1,face()+1,-p.x)));
					// Keep socket heads clear of the support webs below the plate.
					cuts.push(keep(slot((screw.spec.headDiameter+1)/2,nominal+p.y,plate-screw.spec.headHeight-0.5,plate,-p.x)));
				}
			} else {
				var radius=guardRadius(),centre=centres(),top=face()-10;
				function capsule(r:Float,z:Float,h:Float):Part {
					var sketch=Sketch.slot(centre+2*r,2*r,new Plane(new Vector(0,centre/2,z),Vector.Y(),Vector.Z()));
					try {var p=sketch.extrude(h);sketch.close();return p;}catch(e:Dynamic){sketch.close();throw e;}
				}
				var outer=keep(capsule(radius,250,top-250)),inner=keep(capsule(radius-2,249,top-248));
				pieces.push(keep(Solids.cut(outer,[inner])));pieces.push(keep(capsule(radius,top-2,2)));
				cuts.push(keep(box(width+2,rear-front,top-3,top+1,0,(front+rear)/2)));
				cuts.push(keep(Part.cylinderSpan(12,249,top+1)));
				if(top>MillDrawbarLayout.BRIDGE_BOTTOM-8)
					cuts.push(keep(box(144,104,top-3,top+1)));
				for(x in [-60.0,60.0])for(y in [-35.0,35.0])cuts.push(keep(Part.cylinderSpan(8,249,top+1,x,y)));
				for(side in [-1,1]) {
					pieces.push(keep(box(20,20,top-4,top-2,side*(radius+4),112)));
					cuts.push(keep(Part.cylinderSpan(2.25,top-5,top+1,side*guardBoltX(),112)));
				}
			}
			var joined=keep(Solids.union(pieces));return Solids.cut(joined,cuts);
		});
	}
}
