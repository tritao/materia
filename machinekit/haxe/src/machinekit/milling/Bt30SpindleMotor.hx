package machinekit.milling;

import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.component.*;
import machinekit.motion.ServoMotor;

/** Assumed compact 2.2 kW / 8000 rpm motor, including its mounting flange.
 * This is not the much larger motor in the Automation Technology kit. */
class Bt30SpindleMotor extends ServoMotor {
	static var recipe:Null<ComponentType>;
	public static function recipeType():ComponentType {
		if (recipe==null) recipe=new ComponentType("machinekit.milling.bt30-spindle-motor-study",[],v->new Bt30SpindleMotor()); return recipe;
	}
	public function new() {
		var speed=8000*2*Math.PI/60,torque=2200/speed;
		super({designation:"ASSUMED-BT30-MOTOR-2200W",ratedTorque:torque,peakTorque:2*torque,ratedSpeed:speed,maxSpeed:speed,
			rotorInertia:0.002,encoderCounts:4096,bodyDiameter:90,bodyLength:160,shaftDiameter:20,shaftLength:30,
			electrical:ServoMotor.acWinding(torque,speed)},Assumed);
	}
	override public function componentType():Null<ComponentType> return recipeType();
	override public function values():ComponentValues return new ComponentValues().setToken("material",materialSpec());
	override public function geometry(detail:ComponentDetail = Preview):Part {
		var tools=[]; for (x in [-30.0,30.0]) for (y in [-30.0,30.0]) tools.push(Part.cylinderSpan(2.1,-8,1,x,y));
		return Solids.cut(Solids.union([super.geometry(detail),Part.box(110,110,10).translated(new Vector(0,0,-10)),
			Solids.cut(Part.cylinderSpan(20,0,2),[Part.cylinderSpan(10.5,-1,3)])]),tools);
	}
}
