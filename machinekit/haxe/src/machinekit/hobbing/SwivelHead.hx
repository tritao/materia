package machinekit.hobbing;

import machinekit.assembly.MachineAssembly;
import machinekit.component.*;
import machinekit.motion.TorqueMotor;
import materia.assembly.AssemblyDefinition.AssemblyJointType;

/** Assumed direct servo A axis, positioned then clamped. Clamp state is a later control concern. */
class SwivelHead extends MachineAssembly {
  public final motor:TorqueMotor;
  public final clamp:RotaryClamp;
  public final bearing:machinekit.motion.CrossedRollerBearing;
  public function new(?motor:TorqueMotor,?clamp:RotaryClamp,maxTilt:Float=Math.PI/4) {
    super();if(!Math.isFinite(maxTilt)||maxTilt<=0||maxTilt>Math.PI/4)throw "invalid swivel tilt range";this.motor=motor==null?new TorqueMotor(new ComponentValues().setNumber("ratedSpeed",Math.PI/12).setNumber("maxSpeed",Math.PI/6)):motor;
    this.clamp=clamp==null?new RotaryClamp():clamp;
    if(this.motor.n("outside")>150||this.motor.n("height")>45)throw "swivel motor exceeds reference housing seat";
    bearing=new machinekit.motion.CrossedRollerBearing();
    addComponent("housing",new SwivelHousing());attach("bearing",bearing,0);attach("motor",this.motor,16);attach("clamp",this.clamp,80);
    addComponent("head",new TablePlaten(new ComponentValues().setNumber("outside",180).setNumber("bore",100)));
    addMemberConnector("housing","swivel",Solids.axial(0,0,90));
    addMateOnAxis("A",Revolute,"housing","swivel","head","axis",{x:0,y:1,z:0},0,
      {lower:-maxTilt,upper:maxTilt,velocity:this.motor.maxSpeed,effort:this.motor.peakTorque,
       assumptions:this.motor.ratingsBasis==Assumed?[{quantity:"speed limit",label:"assumed direct motor rating"},{quantity:"effort limit",label:"assumed peak motor torque"}]:null});
    exposeConnector("mount","housing","mount");exposeConnector("head","head","top");
  }
  function attach(id:String,part:MachineComponent,z:Float):Void {
    addComponent(id,part);addMemberConnector("housing",id+"Seat",Solids.axial(0,0,z));addMate(id+"-mount",Fixed,"housing",id+"Seat",id,"mount");
  }
}
