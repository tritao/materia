package machinekit.hobbing;

import machinekit.assembly.MachineAssembly;
import machinekit.component.*;
import machinekit.motion.TorqueMotor;
import materia.assembly.AssemblyDefinition.AssemblyJointType;

/** Compact assumed direct-drive swivel: motor inside the crossed-roller ring,
    outer clamp in the same axial bay. Procurement and thermal ratings remain assumed. */
class SwivelHead extends MachineAssembly {
  public final motor:TorqueMotor;
  public final clamp:RotaryClamp;
  public final bearing:machinekit.motion.CrossedRollerBearing;
  public function new(?motor:TorqueMotor,?clamp:RotaryClamp,maxTilt:Float=Math.PI/4) {
    super();if(!Math.isFinite(maxTilt)||maxTilt<=0||maxTilt>Math.PI/4)throw "invalid swivel tilt range";this.motor=motor==null?new TorqueMotor(new ComponentValues().setNumber("height",20).setNumber("ratedSpeed",Math.PI/12).setNumber("maxSpeed",Math.PI/6)):motor;
    this.clamp=clamp==null?new RotaryClamp(new ComponentValues().setNumber("outside",200).setNumber("bore",180)):clamp;
    if(this.motor.parameterNumber("outside")>150||this.motor.parameterNumber("height")>20)throw "swivel motor exceeds reference housing seat";
    bearing=new machinekit.motion.CrossedRollerBearing(new ComponentValues().setNumber("outside",180).setNumber("bore",150));
    addComponent("housing",new SwivelHousing());attach("bearing",bearing,5);attach("motor",this.motor,5);attach("clamp",this.clamp,15);
    addComponent("head",new TablePlaten(new ComponentValues().setNumber("outside",200).setNumber("bore",100).setNumber("height",10)));
    addMemberConnector("housing","swivel",Solids.axial(0,0,25));
    addMateOnAxis("A",Revolute,"housing","swivel","head","axis",{x:0,y:1,z:0},0,
      {lower:-maxTilt,upper:maxTilt,velocity:this.motor.maxSpeed,effort:this.motor.peakTorque,acceleration:Math.PI/2,
       assumptions:(this.motor.ratingsBasis==Assumed?[{quantity:"speed limit",label:"assumed direct motor rating"},{quantity:"effort limit",label:"assumed peak motor torque"}]:[]).concat([{quantity:"acceleration limit",label:"assumed swivel positioning limit pi/2 rad/s²"}])});
    exposeConnector("mount","housing","mount");exposeConnector("head","head","top");
  }
  function attach(id:String,part:MachineComponent,z:Float):Void {
    addComponent(id,part);addMemberConnector("housing",id+"Seat",Solids.axial(0,0,z));addMate(id+"-mount",Fixed,"housing",id+"Seat",id,"mount");
  }
}
