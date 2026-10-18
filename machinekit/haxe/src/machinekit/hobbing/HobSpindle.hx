package machinekit.hobbing;

import machinekit.assembly.MachineAssembly;
import machinekit.component.*;
import machinekit.motion.*;
import materia.assembly.AssemblyDefinition.AssemblyJointType;

/** Direct B spindle, an assumed 1.5 kW at 4000 rpm / 8000 rpm maximum reference.
    Back-to-back bearing envelopes do not claim a catalogue preload or exact HSK taper. */
class HobSpindle extends MachineAssembly {
  public final motor:TorqueMotor;
  public final frontBearing:AngularContactBearing;
  public final rearBearing:AngularContactBearing;
  public final arbor:HobArbor;
  public final encoder:RotaryEncoderRing;
  public function new(?motor:TorqueMotor,?arbor:HobArbor) {
    super();var rated=4000*2*Math.PI/60;
    this.motor=motor==null?new TorqueMotor(new ComponentValues()
      .setNumber("outside",62).setNumber("bore",25).setNumber("height",100)
      .setNumber("continuousTorque",1500/rated).setNumber("peakTorque",3000/rated)
      .setNumber("ratedSpeed",rated).setNumber("maxSpeed",8000*2*Math.PI/60)
      .setNumber("rotorInertia",0.0002).setNumber("coggingTorque",0.003)
      .setInteger("poleCount",4).setInteger("coggingOrder",12)
      .setNumber("torqueConstant",0.3).setNumber("backEmfConstant",0.3/Math.sqrt(3)).setNumber("phaseResistance",0.15)):motor;
    this.arbor=arbor==null?new HobArbor():arbor;
    frontBearing=new AngularContactBearing();rearBearing=new AngularContactBearing();
    encoder=new RotaryEncoderRing(new ComponentValues().setNumber("outside",62).setNumber("bore",40).setNumber("height",6).setInteger("countsPerRevolution",1048576));
    if(this.motor.parameterNumber("outside")>62||this.motor.parameterNumber("height")>110||this.arbor.parameterNumber("outside")!=16)
      throw "spindle reference seats do not fit installed parts";
    addComponent("housing",new SpindleHousing());
    attach("motor",this.motor,40);attach("frontBearing",frontBearing,0);attach("encoder",encoder,150);
    addComponent("rearBearing",rearBearing);
    addMemberConnector("housing","rearBearingSeat",materia.assembly.AssemblyFrames.alongY(0,0,34,0,0,-1));
    addMate("rearBearing-mount",Fixed,"housing","rearBearingSeat","rearBearing","mount");
    addComponent("arbor",this.arbor);addMemberConnector("housing","gaugeLine",Solids.axial(0,0,180));
    addMateOnAxis("B",Continuous,"housing","gaugeLine","arbor","gaugeLine",{x:0,y:1,z:0},0,
      {lower:null,upper:null,velocity:this.motor.maxSpeed,effort:this.motor.peakTorque,
       assumptions:this.motor.ratingsBasis==Assumed?[{quantity:"speed limit",label:"assumed direct motor rating"},{quantity:"effort limit",label:"assumed peak motor torque"}]:null});
    addEncoder("hob-angle","B","encoder");
    exposeConnector("mount","housing","mount");exposeConnector("gaugeLine","arbor","gaugeLine");exposeConnector("hobCentre","arbor","hobCentre");
  }
  function attach(id:String,part:MachineComponent,z:Float):Void {
    addComponent(id,part);addMemberConnector("housing",id+"Seat",Solids.axial(0,0,z));
    addMate(id+"-mount",Fixed,"housing",id+"Seat",id,"mount");
  }
  public function interfaceName():String return "HSK-E25";
  public function jointTorque(motorTorque:Float):Float return motor.jointTorque(motorTorque);
}
