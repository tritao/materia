package machinekit.hobbing;

import machinekit.assembly.MachineAssembly;
import machinekit.component.*;
import machinekit.motion.*;
import materia.assembly.AssemblyDefinition.AssemblyJointType;

/** C-axis package. Joint units rad/rad/s/N m. The clamp is hardware, not active motion coupling. */
class RotaryTable extends MachineAssembly implements machinekit.motion.PrecisionRotaryAxis {
  public final motor:TorqueMotor;
  public final bearing:CrossedRollerBearing;
  public final encoder:RotaryEncoderRing;
  public final clamp:RotaryClamp;
  public function new(?motor:TorqueMotor,?bearing:CrossedRollerBearing,?encoder:RotaryEncoderRing,?clamp:RotaryClamp) {
    super();this.motor=motor==null?new TorqueMotor():motor;this.bearing=bearing==null?new CrossedRollerBearing():bearing;
    this.encoder=encoder==null?new RotaryEncoderRing():encoder;this.clamp=clamp==null?new RotaryClamp():clamp;
    var large=this.bearing.parameterNumber("outside")==220;
    if(this.motor.parameterNumber("outside")>150||this.motor.parameterNumber("height")>45||(!large&&this.bearing.parameterNumber("outside")!=120)||this.bearing.parameterNumber("height")!=(large?20:16)
      ||this.encoder.parameterNumber("outside")>130||this.encoder.parameterNumber("height")>8||this.clamp.parameterNumber("height")>10)
      throw "rotary table reference seats do not fit installed parts";
    addComponent("housing",new TableHousing(large?new ComponentValues().setNumber("outside",270).setNumber("bearingSeat",220).setNumber("bearingWidth",20):null));
    attach("motor",this.motor,0);attach("bearing",this.bearing,45);attach("clamp",this.clamp,80);attach("encoder",this.encoder,90);
    addComponent("platen",new TablePlaten(large?new ComponentValues().setNumber("outside",240).setNumber("bore",40).setNumber("faceBoltPcd",160).setNumber("stemDiameter",200).setNumber("stemBore",160).setNumber("stemHeight",33):new ComponentValues().setNumber("bore",40).setNumber("faceBoltPcd",120)));addMemberConnector("housing","platenSeat",Solids.axial(0,0,98));
    addMateOnAxis("C",Continuous,"housing","platenSeat","platen","axis",{x:0,y:1,z:0},0,
      {lower:null,upper:null,velocity:this.motor.maxSpeed,effort:this.motor.peakTorque,acceleration:10,
       assumptions:(this.motor.ratingsBasis==Assumed?[{quantity:"speed limit",label:"assumed direct motor rating"},{quantity:"effort limit",label:"assumed peak motor torque"}]:[]).concat([{quantity:"acceleration limit",label:"assumed table positioning limit 10 rad/s²"}])});
    addEncoder("table-angle","C","encoder");
    exposeConnector("mount","housing","mount");exposeConnector("work","platen","top");
  }
  function attach(id:String,part:MachineComponent,z:Float):Void {
    addComponent(id,part);addMemberConnector("housing",id+"Seat",Solids.axial(0,0,z));
    addMate(id+"-mount",Fixed,"housing",id+"Seat",id,"mount");
  }
  public function assembly():MachineAssembly return this;
  public function specification():machinekit.motion.RotaryAxisSpec {
    var v=machinekit.component.ComponentRecipeSupport.resolve(machinekit.motion.RotaryAxisSpec.parameters());
    // The existing built assembly retains its own assumed engineering values.
    for(row in machinekit.motion.RotaryAxisSpec.defaults())v.setToken(row.name+"Basis","Assumed").setToken(row.name+"Reason","Built-up reference assembly; constituent engineering assumptions, not catalogue qualification.");
    v.setNumber("height",118).setNumber("faceDiameter",components().filter(m->m.id=="platen")[0].component.parameterNumber("outside")).setNumber("bore",40).setNumber("housingDiameter",components().filter(m->m.id=="housing")[0].component.parameterNumber("outside"))
      .setNumber("centeringDiameter",0).setNumber("facePcd",components().filter(m->m.id=="platen")[0].component.parameterNumber("faceBoltPcd"))
      .setNumber("radialRunout",bearing.radialRunout).setNumber("axialRunout",bearing.axialRunout)
      .setNumber("momentStiffness",bearing.parameterNumber("momentStiffness")).setNumber("pivotBelowFace",65)
      .setNumber("absoluteAccuracy",encoder.angularAccuracy).setNumber("encoderCounts",encoder.parameterCount("countsPerRevolution"))
      .setNumber("continuousTorque",motor.continuousTorque).setNumber("peakTorque",motor.peakTorque)
      .setNumber("ratedSpeed",motor.ratedSpeed).setNumber("maxSpeed",motor.maxSpeed).setNumber("rotorInertia",motor.rotorInertia)
      .setNumber("coggingTorque",motor.coggingTorque).setNumber("sealDrag",0).setNumber("sealDragRipple",0)
      .setNumber("torqueConstant",motor.parameterNumber("torqueConstant")).setNumber("backEmfConstant",motor.parameterNumber("backEmfConstant"))
      .setNumber("mass",massProperties().mass)
      .setNumber("rotatingMass",components().filter(m->m.id=="platen")[0].component.massProperties().mass)
      .setNumber("continuousCurrent",motor.continuousTorque/motor.parameterNumber("torqueConstant"))
      .setNumber("peakCurrent",motor.peakTorque/motor.parameterNumber("torqueConstant"));
    v.setToken("massBasis","Derived").setToken("massReason","CAD volumes x declared densities; constituent densities and geometry carry their own assumptions.")
      .setToken("rotatingMassBasis","Derived").setToken("rotatingMassReason","Built platen CAD mass, not purchased internals.");
    return new machinekit.motion.RotaryAxisSpec(v,"motor","encoder","bearing");
  }
  public function jointTorque(motorTorque:Float):Float return motor.jointTorque(motorTorque);
}
