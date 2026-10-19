package machinekit.motion;
import machinekit.assembly.MachineAssembly;
import machinekit.component.ComponentValues;
import machinekit.component.Solids;
/** Purchased DD rotary axis with the same mount/work/C contract as a built table. */
class PurchasedRotaryTable extends MachineAssembly implements PrecisionRotaryAxis {
  public final body:PurchasedRotaryPart;
  public final rotor:PurchasedRotaryPart;
  public function new(?values:ComponentValues){super();body=new PurchasedRotaryPart(values);rotor=new PurchasedRotaryPart(body.values().copy().setToken("variant","rotor"));
    addComponent("housing",body);addComponent("platen",rotor);
    addMemberConnector("housing","rotation",Solids.axial(0,0,body.axis.number("height")-body.axis.number("faceThickness")));
    addMateOnAxis("C",Continuous,"housing","rotation","platen","axis",{x:0,y:1,z:0},0,
      {lower:null,upper:null,velocity:body.axis.number("maxSpeed"),effort:body.axis.number("peakTorque"),acceleration:10,
      assumptions:[{quantity:"acceleration limit",label:"assumed positioning acceleration 10 rad/s²"}]});
    exposeConnector("mount","housing","mount");exposeConnector("work","platen","top");
    for(port in ["air","encoderCable","driverAc","clampRelease","clampReleased","rotaryUnion"])exposePort(port,"housing",port);
  }
  public function assembly():MachineAssembly return this;
  public function specification():RotaryAxisSpec return body.axis;
}
