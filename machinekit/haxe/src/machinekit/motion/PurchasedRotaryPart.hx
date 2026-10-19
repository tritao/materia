package machinekit.motion;
import machinekit.component.*;
import machinekit.component.ComponentRecipeSupport as P;
import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import cadkit.InertiaTensor;

/** One purchased unit, split into fixed/rotating envelopes solely for kinematics.
    The rotor is included in the unit's BOM; no internal bearing/motor construction is invented. */
class PurchasedRotaryPart extends MachineComponent implements MotorDrive implements EncoderPart {
  public static inline final PURCHASE_ID="PURCHASED-DD-200-210";
  public final axis:RotaryAxisSpec;
  public final variant:String;
  static var recipe:Null<ComponentType>;
  public static function parameters():Array<ComponentParameter>return RotaryAxisSpec.parameters().concat([P.choice("variant",["body","rotor","adapter"],"body"),P.length("adapterHeight",92),P.length("adapterOutside",270)]);
  public static function recipeType():ComponentType {if(recipe==null)recipe=new ComponentType("machinekit.motion.purchased-rotary-part",parameters(),v->new PurchasedRotaryPart(v),true);return recipe;}
  public function new(?overrides:ComponentValues){var v=P.resolve(parameters(),overrides);variant=v.token("variant");axis=new RotaryAxisSpec(v,"housing","housing","housing");
    super(variant=="body"?PURCHASE_ID:variant=="rotor"?"PURCHASED-DD-ROTOR":"ROTARY-AXIS-SEAT-ADAPTER",variant=="body"?"Purchased direct-drive table; specification in STRAIN_WAVE_RESEARCH.md; mounting interface provisional":variant=="rotor"?"Rotating envelope included in purchased table":"Ground steel table seat adapter, provisional mounting pattern","steel",false,v);
    var height=variant=="body"?axis.number("height")-axis.number("faceThickness"):variant=="rotor"?axis.number("faceThickness"):v.number("adapterHeight");
    if(height<=0||axis.number("mass")<=axis.number("rotatingMass"))throw "invalid purchased rotary mass/envelope";
    addConnector("mount",Mount,Solids.axial(0,0,0));addConnector("axis",Axis,Solids.axial(0,0,0));addConnector("top",Mount,Solids.axial(0,0,height));
    if(variant=="rotor"){
      addFacet(new IncludedInPurchase(PURCHASE_ID));var j=axis.number("rotorInertia")*1e6;
      declareMass(axis.number("rotatingMass"),new Vector(0,0,height/2),new InertiaTensor(j/2,0,0,j/2,0,j));
    }else if(variant=="body"){
      var mass=axis.number("mass")-axis.number("rotatingMass"),r=axis.number("housingDiameter")/2,b=axis.number("bore")/2;
      var iz=mass*(r*r+b*b)/2,ix=mass*(3*(r*r+b*b)+height*height)/12;
      declareMass(mass,new Vector(0,0,height/2),new InertiaTensor(ix,0,0,ix,0,iz));
      addPort({name:"air",kind:Pneumatic,role:Consumer,iface:Unspecified,required:true});
      addPort({name:"encoderCable",kind:Signal,role:Supply,iface:Unspecified,required:true});
      addPort({name:"driverAc",kind:ElectricalPower,role:Consumer,iface:Unspecified,required:true});
      addPort({name:"clampRelease",kind:Signal,role:Consumer,iface:Unspecified,required:true});
      addPort({name:"clampReleased",kind:Signal,role:Supply,iface:Unspecified,required:false});
      addPort({name:"rotaryUnion",kind:Hydraulic,role:Passive,iface:Unspecified,required:false});
      addConnector("airService",Mount,Solids.axial(0,-axis.number("barrelDiameter")/2,height/3));
      addConnector("cableService",Mount,Solids.axial(axis.number("barrelDiameter")/2,0,height/3));
      addConnector("unionAccess",Mount,Solids.axial(0,0,0));
    }
  }
  override public function componentType():Null<ComponentType>return recipeType();
  override public function hasGeometry():Bool return true;
  override public function geometry(detail:ComponentDetail=Preview):Part {
    var h=variant=="body"?axis.number("height")-axis.number("faceThickness"):variant=="rotor"?axis.number("faceThickness"):parameterNumber("adapterHeight");
    var od=variant=="body"?axis.number("housingDiameter"):variant=="rotor"?axis.number("faceDiameter"):parameterNumber("adapterOutside");
    var bore=axis.number("bore");
    var cuts=[Part.cylinderSpan(bore/2,-1,h+1)];
    if(variant=="rotor"){
      cuts.push(Part.cylinderSpan(axis.number("centeringDiameter")/2,h-3,h+1));
      for(i in 0...6){var a=(i+0.5)*Math.PI/3,r=axis.number("facePcd")/2;cuts.push(Part.cylinderSpan(3.4,0,h+1,r*Math.cos(a),r*Math.sin(a)));}
    }
    if(variant=="body")for(i in 0...6){var a=(i+0.5)*Math.PI/3,r=axis.number("flangePcd")/2;cuts.push(Part.cylinderSpan(5.5,-1,15,r*Math.cos(a),r*Math.sin(a)));}
    var envelope=variant=="body"?Solids.union([Part.cylinderSpan(od/2,0,axis.number("flangeThickness")),Part.cylinderSpan(axis.number("barrelDiameter")/2,axis.number("flangeThickness"),h)]):Part.cylinderSpan(od/2,0,h);
    return Solids.cut(envelope,cuts);
  }
  override public function motorDrive():Null<MotorDrive>return variant=="body"?this:null;
  public function electricalRating():Null<ServoElectricalRating> return {torqueConstant:axis.number("torqueConstant"),backEmfConstant:axis.number("backEmfConstant"),phaseResistance:axis.number("phaseResistance"),mechanicalEfficiency:0.95};
  public function actuator(id:String,joint:String,volts:Float,margin:Float,?current:Float):materia.assembly.AssemblyDefinition.AssemblyActuator {
    return {id:id,joint:joint,drive:"servo",maxEffort:axis.number("peakTorque"),maxRate:axis.number("maxSpeed"),
      rotorInertia:0,ratedTorque:axis.number("continuousTorque"),peakTorque:axis.number("peakTorque"),ratedSpeed:axis.number("ratedSpeed"),maxSpeed:axis.number("maxSpeed"),
      torqueSpeed:[0,axis.number("peakTorque"),axis.number("ratedSpeed"),axis.number("peakTorque"),axis.number("maxSpeed"),axis.number("continuousTorque")],
      assumed:["Electrical constants and driver input tolerance assumed; rotating inertia is in the rotor's declared physical tensor, not counted twice"]};
  }
  public function encoder(id:String,joint:String):materia.assembly.AssemblyDefinition.AssemblyEncoder return {id:id,joint:joint,kind:"absolute",counts:Std.int(axis.number("encoderCounts"))};
}
