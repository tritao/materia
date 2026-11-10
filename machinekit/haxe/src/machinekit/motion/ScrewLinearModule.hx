package machinekit.motion;

import machinekit.assembly.*;
import machinekit.component.*;
import materia.assembly.AssemblyFrames as F;
import materia.assembly.AssemblyRecord.AssemblyFrame;
import machinekit.assembly.AxisBuilder.AxisSpec;
import machinekit.assembly.AssemblyIntegrity.FastenerFitOrder;

/** Screw-driven precision module. The containing machine derives hardware once
 * for the complete assembly. No hobber layout or part identity enters here. */
class ScrewLinearModule extends AxisBuilder {
  public final spec:ScrewLinearModuleSpec;
  public final travel:AxisSpec;
  public function new(?spec:ScrewLinearModuleSpec,?hostBase:MachineComponent,?hostCarriage:MachineComponent) {
    super();this.spec=spec==null?new ScrewLinearModuleSpec():spec;
    var s=this.spec;
    travel={id:"travel",lower:-s.stroke/2,upper:s.stroke/2,initial:0.0};
    group("Ground base and guides");
    place("base",hostBase==null?made("base"):hostBase,F.identity());
    for(side in [-1,1]) {
      var label=side<0?"Left":"Right";
      attach("wall"+label,made("wall"+label),F.identity(),"base");
      attach("rail"+label,LinearRail.metric(s.profile,s.railLength),s.railPose(side),"wall"+label);
    }
    var blockPoses=[for(side in [-1,1])for(row in [-1,1])blockPose(side,row)];
    slide(travel,"railLeft","blockLeftFront",LinearRailBlock.metric(s.profile,LinearRailBlock.SUPPLIER),blockPoses[0],{x:1.0,y:0,z:0},blockPoses,s.overtravel);
    group("Carriage");
    attach("carriage",hostCarriage==null?made("carriage"):hostCarriage,F.identity(),"blockLeftFront");
    for(side in [-1,1])for(row in [-1,1])if(side!=-1||row!=-1)
      attach("block"+(side<0?"Left":"Right")+(row<0?"Front":"Rear"),LinearRailBlock.metric(s.profile,LinearRailBlock.SUPPLIER),blockPose(side,row),"carriage");
    attach("nutCarrier",made("nutCarrier"),F.identity(),"carriage");
    group("Screw drive");
    var screw=new HzMotionDrawingScrew(s.screwLength,s.drive.endProfile);
    s.drive.place(this,s.axis(),{fixedHost:"base",floatingHost:"base",nutHost:"nutCarrier"},screw);
    attach("nut",new BallNut(),s.nutPose(),"nutCarrier");
    addTransmission("lead","travel","screw-turn",Transmission.LeadScrew("screw","nut"),Sense.Opposite);
    supportScrew("lead",ScrewSupport.Fixed,s.drive.floating.support);
    group("Sealed scale");
    attach("scaleHousing",s.scaleHousing,s.scalePose(),"wallLeft");
    attach("scaleHeadCarrier",made("scaleHeadCarrier"),F.identity(),"carriage");
    attach("scaleHead",s.scaleHead,s.scalePose(true),"scaleHeadCarrier");
    addEncoder("position","travel","scaleHead");exposePort("position-feedback","scaleHead","biss");exposePort("scale-air","scaleHousing","air");
    group("Stops");
    for(role in ScrewLinearModulePart.ROLES)if(StringTools.startsWith(role,"stop"))attach(role,made(role),F.identity(),"base");
    exposeConnector("mount","base","mount");exposeConnector("carriage","carriage","interface");
    // Deliberately no controller/drive choice in the mechanical library component.
    for(requirement in serviceRemovals())addDesignRequirement(requirement);
  }
  function made(role:String):ScrewLinearModulePart return new ScrewLinearModulePart(spec.values().setToken("role",role));
  function blockPose(side:Int,row:Int):AssemblyFrame return AxisBuilder.orient(spec.carriageCentre+row*spec.blockSpacing/2,spec.railY(side),
    spec.baseThickness+spec.wallHeight+spec.guide.railHeight,[0.0,0,1],[1.0,0,0]);
  public function fittingOrder():Array<FastenerFitOrder> {
    var out:Array<FastenerFitOrder>=[
      {member:"railLeft",fastened:["wallLeft"],faces:["stock-base"]},{member:"railRight",fastened:["wallRight"],faces:["stock-base"]},
      {member:"carriage",fastened:["motorMount","floatingBearing"],faces:["foot"]},
      {member:"blockLeftFront",fastened:["scaleHead"],faces:["mount"]},
      {member:"blockLeftRear",fastened:["scaleHead"],faces:["mount"]}
    ];
    return out;
  }
  /** Every derived joint has one alignment instruction; unknown additions fail
   * rather than inheriting a precision assumption from their member name. */
  public function alignment():Map<String,String> {
    var derived=derivedFasteners();if(derived==null)throw "Derive module joints before alignment";
    var methods=new Map<String,String>();
    for(joint in derived.joints) {
      var member=joint.clampedMember, face=joint.clampedFace, method:String;
      if(member=="railLeft"||member=="railRight"||member=="motor")method="reference edge";
      else if(member=="motorMount"||member=="floatingBearing"||member=="nut")method="indicate then match-pin";
      else if(member=="wallLeft"||member=="wallRight"||StringTools.startsWith(member,"stop"))method="drawing dowels";
      else if(member=="scaleHousing"||member=="scaleHead"||member=="carriage"&&face=="scale-head-carrier")method="shim or adjust";
      else if(member=="carriage"&&face=="nut-carrier")method="indicate then match-pin";
      else if(member=="carriage"&&StringTools.startsWith(face,"block"))method="shim or adjust";
      else throw 'Module precision joint ${joint.id} needs an alignment declaration';
      methods.set(joint.id,method);
    }
    return methods;
  }
  /** Tool IDs are shared with the containing machine's tooling BOM. */
  public function assemblySteps():Array<{parts:Array<String>,tools:Array<String>,measurement:String}> return [
    {parts:["base","wallLeft","wallRight"],tools:["straightedge-granite","dial-indicator","magnetic-base","torque-driver"],measurement:"Inspect bought stock; require 0.01 mm block-pad coplanarity, select accepted stock or supplier-finish failing small faces; replication is reserved for large casting surfaces. Fit drawing dowels after bolting full-length walls."},
    {parts:["railLeft","railRight","blockLeftFront","blockLeftRear","blockRightFront","blockRightRear"],tools:["straightedge-granite","dial-indicator","magnetic-base","block-transfer-sleeves"],measurement:"Locate rails on reference edges, indicate parallel and transfer matched blocks without losing balls."},
    {parts:["motorMount","floatingBearing","screw","motor"],tools:["dial-indicator","magnetic-base","torque-driver"],measurement:"Fit supports and the actual screw loosely. After the carriage/nut carrier is fitted, indicate the screw parallel to rails at both travel ends, torque and match-pin supports. Catalogue motor pilot locates the motor."},
    {parts:["nut","nutCarrier","carriage"],tools:["dial-indicator","magnetic-base","torque-driver"],measurement:"Indicate nut carrier to screw at BOTH ends of travel, tighten then match-drill and ream; do not pre-drill carrier dowels. Adjust block pads to coplanarity target."},
    {parts:["wallLeft","scaleHousing","scaleHeadCarrier","scaleHead"],tools:["dial-indicator","magnetic-base","feeler-gauges","af3-low-profile-key","air-fitting-socket"],measurement:"Bench-fit the carrier to the carriage before the host head/seat. Reference housing seat to guide edge. Indicate housing to 0.1 mm and head to 0.05 mm parallelism. Fit carrier face to measured stack (0.5 mm each-way allowance ASSUMED); set lateral offset within 0.2 mm, gap 1 +/-0.3 mm at the head; confirm the reading window and 75 mm cable bend. Head screws use the low-profile key from below and slide the head out longitudinally for service."},
    {parts:["stopFront","stopRear"],tools:["dial-indicator","magnetic-base","torque-driver"],measurement:"Indicate each floor block against the nut carrier at its overtravel end, then tighten its two M6. Verify both stops engage before screw support contact. Fit the drawing dowels, which carry the crash; verify their model capacity. Host must limit torque to the bought bearing rating."}
  ];
  public function stopContacts():Array<{a:String,b:String}> return [{a:"nutCarrier",b:"stopFront"},{a:"nutCarrier",b:"stopRear"}];

  public function serviceRemovals():Array<machinekit.design.DesignRequirement> return [
    machinekit.design.DesignRequirement.ServiceRemoval("motor",["motor","motorShaft"],[{direction:[1.0,0,0],distance:spec.drive.motor.rating.shaftLength+10}],[],[]),
    machinekit.design.DesignRequirement.ServiceRemoval("screw",["screw","coupling","nut"],[{direction:[1.0,0,0],distance:spec.baseMax+10}],
      ["motor","motorShaft","motorMount","floatingRing"],[]),
    machinekit.design.DesignRequirement.ServiceRemoval("scaleHead",["scaleHead"],[{direction:[1.0,0,0],distance:spec.baseMax-spec.scaleCentre+spec.scaleHead.series.headLength+20}],[],[]),
    machinekit.design.DesignRequirement.ServiceRemoval("scaleHousing",["scaleHousing"],[{direction:[1.0,0,0],distance:spec.baseMax-spec.scaleStart()+spec.scaleHousing.overallLength()+20}],["scaleHead"],[]),
    machinekit.design.DesignRequirement.ServiceRemoval("carriage",["carriage","scaleHeadCarrier","scaleHead"],[{direction:[0.0,0,1],distance:spec.carriageThickness+20}],[],[])
  ];
  /** Rated thrust / peak motor torque, plus the containing machine's declared
   * payload shear. Half thrust per guide is 2x equal sharing over four blocks. */
  public function loads(payload:machinekit.assembly.JointSlip.LoadCase):Array<machinekit.assembly.JointSlip.JointLoad> {
    var nut=new BallNut(),motor=spec.drive.motor.rating;
    var rated=motor.ratedTorque*2*Math.PI/(nut.lead/1000),peak=motor.peakTorque*2*Math.PI/(nut.lead/1000);
    var out:Array<machinekit.assembly.JointSlip.JointLoad>=[
      {label:"Fixed screw support",between:["motorMount","base"],shear:{peak:rated,momentary:peak}},
      {label:"Nut carrier",between:["carriage","nutCarrier"],shear:{peak:rated,momentary:peak}},
      {label:"Motor flange",between:["motor","motorMount"],torque:{peak:motor.ratedTorque,momentary:motor.peakTorque}}
    ];
    for(side in ["Left","Right"]) {
      out.push({label:"Rail "+side,between:["rail"+side,"wall"+side],shear:{peak:rated/2,momentary:peak/2}});
      out.push({label:"Wall "+side,between:["wall"+side,"base"],shear:payload});
      for(row in ["Front","Rear"])out.push({label:"Guide block "+side+row,between:["carriage","block"+side+row],shear:{peak:rated/2,momentary:peak/2}});
    }
    for(connection in [["scaleHousing","wallLeft"],["scaleHead","scaleHeadCarrier"],["carriage","scaleHeadCarrier"]])
      out.push({label:"Scale "+connection[0],between:connection,shear:{peak:20.0,momentary:40.0}}); // ASSUMED includes 4 N scale drag and moving cable.
    for(role in ScrewLinearModulePart.ROLES)if(StringTools.startsWith(role,"stop"))out.push({label:"End stop "+role,between:[role,"base"],shear:{peak:rated,momentary:peak}});
    return out;
  }
  /** Worst-case setting allowance, not a supplier height-tolerance claim.
   * Fit the separate carrier's head face to the measured stack; leave 0.5 mm
   * stock each way. Native slots/clearance set gap and parallelism at the head. */
  public function scaleAdjustment():{normalStack:Float,normalFit:Float,gapStack:Float,gapAdjustment:Float} {
    var screw=machinekit.standard.SocketHeadCapScrew.catalog().get("M4");
    var headHole=spec.scaleHead.series.headHoleDiameter;if(headHole==null)throw "Scale head needs its catalogue mounting holes";
    // ASSUMED: base flatness, wall height, block height, scale housing and carrier finish.
    return {normalStack:0.10+0.05+0.02+0.05+0.02,normalFit:0.5,
      gapStack:0.05+0.05+0.02+0.10,
      gapAdjustment:(screw.clearanceMedium-screw.diameter)/2+(headHole-screw.diameter)/2};
  }
  public function stockAcceptance():String {
    var plate=machinekit.structural.GroundStock.plate(machinekit.structural.GroundStock.STEEL,spec.baseThickness);
    var bar=machinekit.structural.GroundStock.bar(spec.wallWidth,spec.wallHeight);
    var worst=plate.flatness*Math.max(spec.baseMax-spec.baseMin,spec.baseWidth)/100+bar.flatness*spec.railLength/100+0.02;
    return 'ASSUMED conservative stock/block bound $worst mm (plate/bar figures per100 mm, block height variation0.02 ASSUMED) versus0.01 mm required block-pad coplanarity. Inspect bought ground stock, select accepted stock or supplier-finish failing small faces; replication is reserved for large casting surfaces; process qualification remains open.';
  }
}
