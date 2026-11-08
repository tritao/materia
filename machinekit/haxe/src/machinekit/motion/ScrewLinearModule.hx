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
  public function new(?spec:ScrewLinearModuleSpec,?hostBase:MachineComponent,?hostCarriage:MachineComponent,?hostGuards:Map<String,MachineComponent>) {
    super();this.spec=spec==null?new ScrewLinearModuleSpec():spec;
    var s=this.spec;
    travel={id:"travel",lower:-s.stroke/2,upper:s.stroke/2,initial:0.0};
    group("Ground base and guides");
    place("base",hostBase==null?made("base"):hostBase,F.identity());
    for(side in [-1,1]) {
      var label=side<0?"Left":"Right";
      attach("riser"+label,made("riser"+label),F.identity(),"base");
      attach("rail"+label,LinearRail.metric(s.profile,s.railLength),s.railPose(side),"riser"+label);
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
    attach("scaleSeat",made("scaleSeat"),F.identity(),"base");
    attach("scaleHousing",s.scaleHousing,s.scalePose(),"scaleSeat");
    attach("scaleHeadCarrier",made("scaleHeadCarrier"),F.identity(),"carriage");
    attach("scaleHead",s.scaleHead,s.scalePose(true),"scaleHeadCarrier");
    addEncoder("position","travel","scaleHead");exposePort("position-feedback","scaleHead","biss");exposePort("scale-air","scaleHousing","air");
    group("Stops");
    for(role in ScrewLinearModulePart.ROLES)if(StringTools.startsWith(role,"stop"))attach(role,made(role),F.identity(),"base");
    if(s.stroke<=20) {
      group("Chip guard");
      var suffixes=hostGuards==null?["Front","Rear"]:[for(key in hostGuards.keys())if(StringTools.startsWith(key,"cover"))key.substr(5)];
      suffixes.sort((a,b)->Reflect.compare(a,b));
      if(suffixes.length!=2)throw "Short module requires two guard halves";
      for(suffix in suffixes) {
        var cover="cover"+suffix,retainer="retainer"+suffix;
        var supplied=hostGuards==null?null:hostGuards.get(cover);
        if(supplied==null)supplied=new ScrewLinearModuleCover(s.values().setToken("role",cover));
        attach(cover,supplied,F.identity(),"base");
        var suppliedRetainer=hostGuards==null?null:hostGuards.get(retainer),lip="lip"+suffix,suppliedLip=hostGuards==null?null:hostGuards.get(lip);
        if(hostGuards!=null&&(suppliedRetainer==null||suppliedLip==null))throw "Host guard requires its retainer and lip: "+suffix;
        attach(retainer,suppliedRetainer==null?new ScrewLinearModuleCover(s.values().setToken("role",retainer)):suppliedRetainer,F.identity(),cover);
        attach(lip,suppliedLip==null?new ScrewLinearModuleCover(s.values().setToken("role",lip)):suppliedLip,F.identity(),retainer);
      }
    }
    if(s.stroke>20)ScrewLinearModuleBellows.install(this);
    exposeConnector("mount","base","mount");exposeConnector("carriage","carriage","interface");
    // Deliberately no controller/drive choice in the mechanical library component.
    for(requirement in serviceRemovals())addDesignRequirement(requirement);
    if(s.stroke<=20&&hostGuards==null)for(front in [true,false]) {
      var suffix=front?"Front":"Rear";
      addDesignRequirement(machinekit.design.DesignRequirement.ServiceRemoval("guard"+suffix,["cover"+suffix,"retainer"+suffix,"lip"+suffix],
        [{direction:[front?-1.0:1.0,0,0],distance:(front?s.centre-s.baseMin:s.baseMax-s.centre)+20}],front?["scaleHead"]:["motor","motorShaft"],[]));
    }
  }
  public function guardMembers():Array<String> return spec.stroke<=20?[for(m in components())if(StringTools.startsWith(m.id,"cover")||StringTools.startsWith(m.id,"retainer")||StringTools.startsWith(m.id,"lip"))m.id]:ScrewLinearModuleBellows.members(spec);
  function guardSuffixes():Array<String> return [for(id in guardMembers())if(StringTools.startsWith(id,"cover"))id.substr(5)];
  function made(role:String):ScrewLinearModulePart return new ScrewLinearModulePart(spec.values().setToken("role",role));
  function blockPose(side:Int,row:Int):AssemblyFrame return AxisBuilder.orient(spec.centre+row*spec.blockSpacing/2,spec.railY(side),
    spec.baseThickness+spec.riserHeight+spec.guide.railHeight,[0.0,0,1],[1.0,0,0]);
  public function fittingOrder():Array<FastenerFitOrder> {
    var out:Array<FastenerFitOrder>=[
      {member:"railLeft",fastened:["riserLeft"],faces:["stock-base"]},{member:"railRight",fastened:["riserRight"],faces:["stock-base"]},
      {member:"scaleHousing",fastened:["scaleSeat"],faces:["base"]},
      {member:"carriage",fastened:["motorMount","floatingBearing"],faces:["foot"]}
    ];
    if(spec.stroke<=20&&guardSuffixes().contains("Front")) {
      out.push({member:"coverFront",fastened:["stopFrontLeft","stopFrontRight"],faces:["base"]});
      out.push({member:"coverRear",fastened:["stopRearLeft","stopRearRight"],faces:["base"]});
    }
    if(spec.stroke>20)for(side in ["Front","Rear"])out.push({member:"mountFixed"+side,fastened:["stop"+side+"Left","stop"+side+"Right"],faces:["base"]});
    if(spec.stroke<=20) {
      var guards=guardMembers();
      var mechanism=[for(m in components())if(!guards.contains(m.id))m.id];
      // Tighten the precision mechanism before fitting shields. Retainer
      // hardware is tightened on the bench as a separate guard subassembly.
      for(guard in guards)out.push({member:guard,fastened:mechanism.filter(id->id!="base")});
      for(id in mechanism)out.push({member:id,fastened:guards.filter(id->StringTools.startsWith(id,"retainer")),faces:["seal"]});
    }
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
      else if(member=="riserLeft"||member=="riserRight"||StringTools.startsWith(member,"stop"))method="drawing dowels";
      else if(StringTools.startsWith(member,"cover")||StringTools.startsWith(member,"retainer")||StringTools.startsWith(member,"mount")||StringTools.startsWith(member,"bellows"))method="shim or adjust";
      else if(member=="scaleSeat"||member=="scaleHousing"||member=="scaleHead"||member=="carriage"&&face=="scale-head-carrier")method="shim or adjust";
      else if(member=="carriage"&&face=="nut-carrier")method="indicate then match-pin";
      else if(member=="carriage"&&StringTools.startsWith(face,"block"))method="shim or adjust";
      else throw 'Module precision joint ${joint.id} needs an alignment declaration';
      methods.set(joint.id,method);
    }
    return methods;
  }
  /** Tool IDs are shared with the containing machine's tooling BOM. */
  public function assemblySteps():Array<{parts:Array<String>,tools:Array<String>,measurement:String}> return [
    {parts:["base","riserLeft","riserRight"],tools:["straightedge-granite","dial-indicator","magnetic-base","torque-driver"],measurement:"Inspect bought stock; require 0.01 mm block-pad coplanarity, replicate failing faces before custom grinding. Fit drawing dowels after bolting risers."},
    {parts:["railLeft","railRight","blockLeftFront","blockLeftRear","blockRightFront","blockRightRear"],tools:["straightedge-granite","dial-indicator","magnetic-base","block-transfer-sleeves"],measurement:"Locate rails on reference edges, indicate parallel and transfer matched blocks without losing balls."},
    {parts:["motorMount","floatingBearing","screw","motor"],tools:["drive-alignment-arbor","nut-bore-height-reference","dial-indicator","magnetic-base","torque-driver"],measurement:"Indicate arbor coaxial and parallel to rails at carriage nut-bore height; tighten then match-drill and ream support dowels. Catalogue motor pilot locates the motor."},
    {parts:["nut","nutCarrier","carriage"],tools:["dial-indicator","magnetic-base","torque-driver"],measurement:"Indicate nut carrier to screw at BOTH ends of travel, tighten then match-drill and ream; do not pre-drill carrier dowels. Adjust block pads to coplanarity target."},
    {parts:["scaleSeat","scaleHousing","scaleHeadCarrier","scaleHead"],tools:["dial-indicator","magnetic-base","feeler-gauges","af3-low-profile-key","air-fitting-socket"],measurement:"Reference housing seat to guide edge. Indicate housing to 0.1 mm and head to 0.05 mm parallelism. Fit carrier face to measured stack (0.5 mm each-way allowance ASSUMED); set lateral offset within 0.2 mm, gap 1 +/-0.3 mm at the head; confirm the reading window and 75 mm cable bend. Head screws use the low-profile key from below and slide the head out longitudinally for service."},
    {parts:["stopFrontLeft","stopFrontRight","stopRearLeft","stopRearRight"],tools:["dial-indicator","magnetic-base","torque-driver"],measurement:"Fit drawing dowels and verify both overtravel stops engage before screw support contact. Host must limit torque to the bought bearing rating."},
    {parts:guardMembers(),tools:["af2.5-key","af5.5-long-spanner","torque-driver","af3-low-profile-key","feeler-gauges"],measurement:spec.stroke<=20?"Preassemble the split covers and separate steel retainers with clip-in PU lips on the bench. Ordinary retainer nuts remain accessible after sliding off one cover; do not trap or glue them. Fit the declared base edge joints. Follow the containing machine's declared guard removal paths and prerequisites; the default longitudinal split requires the head off at the front and motor/shaft off at the rear. Host splits must leave precision supports installed for routine guard and wiper service. Check 10 mm minimum roof/carriage overlap and continuous lip contact through overtravel. Outward R75 cable, end sealing and clip profile require qualification.":"Install the declared bellows and separate flange mounts; through-tapped flange holes return on the bolted interface. Indicate the two mount planes parallel and check full travel before fitting guards. Catalogue row and cable envelope remain ASSUMED."}
  ];
  public function stopContacts():Array<{a:String,b:String}> {
    var out=[for(role in ScrewLinearModulePart.ROLES)if(StringTools.startsWith(role,"stop")){a:"carriage",b:role}];
    if(spec.stroke<=20)for(side in guardSuffixes())out.push({a:"carriage",b:"lip"+side});return out;
  }
  public function serviceRemovals():Array<machinekit.design.DesignRequirement> return [
    machinekit.design.DesignRequirement.ServiceRemoval("motor",["motor","motorShaft"],[{direction:[1.0,0,0],distance:spec.drive.motor.rating.shaftLength+10}],[],[]),
    machinekit.design.DesignRequirement.ServiceRemoval("screw",["screw","coupling","nut"],[{direction:[1.0,0,0],distance:spec.baseMax+10}],
      ["motor","motorShaft","motorMount","floatingRing"].concat(guardMembers()),[]),
    machinekit.design.DesignRequirement.ServiceRemoval("scaleHead",["scaleHead"],[{direction:[-1.0,0,0],distance:spec.centre-spec.baseMin+spec.scaleHead.series.headLength+20}],[],[]),
    machinekit.design.DesignRequirement.ServiceRemoval("scaleHousing",["scaleHousing"],[{direction:[1.0,0,0],distance:spec.baseMax-spec.scaleStart()+spec.scaleHousing.overallLength()+20}],["scaleHead"].concat(guardMembers()),[]),
    machinekit.design.DesignRequirement.ServiceRemoval("carriage",["carriage","scaleHeadCarrier","scaleHead"],[{direction:[0.0,0,1],distance:spec.carriageThickness+20}],guardMembers(),[])
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
      out.push({label:"Rail "+side,between:["rail"+side,"riser"+side],shear:{peak:rated/2,momentary:peak/2}});
      out.push({label:"Riser "+side,between:["riser"+side,"base"],shear:payload});
      for(row in ["Front","Rear"])out.push({label:"Guide block "+side+row,between:["carriage","block"+side+row],shear:{peak:rated/2,momentary:peak/2}});
    }
    if(spec.stroke>20)for(suffix in ["Front","Rear"])for(connection in [["mountFixed"+suffix,"base"],["mountMoving"+suffix,"carriage"],["bellows"+suffix+"-fixed","mountFixed"+suffix],["bellows"+suffix+"-moving","mountMoving"+suffix]])
      out.push({label:"Bellows "+connection[0],between:connection,shear:{peak:40.0,momentary:100.0},torque:{peak:5.0,momentary:10.0}}); // ASSUMED guard loads.
    if(spec.stroke<=20)for(suffix in guardSuffixes()) {
      out.push({label:"Guard "+suffix,between:["cover"+suffix,"base"],shear:{peak:30.0,momentary:100.0}});
      out.push({label:"Wiper retainer "+suffix,between:["retainer"+suffix,"cover"+suffix],shear:{peak:10.0,momentary:20.0}});
    }
    for(connection in [["scaleSeat","base"],["scaleHousing","scaleSeat"],["scaleHead","scaleHeadCarrier"],["carriage","scaleHeadCarrier"]])
      out.push({label:"Scale "+connection[0],between:connection,shear:{peak:20.0,momentary:40.0}}); // ASSUMED includes 4 N scale drag and moving cable.
    for(role in ScrewLinearModulePart.ROLES)if(StringTools.startsWith(role,"stop"))out.push({label:"End stop "+role,between:[role,"base"],shear:{peak:rated,momentary:peak}});
    return out;
  }
  /** Worst-case setting allowance, not a supplier height-tolerance claim.
   * Fit the separate carrier's head face to the measured stack; leave 0.5 mm
   * stock each way. Native slots/clearance set gap and parallelism at the head. */
  /** Long-stroke bellows include the complete scale cable envelope. */
  public function coverPlan():Null<machinekit.cover.WayCoverPlan> return spec.stroke<=20?null:ScrewLinearModuleBellows.plan(spec);
  public function scaleAdjustment():{normalStack:Float,normalFit:Float,gapStack:Float,gapAdjustment:Float} {
    var screw=machinekit.standard.SocketHeadCapScrew.catalog().get("M4");
    var headHole=spec.scaleHead.series.headHoleDiameter;if(headHole==null)throw "Scale head needs its catalogue mounting holes";
    // ASSUMED: base flatness, riser thickness, block height, scale-seat thickness, carrier finish.
    return {normalStack:0.10+0.05+0.02+0.05+0.02,normalFit:0.5,
      gapStack:0.05+0.05+0.02+0.10,
      gapAdjustment:(screw.clearanceMedium-screw.diameter)/2+(headHole-screw.diameter)/2};
  }
  public function stockAcceptance():String {
    var plate=machinekit.structural.GroundStock.plate(machinekit.structural.GroundStock.STEEL,spec.baseThickness);
    var bar=machinekit.structural.GroundStock.bar(spec.riserWidth,spec.riserHeight);
    var worst=plate.flatness*Math.max(spec.baseMax-spec.baseMin,spec.baseWidth)/100+bar.flatness*spec.railLength/100+0.02;
    return 'ASSUMED conservative stock/block bound $worst mm (plate/bar figures per100 mm, block height variation0.02 ASSUMED) versus0.01 mm required block-pad coplanarity. Inspect bought ground stock, replicate failing faces before custom grinding; process qualification remains open.';
  }
}
