import machinekit.assembly.MachineAssembly;
import machinekit.component.MachineComponent;
import machinekit.motion.*;
import machinekit.milling.*;
import machinekit.transmission.TimingBelt;
import machinekit.transmission.TimingPulley;
import haxe.Json;

private typedef DutyPhase = {var name:String;var seconds:Float;var speedMmPerSecond:Float;var acceleration:Float;var cuttingN:Float;}
private typedef Demand = {var peakNm:Float;var rmsNm:Float;var maximumRpm:Float;var loadInertiaKgM2:Float;var inertiaRatio:Float;var cycleSeconds:Float;var phases:Array<PhaseDemand>;}
private typedef PhaseDemand = {var name:String;var seconds:Float;var motorRpm:Float;var torqueUpperBoundNm:Float;}
private typedef AxisData = {
 var axis:String;var root:String;var motor:String;var screw:String;var rotating:Array<String>;
 var bearingSpan:Float;var stroke:Float;var travel:Float;var vertical:Bool;
}

/** Local-model screening. Conservative torque magnitudes deliberately do not
 * cancel gravity, cutting or inertia; the reported RMS is an upper bound.
 */
class MillAxisSizing {
 public static final PAYLOAD_KG=20.0;
 public static final CUTTING_N=500.0;
 public static final ACCELERATION=1.0;
 public static final RAPID_MM_MIN=10000.0;
 public static final FEED_MM_MIN=3000.0;
 public static final GUIDE_DRAG_N=30.0;
 public static final RESERVE=1.25;
 public static final INERTIA_RATIO_TARGET=10.0;
 static final G=9.80665;

 static function member(a:MachineAssembly,id:String):MachineComponent {
  for(m in a.components())if(m.id==id)return m.component;
  throw 'Missing sizing member $id';
 }
 static function descends(id:String,root:String,parents:Map<String,String>):Bool {
  var seen:Map<String,Bool> = [];
  while(id!=null&&!seen.exists(id)){if(id==root)return true;seen.set(id,true);id=parents.get(id);}
  return false;
 }
 static function cycle(stroke:Float):Array<DutyPhase> {
  var phases:Array<DutyPhase> = [];
  for(leg in [{name:"cut",speed:FEED_MM_MIN/60,force:CUTTING_N},{name:"rapid return",speed:RAPID_MM_MIN/60,force:0.0}]) {
   var v=Math.min(leg.speed,Math.sqrt(stroke*ACCELERATION*1000));
   var ramp=v/(ACCELERATION*1000),cruise=Math.max(0,stroke/v-ramp);
   phases.push({name:leg.name+" accelerate",seconds:ramp,speedMmPerSecond:v,acceleration:ACCELERATION,cuttingN:leg.force});
   if(cruise>0)phases.push({name:leg.name+" cruise",seconds:cruise,speedMmPerSecond:v,acceleration:0,cuttingN:leg.force});
   phases.push({name:leg.name+" decelerate",seconds:ramp,speedMmPerSecond:v,acceleration:ACCELERATION,cuttingN:leg.force});
  }
  phases.push({name:"powered dwell",seconds:1,speedMmPerSecond:0,acceleration:0,cuttingN:0});
  return phases;
 }
 /** q is metres of travel per motor radian; rotary inertia is already referred
  * to the motor. Efficiency penalizes load inertia, not the motor's own rotor.
  */
 static function demand(mass:Float,q:Float,efficiency:Float,rotary:Float,rotor:Float,gravity:Float,drag:Float,stroke:Float):Demand {
  if(!(mass>0)||!(q>0)||!(efficiency>0&&efficiency<=1)||!(rotor>0)||rotary<0||!Math.isFinite(mass+q+rotary+rotor+gravity+drag)||gravity<0||drag<0||!(stroke>0))throw "Invalid axis sizing input";
  var peak=0.0,squares=0.0,time=0.0,rpm=0.0,phases:Array<PhaseDemand> = [];
  for(p in cycle(stroke)) {
   var speed=p.speedMmPerSecond/1000/q,alpha=p.acceleration/q;
   var force=gravity+(p.speedMmPerSecond>0?GUIDE_DRAG_N+p.cuttingN:0)+mass*p.acceleration;
   var torque=force*q/efficiency+(rotary/efficiency+rotor)*alpha+(p.speedMmPerSecond>0?drag:0);
   peak=Math.max(peak,torque);squares+=torque*torque*p.seconds;time+=p.seconds;rpm=Math.max(rpm,speed*60/(2*Math.PI));
   phases.push({name:p.name,seconds:p.seconds,motorRpm:speed*60/(2*Math.PI),torqueUpperBoundNm:torque});
  }
  var load=mass*q*q+rotary;
  return {peakNm:peak,rmsNm:Math.sqrt(squares/time),maximumRpm:rpm,loadInertiaKgM2:load,inertiaRatio:load/rotor,cycleSeconds:time,phases:phases};
 }
 static function near(a:Float,b:Float,label:String):Void {if(!Math.isFinite(a)||Math.abs(a-b)>1e-6)throw '$label: $a != $b';}
 static function checks():Void {
  // Analytical inertial reflection: 100 kg on a 5 mm lead, direct drive.
  var q=0.005/(2*Math.PI),d=demand(100,q,1,0,0.0001,0,0,50);
  near(d.loadInertiaKgM2,100*q*q,"linear inertia reflection");
  var expected=(100+GUIDE_DRAG_N+CUTTING_N)*q+0.0001/q;
  near(d.peakNm,expected,"force and rotor acceleration torque");
  var heavier=demand(200,q,1,0,0.0001,0,0,50);
  if(heavier.peakNm<=d.peakNm)throw "Payload must increase acceleration demand";
  var vertical=demand(100,q,0.9,0,0.0001,100*G,0,50);
  near(vertical.phases[vertical.phases.length-1].torqueUpperBoundNm,100*G*q/0.9,"powered gravity dwell");
  var shortCycle=cycle(1),distance=0.0;
  for(p in shortCycle)if(p.name.indexOf("rapid return")>=0)distance+=p.speedMmPerSecond*p.seconds*(p.acceleration>0?0.5:1);
  near(distance,1,"short triangular return covers commanded distance");
  if(d.rmsNm>d.peakNm)throw "RMS cannot exceed peak";
  Sys.println("Axis sizing analytical checks passed");
 }
 static function axisReport(a:MachineAssembly,data:AxisData):Dynamic {
  var parents=a.mateParents(),mass=0.0,masses:Array<Dynamic> = [];
  for(m in a.components())if(descends(m.id,data.root,parents)) {
   var p=m.component.massProperties();mass+=p.mass;masses.push({member:m.id,kg:p.mass});
  }
  if(masses.length==0)throw 'No moving members for ${data.axis}';
  var motor=cast(member(a,data.motor),ServoMotor),screw=cast(member(a,data.screw),LeadScrew);
  var nut=cast(member(a,data.axis=="z"?"zAxis/nut":"xyStage/nut"+data.axis.toUpperCase()),LeadScrewNut);
  var relation=LeadScrew.relation(screw,nut,1),ratio=1.0,beltEfficiency=1.0,beltDrag=0.0;
  if(data.axis=="x") {
   var belt=cast(member(a,"xyStage/beltX"),TimingBelt);
   var transmission=TimingBelt.reduction(belt,cast(member(a,"xyStage/motorPulleyX"),TimingPulley),cast(member(a,"xyStage/screwPulleyX"),TimingPulley),1);
   ratio=Math.abs(transmission.ratio);beltEfficiency=transmission.efficiency;beltDrag=transmission.drag==null?0:transmission.drag;
  }
  var rotary=0.0,inertias:Array<Dynamic> = [];
  for(id in data.rotating) {
   var p=member(a,id).massProperties();if(p.inertia==null)throw 'Missing rotating inertia for $id';
   var own=(p.inertia.zz+p.mass*(p.centreOfMass.x*p.centreOfMass.x+p.centreOfMass.y*p.centreOfMass.y))*1e-6;
   var scale=id=="xyStage/motorPulleyX"?1.0:ratio*ratio;
   rotary+=own*scale;inertias.push({member:id,ownAxisKgM2:own,referredToMotorKgM2:own*scale});
  }
  var loaded=mass+(data.vertical?0:PAYLOAD_KG),q=ratio/(1000*Math.abs(relation.ratio));
  var efficiency=relation.efficiency*beltEfficiency,drag=(relation.drag==null?0:relation.drag)*ratio/beltEfficiency+beltDrag;
  var gravity=data.vertical?loaded*G:0;
  var d=demand(loaded,q,efficiency,rotary,motor.rating.rotorInertia,gravity,drag,data.stroke);
  var sustainedCut=(gravity+GUIDE_DRAG_N+CUTTING_N)*q/efficiency+drag;
  var critical=screw.criticalSpeed(Fixed,Simple,data.bearingSpan)*60/(2*Math.PI);
  var screwRpm=d.maximumRpm*ratio,availableRated=motor.rating.ratedTorque/RESERVE;
  var ratedSpeed=motor.rating.ratedSpeed*60/(2*Math.PI),maxSpeed=motor.rating.maxSpeed*60/(2*Math.PI);
  var powerPass=d.rmsNm<=availableRated&&d.peakNm*RESERVE<=motor.rating.peakTorque&&d.maximumRpm<=ratedSpeed&&screwRpm<=critical;
  var hold=gravity*q; // frictionless backdrive is conservative; no reliance on guide friction.
  var candidates:Array<Dynamic> = [];
  for(name in LeadshineServo.catalog().designations()) {
   var candidate=LeadshineServo.model(name),r=candidate.rating;
   var cd=demand(loaded,q,efficiency,rotary,r.rotorInertia,gravity,drag,data.stroke);
   var speedOK=cd.maximumRpm<=r.ratedSpeed*60/(2*Math.PI);
   var passes=cd.rmsNm*RESERVE<=r.ratedTorque&&cd.peakNm*RESERVE<=r.peakTorque&&speedOK&&screwRpm<=critical;
   candidates.push({motor:name,watts:candidate.spec.watts,voltage:candidate.spec.voltage,brake:candidate.spec.brake,
    rmsNm:cd.rmsNm,peakNm:cd.peakNm,inertiaRatio:cd.inertiaRatio,
    torqueAndSpeedScreenPass:passes,sustainedCutScreenPass:sustainedCut*RESERVE<=r.ratedTorque,studyInertiaTargetPass:cd.inertiaRatio<=INERTIA_RATIO_TARGET,
    mountingCompatibleConfirmed:false,brakeHoldingCapacityConfirmed:false});
  }
  return {axis:data.axis,status:powerPass?"preliminary torque/speed screen passes; supplier qualification pending":"preliminary screen fails or needs speed-dependent curve",
   motor:motor.designation,ratingsBasis:Std.string(motor.ratingsBasis),cadMovingMassKg:mass,payloadKg:data.vertical?0:PAYLOAD_KG,totalTranslatingMassKg:loaded,
   movingMembers:masses,rotatingInertiaMembers:inertias,rotaryLoadInertiaAtMotorKgM2:rotary,
   leadMm:Math.abs(screw.thread.signedLead()),screwRevolutionsPerMotorRevolution:ratio,efficiency:efficiency,runningRotaryDragNm:drag,
   dutyStrokeMm:data.stroke,travelMm:data.travel,demand:d,ratedTorqueNm:motor.rating.ratedTorque,peakTorqueNm:motor.rating.peakTorque,
   rmsUtilizationWithReserve:d.rmsNm*RESERVE/motor.rating.ratedTorque,peakUtilizationWithReserve:d.peakNm*RESERVE/motor.rating.peakTorque,
   motorRatedRpm:ratedSpeed,motorMaxRpm:maxSpeed,requiredScrewRpm:screwRpm,screwSpeedLimitRpm:critical,bearingSpanMm:data.bearingSpan,
   sustainedCutTorqueNm:sustainedCut,sustainedCutUtilizationWithReserve:sustainedCut*RESERVE/motor.rating.ratedTorque,
   sustainedCutScreenPass:sustainedCut*RESERVE<=motor.rating.ratedTorque,
   ratedSpeedRegion:d.maximumRpm<=ratedSpeed,torqueAndSpeedScreenPass:powerPass,studyInertiaTargetPass:d.inertiaRatio<=INERTIA_RATIO_TARGET,
   zHolding:data.vertical?{frictionlessHoldingNm:hold,requiredHoldingWithReserveNm:hold*RESERVE,internalBrakePresent:Std.isOfType(motor,LeadshineServo)&&cast(motor,LeadshineServo).spec.brake,
    holdingCapacityQualified:false,
    qualification:"Internal Leadshine brake torque and timing are unpublished locally; dynamic stopping, release sequencing and power-loss safety unqualified"}:null,
   leadshineCandidates:candidates};
 }
 public static function main():Void {
  checks();if(Sys.args().indexOf("--checks-only")>=0)return;
  var c=MillMotorStudy.context(),z:MillZAxisLayout=c.frame.zAxis,xy:MillXYStageLayout=c.frame.xyAxis;
  var axes:Array<AxisData> = [
   {axis:"x",root:"xyStage/blockX0",motor:"xyStage/motorX",screw:"xyStage/screwX",rotating:["xyStage/screwX","xyStage/screwRetainerX","xyStage/screwPulleyX","xyStage/motorPulleyX"],bearingSpan:xy.xFloatingX-xy.xFixedX,stroke:50,travel:2*xy.xHalfTravel,vertical:false},
   {axis:"y",root:"xyStage/blockY0",motor:"xyStage/motorY",screw:"xyStage/screwY",rotating:["xyStage/screwY","xyStage/screwRetainerY","xyStage/couplingY"],bearingSpan:Math.abs(xy.yFixedY-xy.yFloatingY),stroke:50,travel:2*xy.yHalfTravel,vertical:false},
   {axis:"z",root:"zAxis/blockLeftLower",motor:"zAxis/motor",screw:"zAxis/screw",rotating:["zAxis/screw","zAxis/coupling"],bearingSpan:Math.abs(z.fixedZ-z.floatingZ),stroke:50,travel:Math.abs(z.lower),vertical:true}];
  var baseline=MillFramePreview.concept(c.reference,c.frame),baseReports=[for(axis in axes)axisReport(baseline,axis)];
  near(baseReports[2].cadMovingMassKg,MillMotorStudy.movingMass(baseline),"Z moving subtree agrees with spindle study");
  if(baseReports[1].cadMovingMassKg<=baseReports[0].cadMovingMassKg)throw "Y must include nested X stage mass";
  for(axis in baseReports) {
   near(axis.requiredScrewRpm,2000,"5 mm lead and 10 m/min direct or equal-pulley drive");
   if(axis.demand.rmsNm>axis.demand.peakNm)throw "Model RMS exceeds peak";
  }
  var variants:Array<Dynamic> = [];
  for(p in [{name:"ELM2L-1500LA100FT-H MS1AE",motor:80,spindle:30},{name:"ELM2M-2000LB130F-H",motor:80,spindle:25},{name:"ELM2M-2900LD180FT-H",motor:96,spindle:18}]) {
   var drive=MillMotorStudy.selection(c.frame,p.name,p.motor,p.spindle),a=MillFramePreview.concept(c.reference,c.frame,new MillBeltSpindle(0,drive)),m=drive.motor();
   var variantZ=axisReport(a,axes[2]);
   if(variantZ.cadMovingMassKg<=baseReports[2].cadMovingMassKg)throw "Larger spindle arrangement should increase Z mass";
   variants.push({spindleMotor:p.name,zAxis:variantZ,spindle:{targetRpm:8000,targetOutputWatts:2200,
    motorRpmAtTarget:8000/drive.speedRatio(),motorRatedRpm:m.rating.ratedSpeed*60/(2*Math.PI),
    outputWattsAtRatedPoint:m.rating.ratedTorque*m.rating.ratedSpeed*0.95,
    ratedPowerScreenPass:m.rating.ratedTorque*m.rating.ratedSpeed*0.95>=2200,
    continuousAtTargetQualified:false,missing:"matched continuous curve/duty, belt power and spindle bearing limits"}});
  }
  var compactLayout=new MillCapacityLayout(),compact=MillCapacityStudy.build(compactLayout,false),compactZ=compactLayout.zLayout();
  var compactZReport=axisReport(compact,{axis:"z",root:"zAxis/blockLeftLower",motor:"zAxis/motor",screw:"zAxis/screw",
   rotating:["zAxis/screw","zAxis/coupling"],
   bearingSpan:Math.abs(compactZ.fixedZ-compactZ.floatingZ),stroke:50,travel:Math.abs(compactZ.lower),vertical:true});
  var installedDrive=MillSpindleDrivePart.installed(z.headHeight),installedMotor=installedDrive.motor();
  var installedWatts=installedMotor.rating.ratedTorque*installedMotor.rating.ratedSpeed*0.95;
  var report={status:"local mechanical model preliminary sizing; no axis has a supplier-qualified motor/drive selection",scope:"Small XYZ and Compact Z; Compact XY drives remain unfitted; legacy BenchMill control model is different",
   assumptions:{payloadKg:PAYLOAD_KG,cuttingForcePerAxisN:CUTTING_N,accelerationMPerS2:ACCELERATION,rapidMmMin:RAPID_MM_MIN,cuttingFeedMmMin:FEED_MM_MIN,
    extraGuideAndSealDragN:GUIDE_DRAG_N,reserveFactor:RESERVE,inertiaRatioStudyTarget:INERTIA_RATIO_TARGET,
    duty:"50 mm cutting traverse, 50 mm rapid return, 1 second powered dwell; triangular profiles when needed",
    provenance:"20 kg/500 N/1 m/s² match MillDesignStudy; feeds, drag, reserve, duty and inertia target are explicit study assumptions",
    torqueModel:"upper bound sums gravity, cutting, drag and inertia magnitudes without cancellation; RMS of those bounds; assumes nameplate torque through rated speed only",
    inertia:"CAD rotating inertia excludes motor shaft because motor rotor inertia already includes it; screw side reflected by ratio squared; belt mass excluded",
    criticalSpeed:"existing LeadScrew Euler-Bernoulli model, BK fixed/BF simple, bearing centre span, 80% first bending speed",
    candidateScope:"isolated motor comparison with unchanged mechanics; no supplier interchangeability, voltage/drive compatibility or candidate mass propagation to other axes claimed"},
   axes:baseReports,compactZAxis:compactZReport,baselineSpindle:{motor:installedMotor.designation,targetRpm:8000,targetOutputWatts:2200,
    spindleToMotorSpeedRatio:installedDrive.speedRatio(),motorRpmAtTarget:8000/installedDrive.speedRatio(),
    assumedBeltEfficiency:0.95,outputWattsAtRatedPoint:installedWatts,ratedPowerScreenPass:installedWatts>=2200,
    continuousAtTargetQualified:false,note:"Selected 1.5 kW motor does not meet the historical 2.2 kW tool-output target"},spindleAlternatives:variants,
   blockers:["Selected Leadshine motors are not yet qualified with installed drives and wiring", "Matched motor/drive torque-speed and thermal duty curves", "Measured friction, process cutting loads and production cycle", "Regenerative energy and bus/braking resistor limits", "Z brake supplier holding and emergency-stop data, power-loss validation", "Screw DN, buckling, axial load, bearings and coupling/belt capacity", "Servo tuning, stiffness, backlash and following error are not established by torque sizing"]};
  var dir=Sys.getEnv("MILL_AXIS_SIZING_OUTPUT");if(dir==null)dir=Sys.getCwd()+"/build/axis-sizing";
  if(!sys.FileSystem.exists(dir))sys.FileSystem.createDirectory(dir);
  sys.io.File.saveContent(dir+"/axis-sizing.json",Json.stringify(report,null,"  "));
  Sys.println("Axis sizing model checks passed: XYZ loads, nesting, spindle mass alternatives and drive speed");
  Sys.println("Axis sizing report: "+dir+"/axis-sizing.json");
 }
}
