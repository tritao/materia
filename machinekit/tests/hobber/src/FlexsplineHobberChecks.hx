import cadkit.modeling.AssemblyModel;
import materia.assembly.AssemblyFrames as F;
import machinekit.assembly.PosedParts;
import machinekit.hobbing.*;
import stockkit.hobbing.HobbingAxes;
import gearkit.GearSpec;

class FlexsplineHobberChecks {
  static var assertions=0;
  static function check(ok:Bool,label:String):Void{assertions++;if(!ok)throw label;}
  static function near(a:Float,b:Float,e:Float,label:String):Void check(Math.abs(a-b)<=e,'$label: $a vs $b');
  static function modelForRuntime(machine:FlexsplineHobber):robotkit.model.RobotModel {
    var model=new AssemblyModel("mm");machine.addTo(model,"");
    var physical:cadbridge.AssemblySimulationBridge.AssemblyPhysicalData={metresPerUnit:0.001,
      parts:[for(part in machine.components()){id:part.id,materialId:"steel",volume:1000.0,centerOfMass:[0.0,0,0],
        inertia:[100000.0,0,0,0,100000.0,0,0,0,100000.0],density:7850.0}]};
    return cadbridge.AssemblySimulationBridge.toRobotModel(model.definition("hobber"),physical).model;
  }
  public static function run():Void {
    var machine=new FlexsplineHobber();machine.validate();
    assertions+=HobberDesignChecks.run(machine);
    var runtimeModel=modelForRuntime(machine);
    for(joint in [FlexsplineHobber.B,FlexsplineHobber.C]){
      var drive=runtimeModel.actuators.filter(a->switch a.transmission{case SimpleTransmission(id,_,_):id==joint;})[0];
      check(runtimeModel.encoderFor(drive)!=null&&drive.positionLoopRate==4000,"hobber gear drive has explicitly bound installed encoder and loop rate");
    }
    for(joint in [FlexsplineHobber.A,FlexsplineHobber.C])
      check(runtimeModel.coupledLimits(joint).requireAcceleration()>0,"auxiliary rotary positioning acceleration is stated");
    var compiled=robotkit.runtime.RobotRuntimeCompiler.compile(runtimeModel,new robotkit.profile.RobotProfile());
    var tilt=[for(i in 0...runtimeModel.joints.length)if(runtimeModel.joints[i].id==FlexsplineHobber.A)i][0];
    check(compiled.joints[tilt].servoStiffness>0,"uncoupled braked swivel gets physical hold gains");
    var cam=FlexsplineHobberCam.job(machine);
    check(cam.passes.length==1&&cam.tableRpm>0,"HB8 CAM fits installed travel and drive limits");
    var support=new MandrelRadialSupport(machine.mandrel,machine.spec.faceWidth);
    check(support.stiffness>0&&support.stiffness<support.sleeveStiffness&&support.stiffness<support.contactStiffness,"sleeve/contact series stiffness below each constituent");
    var installed=installedSupport(machine);
    check(installed.supportsRim&&installed.bandStart>=installed.sleeveStart&&installed.bandEnd<=installed.sleeveEnd,"assembled toothed band lies entirely on designed expansion sleeve");
    near(installed.radialGap,0.025,1e-9,"assembled bore/sleeve radial clearance matches design");
    var unsupported=new MandrelRadialSupport(machine.mandrel,machine.spec.faceWidth,0);
    check(!unsupported.supportsRim&&unsupported.stiffness==0,"rim outside sleeve gets no mandrel radial stiffness");
    var wide=new MandrelRadialSupport(machine.mandrel,20);
    check(!wide.supportsRim&&wide.stiffness==0,"rim wider than sleeve also falls back to wall stiffness");
    var oilValues=machine.mandrel.values().copy();oilValues.setNumber("oilBulkModulus",3000);
    var oilSupport=new MandrelRadialSupport(new ExpandingMandrel(oilValues),machine.spec.faceWidth);
    near(oilSupport.stiffness,support.stiffness,1e-9,"oil bulk modulus does not stiffen first-order ovalisation");
    near(oilSupport.uniformOilStiffness,2*support.uniformOilStiffness,1e-6,"uniform oil compression scales with bulk modulus");
    var ring=new stockkit.hobbing.RingCompliance(machine.spec.rootRadius()-0.3,0.6,machine.spec.faceWidth,210000,1000,"explicit test fixture stiffness");
    var sync=new stockkit.hobbing.HobbingErrorSource("sync","runtime observations","test zero measured following error",(t,a)->a);
    var errors=FlexsplineHobberErrors.sources(machine,t->20,ring,sync);
    check(errors.length==21,"HB11 installed parts expose independently selectable error sources");
    var mandrelError=errors.filter(e->e.id=="mandrel")[0];
    near(mandrelError.apply(0,new HobbingAxes(46,0,4,0,0,0)).pose().get_translation(0),46-machine.mandrel.eccentricity,1e-12,"part eccentricity controls physical work offset");
    check(errors.filter(e->e.id=="wall")[0].basis.indexOf("explicit test fixture stiffness")>=0,"wall support assumption stays in budget provenance");
    var forceStock=stockkit.RingStock.annulus(new stockkit.RingGrid(29,30,0,1,1,1));
    var forces=new stockkit.hobbing.HobbingForces(forceStock,30,20*Math.PI/180,1);
    try {
      forceStock.cutRegion([new stockkit.RingCut(0,0,1,0,0.001,0,0)]);
      forces.afterPass(new stockkit.hobbing.GashPass(0,0,0,0.5,1),new HobbingAxes(46,0,0,0,0,0),new HobbingAxes(46,0,0,0,0.625,0),16);
      check(FlexsplineHobberLoads.fits(machine,cam,forces),"HB10 installed ratings accept modest measured chip load with reserve");
      var options=FlexsplineHobberLoads.planOptions(forces);
      check(options.resistingEfforts.get("X")==forces.maxRadial,"radial load reaches PlanCheck");
      check(options.resistingEfforts.get(FlexsplineHobber.C)==forces.maxTableTorqueBound,"table reaction reaches PlanCheck");
      forceStock.cutRegion([new stockkit.RingCut(0,0,1,0.001,1,0,1)]);
      forces.afterPass(new stockkit.hobbing.GashPass(0,1,1,1.5,2),new HobbingAxes(46,0,0,0,0,0),new HobbingAxes(46,0,0,0,0.625,0),16);
      check(!FlexsplineHobberLoads.fits(machine,cam,forces),"HB10 installed ratings reject heavy measured chip load");
    }catch(e:Dynamic){forces.dispose();forceStock.dispose();throw e;}forces.dispose();forceStock.dispose();
    var model=new AssemblyModel("mm");machine.addTo(model,"");var state=model.initialState();
    var parts=new PosedParts();
    try {
      for(x in [machine.travel[0].lower,machine.travel[0].upper])for(y in [-10.0,10])for(z in [-10.0,190])for(a in [-Math.PI/12,Math.PI/12]){
        var b=0.73,c=1.13;state.setJoint("X",x);state.setJoint("Y",y);state.setJoint("Z",z);
        state.setJoint(FlexsplineHobber.A,a);state.setJoint(FlexsplineHobber.B,b);state.setJoint(FlexsplineHobber.C,c);state.forwardKinematics();
        var actual=machine.hobToWork(state),expected=new HobbingAxes(x,y,z,a,b,c).pose();
        near(actual.x,expected.get_translation(0),1e-8,"mechanical hob/work frame X");
        near(actual.y,expected.get_translation(1),1e-8,"mechanical hob/work frame Y");
        near(actual.z,expected.get_translation(2),1e-8,"mechanical hob/work frame Z");
        for(j in 0...3){var v=F.transformVector(actual,j==0?1:0,j==1?1:0,j==2?1:0);
          near(v.x,expected.get_rotation(j),1e-8,"mechanical rotation X");near(v.y,expected.get_rotation(3+j),1e-8,"mechanical rotation Y");near(v.z,expected.get_rotation(6+j),1e-8,"mechanical rotation Z");}
        var cutter=parts.posed(machine.hob,state.worldPose("hob"));
        try {
          for(id in ["mandrel","table/platen"]){var p=machine.components().filter(m->m.id==id)[0].component;
            var fixture=parts.posed(p,state.worldPose(id));
            try {
              // The integral flange is wider than the sleeve but below the tool.
              // Check the actual solids rather than a whole-fixture radial AABB.
              var volume=PosedParts.sharedVolume(cutter,fixture),gap=cutter.distance(fixture);
              check(volume<1e-6,'hob intersects $id at travel corner: $volume mm3');
              check(gap>0.05,'hob clearance from $id at travel corner: $gap mm');
            }catch(e:Dynamic){fixture.close();throw e;}fixture.close();}
        }catch(e:Dynamic){cutter.close();throw e;}cutter.close();
      }
      var joint=model.definition("hobber").joints;
      check(joint.filter(j->j.id==FlexsplineHobber.C).length==1,"independent table C joint");
      var definition=model.definition("hobber");
      check(definition.couplings.filter(k->k.source==FlexsplineHobber.B||k.target==FlexsplineHobber.C).length==0,"B/C have no mechanical gear coupling");
      for(axis in ["X","Y","Z"]){var j=joint.filter(k->k.id==axis)[0];check(j.limits.lower!=null&&j.limits.upper!=null,"bounded linear travel");}
      check(!FoundationIntegrity.checkMounts(machine,machine.precisionMounts).hasErrors(),"foundation precision mounts all land on inserts");
      for(axis in ["X","Y","Z"]){
        var coupling=definition.couplings.filter(k->k.id==axis+"lead")[0];
        near(Math.abs(coupling.ratio),2*Math.PI/5,1e-12,"ball screw derives rad/mm coupling");
        var drive=machine.actuatorFor(axis+"drive");check(drive.maxRate>0&&drive.maxEffort>0,"installed servo supplies finite drive limits");
      }
      var actuators=definition.actuators;if(actuators==null)throw "missing drives";
      check(actuators.length==6,"six powered drives exported");
      var bDrive=actuators.filter(a->a.joint==FlexsplineHobber.B)[0];check(bDrive.processVelocity!=null,"B is a process velocity spindle");
      var aDrive=actuators.filter(a->a.joint==FlexsplineHobber.A)[0];
      check(aDrive.brake!=null,"A has a spring-applied static clamp with release feedback");
      check(machine.billOfMaterials().lines().length>30,"machine BOM contains mechanical and drive hardware");
      check(machine.encode().length>0,"complete mechanical description serializes");
    }catch(e:Dynamic){parts.close();throw e;}
    parts.close();
    for(s in [new GearSpec(1,40,10,20*Math.PI/180),new GearSpec(0.5,100,15,20*Math.PI/180)]){
      var other=new FlexsplineHobber(s),depth=s.pitchRadius()+other.hob.design.pitchDiameter/2+s.profileShift*s.module;
      check(depth>=other.travel[0].lower&&depth<=other.travel[0].upper,"small/comparison blank reaches full depth with matching fixture");
    }
    Sys.println('FlexsplineHobberChecks: $assertions assertions passed');
  }
  /** Small physical integration fixture, separate from reference-budget acceptance. */
  static function wallCuts():Void {
    var machine=new FlexsplineHobber(),support=new MandrelRadialSupport(machine.mandrel,machine.spec.faceWidth);
    var spec=new GearSpec(1,20,2,20*Math.PI/180);
    var hob=new gearkit.HobDesign(gearkit.ConjugateRack.of(new gearkit.ToothProfile(spec)),1,32,4);
    var motion=new stockkit.hobbing.IdealHobbingJob(spec,hob,3000,1.1,0,1);
    var grid=stockkit.RingStock.fromGearBlank(spec,0.6,0.1,0.2);
    var passes=motion.schedule(grid.grid);grid.dispose();
    var force=100*Math.tan(spec.pressureAngle);
    var rows:Array<{factor:Float,thicknessChangeUm:Float,profileUm:Float}> = [];
    var inspection=new gearkit.GearInspection(spec,spec.pitchRadius());
    var baseline=new stockkit.hobbing.HobbingErrorJob(spec,hob,motion.axesAt,passes,[],0.6,0.1,0.2,-6,6,0.0001,inspection).run([]);
    for(factor in [0.3,1.0,3.0]){
      var ring=new stockkit.hobbing.RingCompliance(machine.spec.rootRadius()-0.3,0.6,machine.spec.faceWidth,210000,support.stiffness*factor,support.basis);
      var job=new stockkit.hobbing.HobbingErrorJob(spec,hob,motion.axesAt,passes,[ring.source("assumed reference support",t->force)],0.6,0.1,0.2,-6,6,0.0001,inspection);
      var measured=job.run(["wall"]),delta=0.0;
      for(i in 0...measured.thickness.length)delta=Math.max(delta,Math.abs(measured.thickness[i]-baseline.thickness[i])*1000);
      check(measured.thickness.length==spec.teeth&&delta>0,"physical wall-source cut changes thickness");
      rows.push({factor:factor,thicknessChangeUm:delta,profileUm:measured.value("Falpha")});
    }
    check(rows[0].thicknessChangeUm>rows[1].thicknessChangeUm&&rows[1].thicknessChangeUm>rows[2].thicknessChangeUm,"physical support sensitivity decreases thickness error");
    Sys.println("WALL_CUT_SENSITIVITY "+haxe.Json.stringify({rows:rows,forceN:force,
      scope:"m1 z20 face2 coarse integration fixture using reference support estimates; constant conservative force; not reference inspection or complete budget"}));
  }
  public static function installedSupport(machine:FlexsplineHobber):MandrelRadialSupport {
    var model=new AssemblyModel("mm");machine.addTo(model,"");var state=model.initialState();state.forwardKinematics();
    var pose=F.compose(F.inverse(state.worldPose("mandrel")),state.worldPose("blank"));
    var blank:machinekit.hobbing.FlexsplineCupBlank=cast machine.components().filter(m->m.id=="blank")[0].component;
    // The authored cup tooth band and mandrel share +Z and the table axis.
    check(Math.abs(pose.x)+Math.abs(pose.y)<1e-8,"rim and mandrel axes coincide");
    return new MandrelRadialSupport(machine.mandrel,machine.spec.faceWidth,pose.z,blank.parameterNumber("bore")/2);
  }
  static function supportSensitivity(meanForceBound:Float,rangeForceBound:Float,peakForce:Float,basis:String,meanMeasured:Bool,?history:Dynamic,forceValidated:Bool=true,?forceSettings:Dynamic):Void {
    var machine=new FlexsplineHobber(),estimate=installedSupport(machine);
    var s=machine.spec,profileLimit=gearkit.Iso1328.tolerance(s,"Falpha",6),pitchLimit=gearkit.Iso1328.tolerance(s,"fp",6);
    // Explicit physical design allocations. Numerical HB3 tolerance is unrelated.
    var profileBudget=profileLimit*0.2,pitchBudget=pitchLimit*0.2,thicknessTolerance=5.0;
    var bare=new stockkit.hobbing.RingCompliance(s.rootRadius()-0.3,0.6,s.faceWidth,210000,0,"unsupported reference ring");
    var seated=estimate.supportsRim&&estimate.contactPressure>peakForce/(Math.PI*(machine.mandrel.parameterNumber("outside")/2)*s.faceWidth);
    var rows=[];
    for(factor in [0.3,1.0,3.0]){
      var support=seated?estimate.stiffness*factor:0;
      var ring=new stockkit.hobbing.RingCompliance(bare.radius,0.6,s.faceWidth,210000,support,estimate.basis);
      var mean=ring.deflection(meanForceBound)*1000,variation=ring.deflection(rangeForceBound)*1000;
      var profile=2*variation,pitch=2*Math.tan(s.pressureAngle)*variation,thickness=2*Math.tan(s.pressureAngle)*mean;
      rows.push({factor:factor,supportNPerMm:support,meanDeflectionUm:mean,varyingDeflectionRangeBoundUm:variation,
        meanThicknessErrorBoundUm:thickness,xCompensationMagnitudeUm:mean,
        peakRadialDeflectionUm:ring.deflection(peakForce)*1000,
        radialStdDevDeflectionUm:history==null?null:ring.deflection(history.radialStdDevN)*1000,
        perPassMaxStepDeflectionUm:history==null?null:ring.deflection(history.perPassRadialMaxStepN)*1000,
        perPassRmsStepDeflectionUm:history==null?null:ring.deflection(history.perPassRadialRmsStepN)*1000,
        varyingProfileBoundUm:profile,varyingPitchBoundUm:pitch,
        passes:thickness<=thicknessTolerance&&profile<=profileBudget&&pitch<=pitchBudget});
    }
    var requiredTotal=Math.max(meanForceBound*1000*2*Math.tan(s.pressureAngle)/thicknessTolerance,
      Math.max(rangeForceBound*2000/profileBudget,rangeForceBound*1000*2*Math.tan(s.pressureAngle)/pitchBudget));
    var required=Math.max(0,requiredTotal-bare.ringStiffness);
    Sys.println("MANDREL_SENSITIVITY "+haxe.Json.stringify({basis:estimate.basis,forceBasis:basis,forceSettings:forceSettings,
      meanRadialForceN:meanMeasured?meanForceBound:null,meanRadialForceUpperBoundN:meanForceBound,
      varyingRadialRangeBoundN:rangeForceBound,peakRadialN:peakForce,radialStatistics:history,forceValidated:forceValidated,
      forceComponents:"all displacement/stiffness inputs are radial N; tangential Fc is used only for cutting load/power checks",
      supportsToothedRim:estimate.supportsRim,bandStartMm:estimate.bandStart,bandEndMm:estimate.bandEnd,
      sleeveStartMm:estimate.sleeveStart,sleeveEndMm:estimate.sleeveEnd,actualRadialFitGapMm:estimate.radialGap,
      supportApplied:seated,seatingScreen:seated,unsupportedRingNPerMm:bare.ringStiffness,
      estimatedNPerMm:estimate.stiffness,profileAllocationUm:profileBudget,pitchAllocationUm:pitchBudget,
      toothThicknessTolerancePlusMinusUm:thicknessTolerance,thicknessBasis:"Assumed physical flexspline tolerance +/-5 um; mean compensable by X infeed; no ISO thickness class",
      varyingBasis:"geometry bounds on the load range about its time mean, in the 20 percent wall share of HB11 class-6 profile and single-pitch limits",
      outsideIsoScope:gearkit.Iso1328.outsideScope(s),rows:rows,
      sufficientEffectiveSupportNPerMm:required,sufficientNominalFor03RangeNPerMm:required/0.3,
      acceptance:!forceValidated?"provisional: force-study acceptance or measured force data pending":rows.filter(r->!r.passes).length==0?"wall allocation passed, dependent on assumed support/contact and force bounds":"open: sufficient stiffness requirement; justify design or measure stiffness",
      scope:"wall-only quasistatic mean/variation bounds; full HB11 combined-source budget and physical reference inspection remain open"}));
    check(rows.length==3,"three stiffness sensitivity factors reported");
    if(seated)check(rows[0].meanDeflectionUm>rows[1].meanDeflectionUm&&rows[1].meanDeflectionUm>rows[2].meanDeflectionUm,"support sensitivity decreases mean deflection");
    else check(rows[0].supportNPerMm==0&&rows[2].supportNPerMm==0,"unseated or unsupported rim uses only cup wall ring stiffness");
  }
  /** Recheck the measured full-stroke envelope against installed continuous ratings.
      This is a static process-load reserve check, not a loaded dynamics simulation. */
  static function referenceLoads(path:String):Void {
    var report:Dynamic=haxe.Json.parse(sys.io.File.getContent(path));
    function number(key:String):Float {
      var value:Dynamic=Reflect.field(report,key);
      if(value==null||!Math.isFinite(value)||value<0)throw 'invalid reference load $key';
      return value;
    }
    var machine=new FlexsplineHobber(),cam=FlexsplineHobberCam.job(machine),reserve=0.8;
    check(report.scope=="complete ideal HB8 reference feed stroke, coarse ring grid","loads came from full reference stroke");
    near(number("seconds"),cam.cuttingSeconds,1e-6,"load stroke matches reference CAM duration");
    var mean:Dynamic=Reflect.field(report,"meanRadialN");
    var meanKnown=mean!=null&&Math.isFinite(mean)&&mean>=0;
    var history:Dynamic=Reflect.field(report,"radialVariation"),range=number("radialN");
    if(history!=null){var measuredRange:Dynamic=Reflect.field(history,"radialRangeN");
      if(measuredRange==null||!Math.isFinite(measuredRange)||measuredRange<0)throw "invalid measured radial variation";
      range=measuredRange;
    }
    // X compensation concerns the material-engaged mean. Retain the whole-stroke
    // mean separately for energy/global comparison, including entry/exit air.
    var thicknessMean:Dynamic=mean,thicknessMeanBasis=Std.string(report.meanBasis);
    if(history!=null){var activeMean:Dynamic=Reflect.field(history,"activeMeanRadialN");
      if(activeMean==null||!Math.isFinite(activeMean)||activeMean<0)throw "invalid active radial mean";
      thicknessMean=activeMean;thicknessMeanBasis="radial time mean over material-engaged windows; full-stroke mean including air retained separately";
    }
    var tolerance:Dynamic=Reflect.field(report,"sweepToleranceMm");
    var validated=(tolerance!=null&&tolerance<=0.00005)||Reflect.field(report,"fineFullStrokeValidated")==true
      ||(Reflect.field(report,"referenceForceAccepted")==true&&Reflect.field(report,"forceToleranceValidated")==true);
    var settings={threads:Reflect.field(report,"threads"),sweepToleranceMm:tolerance,pathToleranceMm:Reflect.field(report,"pathToleranceMm"),
      historyCsv:Reflect.field(report,"historyCsv"),fineFullStrokeValidated:Reflect.field(report,"fineFullStrokeValidated")==true,forceStudyValidated:validated,acceptanceBasis:Reflect.field(report,"acceptanceBasis")};
    Sys.println("REFERENCE_FORCE_INPUTS "+haxe.Json.stringify({peakTangentialN:number("forceN"),peakRadialN:number("radialN"),
      meanRadialN:meanKnown?mean:null,meanForThicknessRadialN:meanKnown?thicknessMean:null,meanForThicknessBasis:thicknessMeanBasis,radialVariation:history,
      variationBasis:history==null?"legacy run has no per-pass record: retain conservative 0..peak radial range":"measured radial window statistics and successive per-gash steps",
      sweepToleranceMm:tolerance,threads:Reflect.field(report,"threads"),fineFullStrokeValidated:Reflect.field(report,"fineFullStrokeValidated")==true,forceStudyValidated:validated,acceptanceBasis:Reflect.field(report,"acceptanceBasis")}));
    supportSensitivity(meanKnown?thicknessMean:number("radialN"),range,number("radialN"),
      meanKnown?thicknessMeanBasis:"legacy peak-only report: mean and varying range conservatively bounded by radial peak; actual mean unavailable",meanKnown,history,validated,settings);
    var b=machine.spindle.motor,c=machine.tableSpec;
    var bTorque=b.continuousTorque*reserve,bPower=b.continuousTorque*b.ratedSpeed*reserve;
    var cTorque=c.continuousTorque*reserve-c.number("sealDrag")-c.number("sealDragRipple")-c.number("coggingTorque");
    check(cam.rpm*2*Math.PI/60<=b.maxSpeed&&cam.tableRpm*2*Math.PI/60<=c.maxSpeed,"reference process speeds within installed limits");
    check(number("torqueNm")<=bTorque&&number("powerW")<=bPower,"measured reference spindle loads retain 20 percent reserve");
    check(number("tableTorqueBoundNm")<=cTorque,"measured reference table torque bound retains 20 percent reserve");
    var members=machine.components();
    var motor:machinekit.motion.ServoMotor=cast members.filter(m->m.id=="Xmotor")[0].component;
    var screw:machinekit.motion.BallScrew=cast members.filter(m->m.id=="Xscrew")[0].component;
    var nut:machinekit.motion.BallNut=cast members.filter(m->m.id=="Xnut")[0].component;
    var xForce=motor.rating.ratedTorque*2*Math.PI/(screw.thread.lead*0.001)*nut.efficiency()*reserve;
    check(number("radialN")<=xForce,"measured reference radial load retains X continuous thrust reserve");
    Sys.println("REFERENCE_RATINGS "+haxe.Json.stringify({spindleTorqueLimitNm:bTorque,spindlePowerLimitW:bPower,
      tableTorqueLimitNm:cTorque,xThrustLimitN:xForce,reserve:0.2,forceSettings:settings,
      scope:"installed assumed continuous ratings; static process loads only, inertia/gravity and tooth peaks excluded"}));
    Sys.println('Reference load ratings: $assertions checks passed');
  }
  public static function main():Void {
    if(Sys.args().indexOf("foot-checks")>=0){HobberFootChecks.run();return;}
    if(Sys.args().indexOf("purchased-table")>=0){HobberPurchasedTableChecks.run();return;}
    if(Sys.args().indexOf("loading-sizing")>=0){HobberLoadingSizing.run();return;}
    if(Sys.args().indexOf("table-tilt-checks")>=0){HobberTableTiltChecks.run();return;}
    if(Sys.args().indexOf("base-report")>=0){Sys.println(haxe.Json.stringify(FlexsplineHobberBaseReport.measure(new FlexsplineHobber())));return;}
    if(Sys.args().indexOf("loading-checks")>=0){HobberLoadingChecks.run();return;}
    if(Sys.args().length>1&&Sys.args()[0]=="layout-section"){FlexsplineHobberLayoutReport.writeSection(new FlexsplineHobber(),Sys.args()[1]);return;}
    if(Sys.args().length>0&&Sys.args()[0]=="ge-range"){HobberGERangeChecks.run();return;}
    if(Sys.args().length>1&&Sys.args()[0]=="scene-bounds"){Sys.println(haxe.Json.stringify(FlexsplineHobberLayoutReport.sceneBounds(Sys.args()[1])));return;}
    if(Sys.args().length>0&&Sys.args()[0]=="ge-frame"){HobberGEFrameChecks.run();return;}
    if(Sys.args().length>0&&Sys.args()[0]=="layout-metrics"){Sys.println(haxe.Json.stringify(FlexsplineHobberLayoutReport.measure(new FlexsplineHobber())));return;}
    if(Sys.args().length>0&&Sys.args()[0]=="installation"){HobberInstallationChecks.run();return;}
    if(Sys.args().length>0&&Sys.args()[0]=="interferences"){HobberInterferenceChecks.run();return;}
    if(Sys.args().length>0&&Sys.args()[0]=="reference-budget"){FlexsplineHobberBudget.run(Sys.args().slice(1));return;}
    if(Sys.args().length==1&&Sys.args()[0]=="wall-cuts"){wallCuts();return;}
    if(Sys.args().length==1&&Sys.args()[0]=="support-sensitivity"){supportSensitivity(100*Math.tan(20*Math.PI/180),100*Math.tan(20*Math.PI/180),100*Math.tan(20*Math.PI/180),"provisional assumed Fc=100 N design load; not a safe bound on the running cut; radial components conditional on this assumption",false,null,false);return;}
    if(Sys.args().length==2&&Sys.args()[0]=="reference-loads"){referenceLoads(Sys.args()[1]);return;}
    if(Sys.args().length>0&&(Sys.args()[0]=="scene"||Sys.args()[0]=="cutting")){
      var cutting=Sys.args()[0]=="cutting";
      var data=materia.project.SceneArtifact.decode(cutting?FlexsplineHobberPreview.cutting():FlexsplineHobberPreview.hobber());
      if(cutting){var moving=data.machineMotion;if(moving==null||moving.hobbing==null)throw "cutting job missing from artifact";
        check(moving.hobbing.teeth==40&&moving.hobbing.gashes==4,"coarse cutting job round-trips typed wire metadata");}
      var definition=data.assemblyDefinition;if(definition==null)throw "preview missing definition";
      var actuators=definition.actuators;if(actuators==null)throw "missing preview drives";
      check(actuators.length==6&&data.parts.length>30,"project preview exports complete mechanical scene and drives");
      Sys.println('FlexsplineHobber scene: $assertions assertions passed, ${data.parts.length} definitions');
    }else run();
  }
}
