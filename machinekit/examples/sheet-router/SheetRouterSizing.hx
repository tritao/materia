import machinekit.sizing.LinearDriveSizing;
import machinekit.sizing.LinearDriveSizing.LinearDriveInput;
import machinekit.sizing.RackDriveSizing;
import machinekit.motion.*;
import machinekit.transmission.*;
import machinekit.component.*;
import materia.assembly.AssemblyDefinition.AssemblyJointRole;

/** Reproducible preliminary sizing from the authored moving member masses, not the SR0 estimates. */
class SheetRouterSizing {
    public static final ACCELERATION=3.0;
    public static final CUTTING_N=SheetRouterCutting.forceEnvelopeN();
    public static final DRAG_N=30.0;
    public static final RESERVE=1.5;
    public static function moving(machine:SheetRouter,axis:String):{kg:Float,members:Array<{id:String,kg:Float}>} {
        var definition=machine.definition(),root:Null<String> = null,parents=machine.mateParents();
        for(joint in definition.joints)if(joint.id==axis&&joint.role==AssemblyJointRole.Tree)root=joint.child;
        if(root==null)throw "Sizing axis has no tree joint: "+axis;
        var total=0.0,members:Array<{id:String,kg:Float}> = [];
        for(member in machine.components()) {
            var cursor:Null<String> = member.id,seen:Map<String,Bool> = [];
            while(cursor!=null&&!seen.exists(cursor)&&cursor!=root) {seen.set(cursor,true);cursor=parents.get(cursor);}
            if(cursor!=root)continue;
            var mass=member.component.massProperties().mass;
            total+=mass;members.push({id:member.id,kg:mass});
        }
        // Count an entire routed chain when its moving carrier follows this axis,
        // unless that chain is already in the subtree. This bounds varying suspended mass.
        for(binding in machine.cableChainBindings) {
            var cursor:Null<String> = binding.carrier;
            while(cursor!=null&&cursor!=root)cursor=parents.get(cursor);
            var counted=false;for(member in members)if(member.id==binding.member)counted=true;
            if(cursor==root&&!counted) {
                var kg=machine.component(binding.member).massProperties().mass;
                total+=kg;members.push({id:binding.member+" (whole chain upper bound)",kg:kg});
            }
        }
        if(!(total>0))throw "Sizing found no moving mass";
        return {kg:total,members:members};
    }
    /** Each driven output member rotates about its local +Z axis through the origin.
     * CAD centroidal tensor plus parallel-axis term, kg mm² -> kg m².
     * Motor shaft is excluded: rotor nameplate inertia already includes it.
     */
    public static function rotary(machine:SheetRouter,ids:Array<String>):Float {
        var result=0.0;
        for(id in ids) {
            var mass=machine.component(id).massProperties(),tensor=mass.inertia;
            if(tensor==null)throw "Missing rotary inertia for "+id;
            result+=(tensor.zz+mass.mass*(mass.centreOfMass.x*mass.centreOfMass.x+mass.centreOfMass.y*mass.centreOfMass.y))*1e-6;
        }
        return result;
    }
    public static function report(machine:SheetRouter,axis:String):Dynamic {
        var mass=moving(machine,axis),vertical=axis=="z",suffix=axis=="y"?"YLeft":axis.toUpperCase();
        var motorPart=machine.component("motor"+suffix),rating:machinekit.motion.ServoMotor.ServoRating;
        // Read the actual wrapped motor rather than duplicating its catalogue selection.
        if(Std.isOfType(motorPart,ServoMotor))rating=cast(motorPart,ServoMotor).rating;
        else if(Std.isOfType(motorPart,BrakeServo))rating=cast(motorPart,BrakeServo).motor.rating;
        else throw "Sizing requires a selected servo motor: "+motorPart.designation;
        var reducer:PlanetaryReducer=vertical?null:cast machine.component("pinion"+suffix+"Reducer");
        var pinion:HelicalPinion=vertical?null:cast machine.component("pinion"+suffix);
        var rack:HelicalRack=vertical?null:cast machine.component("rack"+suffix);
        var preload:PreloadedPinionMount=vertical?null:cast machine.component("pinion"+suffix+"Preload");
        var ratio=vertical?1:reducer.n("ratio");
        var radius=vertical?cast(machine.component("screwZ"),LeadScrew).thread.lead/1000/(2*Math.PI):pinion.pitchDiameter()/2000;
        var rotating=rotary(machine,vertical?["screwZ","screwZCoupling"]:["pinion"+suffix,"pinion"+suffix+"OutputShaft"]);
        var input:LinearDriveInput={massKg:mass.kg,motors:axis=="y"?2:1,metresPerRadian:radius/ratio,
            efficiency:vertical?0.9:rack.n("efficiency")*reducer.n("efficiency"),
            rotaryInertiaKgM2:rotating/(ratio*ratio)+(vertical?0:reducer.n("inputInertia")),
            acceleration:ACCELERATION,velocity:vertical?machine.routerSpec.rapidZMmPerMinute/60000:40.0/60,strokeM:0.1,
            cuttingForceN:CUTTING_N,dragForceN:DRAG_N,gravityForceN:vertical?mass.kg*9.80665:0,
            rotaryDragNm:vertical?cast(machine.component("screwZNut"),LeadScrewNut).drag():0,reserve:RESERVE};
        var demand=LinearDriveSizing.demand(input,rating),failures=demand.failures.copy();
        var catalogueRack:Dynamic=null,toothReport:Dynamic=null,compliance:Dynamic=null,reducerLoads:Dynamic=null,belt:Dynamic=null,screwReport:Dynamic=null,accuracy:Dynamic=null;
        if(!vertical) {
            var tooth=RackDriveSizing.strength(pinion,demand.peakForcePerMotorN*RESERVE,1.5,preload.preload());
            toothReport=tooth;
            catalogueRack=SheetRouterRackRating.check(pinion,rack,demand.peakForcePerMotorN*RESERVE,tooth.flankNormalN);
            if(!(cast catalogueRack.passes:Bool))failures.push("rack: manufacturer feed-force rating including preload and mounting derating");
            var outputTorque=demand.peakForcePerMotorN*radius/rack.n("efficiency")+rotating*ACCELERATION/radius;
            var continuousOutput=(CUTTING_N+DRAG_N)/input.motors*radius/rack.n("efficiency");
            var bearingRadial=tooth.radialBearingN+preload.preload()*RESERVE;
            reducerLoads={peakOutputNm:outputTorque,ratedOutputNm:continuousOutput,axialN:tooth.axialN,radialN:bearingRadial,
                preloadN:preload.preload(),requiredPreloadN:tooth.separatingN};
            if(outputTorque*RESERVE>reducer.n("peakOutputTorque"))failures.push("reducer: peak output torque");
            if(continuousOutput*RESERVE>reducer.n("ratedOutputTorque"))failures.push("reducer: rated output torque");
            if(tooth.axialN>reducer.n("axialLoad"))failures.push("reducer: axial bearing load");
            if(bearingRadial>reducer.n("radialLoad"))failures.push("reducer: radial bearing load including preload");
            if(!preload.resists(tooth.separatingN))failures.push("preload: separating force at reserved peak acceleration");
            var rackCompliance=RackDriveSizing.compliance(pinion,rack,reducer,preload);compliance=rackCompliance;
            var axisStiffness=rackCompliance.stiffnessNPerM*input.motors;
            var cuttingError=(CUTTING_N+DRAG_N)/axisStiffness+rackCompliance.backlashM;
            var acceleratingError=demand.peakForcePerMotorN*RESERVE/rackCompliance.stiffnessNPerM+rackCompliance.backlashM;
            accuracy={axisStiffnessNPerM:axisStiffness,cuttingAndReversalM:cuttingError,reservedAccelerationAndReversalM:acceleratingError,
                cuttingDriveBudgetM:0.0001,accelerationDriveBudgetM:0.0002,
                idealDriveOnlyHz:Math.sqrt(axisStiffness/mass.kg)/(2*Math.PI),
                hingeComplianceFraction:rackCompliance.hingeMPerN/(1/rackCompliance.stiffnessNPerM),
                halfHingeStiffnessHz:Math.sqrt(input.motors/(1/rackCompliance.stiffnessNPerM+rackCompliance.hingeMPerN)/mass.kg)/(2*Math.PI),
                tenPercentHeavierHz:Math.sqrt(axisStiffness/(1.1*mass.kg))/(2*Math.PI)};
            if(cuttingError>0.0001)failures.push("rack drive: 0.1 mm cutting/reversal budget");
            if(acceleratingError>0.0002)failures.push("rack drive: 0.2 mm reserved acceleration/reversal budget");
            // HTD proposal: 20 T input, ratio-matched output, 5 mm pitch, 300 mm spans.
            // Assumed EA=100 kN; no supplier belt power/tooth or packaging qualification.
            var beltSpring=RackDriveSizing.beltCompliance(radius,20*ratio*0.005/(2*Math.PI),0.3,100000);
            var beltCompliance=RackDriveSizing.compliance(pinion,rack,null,preload,beltSpring);
            var drivenRadius=20*ratio*0.005/(2*Math.PI);
            beltCompliance.backlashM+=radius*TimingBelt.toothClearance(HTD5M)/1000/drivenRadius;
            belt={series:beltCompliance,axialRigidityN:100000,freeSpanM:0.3,inputTeeth:20,outputTeeth:20*ratio,
                drivenPitchDiameterMm:2*drivenRadius*1000,toothCapacityQualified:false,packagingQualified:false};
        }
        if(vertical) {
            var screw:LeadScrew=cast machine.component("screwZ"),nut:LeadScrewNut=cast machine.component("screwZNut");
            var critical=screw.criticalSpeed(Fixed,Free)*60/(2*Math.PI);
            var root=screw.thread.rootDiameter()/1000,length=screw.totalLength/1000;
            var buckling=Math.PI*Math.PI*210e9*Math.PI*Math.pow(root,4)/64/(4*length*length);
            var brake=motorPart.motorBrakeSpec();
            if(critical<demand.motorRpm)failures.push("Z screw: fixed/free critical speed");
            if(buckling<demand.peakForcePerMotorN*RESERVE)failures.push("Z screw: fixed/free Euler buckling");
            if(brake==null||brake.holdingTorque()<input.gravityForceN*radius*RESERVE)failures.push("Z brake: static holding torque");
            screwReport={nominalMm:screw.thread.screwDiameter,rootMm:screw.thread.rootDiameter(),lengthMm:screw.totalLength,
                criticalRpm:critical,bucklingN:buckling,stiffnessNPerM:screw.supportedStiffness(nut)*1000,
                backlashM:nut.backlash()/1000,
                alternative20CriticalRpm:critical*17/13,alternative20BucklingN:buckling*Math.pow(17.0/13,4),
                requested15MMinRpm:15000/screw.thread.lead,selectedRapidMmPerMinute:machine.routerSpec.rapidZMmPerMinute,brakeHoldingNm:brake==null?0:brake.holdingTorque(),
                requiredBrakeNm:input.gravityForceN*radius*RESERVE};
        }
        var alternatives:Array<Dynamic> = [];
        if(axis=="y")alternatives.push({selection:"dual 400 W Y, unchanged mechanical load",mountingQualified:false,
            demand:LinearDriveSizing.demand(input,ServoMotor.model("GENERIC-HV-SERVO-400W").rating)});
        if(axis=="x") {
            var original:LinearDriveInput=cast Reflect.copy(input);original.metresPerRadian=radius/5;
            original.rotaryInertiaKgM2=rotating/25+reducer.n("inputInertia");
            alternatives.push({selection:"original 400 W X at 5:1",demand:LinearDriveSizing.demand(original,ServoMotor.model("GENERIC-HV-SERVO-400W").rating)});
        }
        return {axis:axis,movingMassKg:mass.kg,movingMembers:mass.members,motor:rating.designation,input:input,
            motorRatings:{ratedNm:rating.ratedTorque,peakNm:rating.peakTorque,ratedRpm:rating.ratedSpeed*60/(2*Math.PI),maxRpm:rating.maxSpeed*60/(2*Math.PI),rotorInertiaKgM2:rating.rotorInertia},
            rotatingOutputInertiaKgM2:rotating,demand:demand,alternatives:alternatives,
            torqueUtilizationWithReserve:{peak:demand.peakNm*RESERVE/rating.peakTorque,rms:demand.rmsNm*RESERVE/rating.ratedTorque,sustained:demand.sustainedNm*RESERVE/rating.ratedTorque},driveAccuracy:accuracy,catalogueRack:catalogueRack,tooth:toothReport,reducer:reducerLoads,compliance:compliance,beltAlternative:belt,screw:screwReport,failures:failures};
    }
    public static function main():Void {
        var reports:Array<Dynamic> = [];
        for(length in [2600.0,3150.0]) {
            var machine=new SheetRouter(new SheetRouterSpec(length));
            var axes:Array<Dynamic> = [];
            for(axis in ["z","x","y"]) {var row=report(machine,axis);axes.push(row);Sys.println("SR4 Y="+length+" "+axis+" mass="+row.movingMassKg+" kg peak="+row.demand.peakNm+" Nm RMS="+row.demand.rmsNm+" Nm failures="+haxe.Json.stringify(row.failures));Sys.stdout().flush();}
            reports.push({workY:length,axes:axes});
        }
        var output=Sys.getEnv("SHEET_ROUTER_SIZING_OUTPUT");if(output==null)output="build/sheet-router-sr4";
        if(!sys.FileSystem.exists(output))sys.FileSystem.createDirectory(output);
        sys.io.File.saveContent(output+"/sizing.json",haxe.Json.stringify({basis:"assumed preliminary screening, not supplier qualification",
            loadSharing:"equal nominal Y motor shares; eccentric guide reactions/yaw loads remain SR5",
            duty:"100 mm loaded out-and-back, triangular/trapezoidal ramps, 1 s powered dwell; conservative magnitudes for acceleration and deceleration",
            exclusions:["Structural/rail/bolted-joint compliance and eccentric Y-side reactions (SR5)","Actual servo following error and tuning", "Gear fillet, fatigue life, lubrication, heat treatment and misalignment factors", "Screw supplier DN/life and support/coupling load ratings", "Belt tooth capacity and packaging"],
            choices:{reduction:"planetary: higher elastic stiffness than the specified EA=100 kN belt proposal; existing 80 mm housing vs 159/239 mm belt output pulleys",z:"1610 with 200 mm travel, 390 mm fixed/free shaft and 750 W braked servo at 10 m/min"},
            reserve:RESERVE,cuttingN:CUTTING_N,acceleration:ACCELERATION,variants:reports},null,"  "));
    }
}
