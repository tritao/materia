import machinekit.sizing.LinearDriveSizing;
import machinekit.sizing.LinearDriveSizing.LinearDriveInput;
import machinekit.sizing.RackDriveSizing;
import machinekit.motion.*;
import machinekit.transmission.*;
import machinekit.component.ComponentValues;
import robotkit.model.*;
import robotkit.model.ActuatorDrive.ServoDrive;
import motionkit.robot.PlanCheck;
import motionkit.robot.PlanCheck.PlanCheckOptions;
import motionkit.robot.TrajectoryStream.StreamSegments;
import motionkit.trajectory.PlanDiagnostic.PlanDiagnosticKind;
import haxe.Int64;

class SheetRouterSizingChecks {
    static var assertions=0;
    static function check(ok:Bool,label:String):Void {assertions++;if(!ok)throw "SR4 sizing: "+label;}
    static function near(a:Float,b:Float,label:String):Void check(Math.isFinite(a)&&Math.abs(a-b)<1e-9*Math.max(1,Math.abs(b)),label+": "+a+" vs "+b);
    static function reject(fn:Void->Void,label:String):Void {var failed=false;try fn() catch(_:Dynamic)failed=true;check(failed,label);}
    static function analytical():Void {
        var motor=ServoMotor.model("GENERIC-HV-SERVO-750W").rating;
        var input:LinearDriveInput={massKg:100,motors:2,metresPerRadian:0.015/5,efficiency:0.9,rotaryInertiaKgM2:0.00002,
            acceleration:3,velocity:0.5,strokeM:0.1,cuttingForceN:300,dragForceN:30,gravityForceN:0,rotaryDragNm:0,reserve:1.5};
        var result=LinearDriveSizing.demand(input,motor);
        near(result.reflectedLinearInertiaKgM2,0.00045,"100 kg / two motors, 15 mm radius, 5:1 inertia hand calculation");
        near(result.externalInertiaKgM2,0.00047,"gearbox input inertia is external, not rotor inertia");
        near(result.inertiaRatio,0.00047/0.00012,"load/rotor inertia ratio");
        near(result.peakNm,315*0.003/0.9+(0.00002/0.9+0.00012)*1000,"load, loss and rotor acceleration torque hand calculation");
        var seconds=0.0,squares=0.0;
        for(phase in result.phases) {seconds+=phase.seconds;squares+=phase.seconds*phase.torqueNm*phase.torqueNm;}
        near(result.rmsNm,Math.sqrt(squares/seconds),"time-weighted RMS");
        near(RackDriveSizing.beltCompliance(0.02,0.1,0.3,100000),6e-8,"two-span reduction belt hand calculation");
        var pinion=new HelicalPinion(),rack=new HelicalRack(),gear=new PlanetaryReducer(),spring=new PreloadedPinionMount();
        var compliance=RackDriveSizing.compliance(pinion,rack,gear,spring),radius=pinion.pitchDiameter()/2000;
        near(compliance.reducerMPerN,radius*radius/12000,"output torsion reflected by pitch radius squared");
        near(compliance.hingeMPerN,Math.pow(Math.tan(pinion.transversePressureAngle()),2)/(40000*0.49),"hinge lever and pressure angle projection");
        near(compliance.backlashM,0.00004+radius*5*Math.PI/(180*60),"rack plus gearbox reversal allowance");
        var first=RackDriveSizing.strength(pinion,100),four=RackDriveSizing.strength(pinion,400);
        near(four.bendingMPa,4*first.bendingMPa,"Lewis load scaling");near(four.contactMPa,2*first.contactMPa,"Hertz square-root load scaling");
        var loaded=RackDriveSizing.strength(pinion,100,1.5,400);
        check(loaded.contactMPa>first.contactMPa && loaded.bendingMPa>first.bendingMPa,"radial preload also loads opposing tooth flanks");
        check(RackDriveSizing.strength(pinion,10000).failures.length>0,"overloaded teeth fail the screen");
        reject(()->{RackDriveSizing.strength(pinion,-1);},"negative tooth force rejected");
        reject(()->{RackDriveSizing.beltCompliance(0.02,0.1,0,100000);},"zero belt span rejected");
        input.efficiency=0;reject(()->{LinearDriveSizing.demand(input,motor);},"invalid efficiency rejected");
    }
    /** Independent one-axis reduction of the CAD sizing row, with actual assembled rack couplings.
     * It preserves translating mass, output rotary inertia, gearbox and rotor coordinates.
     */
    static function model(machine:SheetRouter,row:Dynamic,weak:Bool):RobotModel {
        var model=new RobotModel("sheet-router-Y-sizing"),base=model.addLink(new Link("base"));
        var table=model.addLink(new Link("moving"));table.mass=row.movingMassKg;
        var slide=model.addJoint(new Joint("y",JointType.Prismatic,base,table));slide.axis=[0,1,0];slide.limits=new JointLimits(-1,4,1,10000,3);
        var definition=machine.definition(),rating=ServoMotor.model(weak?"GENERIC-HV-SERVO-400W":"GENERIC-HV-SERVO-750W").rating;
        for(side in ["Left","Right"]) {
            var id="pinionY"+side,rotor=model.addLink(new Link(id));rotor.mass=0.1;
            rotor.inertiaTensor=[0.0,0,0,0,0.0,0,0,0,row.rotatingOutputInertiaKgM2];
            var shaft=model.addJoint(new Joint(id+"-turn",JointType.Continuous,base,rotor));shaft.limits=new JointLimits(-1e9,1e9);
            shaft.armature=(rating.rotorInertia+0.00002)*25;
            for(source in definition.couplings)if(source.target==shaft.id) {
                var coupling=new JointCoupling(source.id,"y",shaft.id,source.ratio*1000,0);
                var stiffness=source.stiffness,backlash=source.backlash,efficiency=source.efficiency;
                if(stiffness==null||backlash==null||efficiency==null)throw "Missing assembled rack allowances";
                coupling.stiffness=stiffness*1000;coupling.backlash=backlash/1000;coupling.efficiency=efficiency;
                if(source.assumptions!=null)coupling.assumptions=[for(value in source.assumptions){quantity:value.quantity,label:value.label}];
                model.addCoupling(coupling);
            }
            var actuator=new Actuator(rating.designation+"-"+side,null,null,Transmission.SimpleTransmission(shaft.id,5,0));
            actuator.drive=new ServoDrive(rating.ratedTorque,rating.peakTorque,rating.ratedSpeed,rating.maxSpeed,rating.rotorInertia,131072);
            actuator.efficiency=0.95;model.addActuator(actuator);
        }
        return model;
    }
    static function planner(machine:SheetRouter,row:Dynamic):Void {
        var options=new PlanCheckOptions();options.margin=1/SheetRouterSizing.RESERVE;options.steady=new SteadyLoads(SheetRouterSizing.DRAG_N);
        options.cuttingForce=SheetRouterSizing.CUTTING_N;options.cuttingFeedLimit=1;options.tolerance=0.0002;
        var selected=model(machine,row,false),checker=new PlanCheck(selected,["y"],options),loads=checker.axisLoads();
        check(loads.length==1&&loads[0].motors.length==2,"PlanCheck recognizes two geared rack motors on one leader");
        var expectedRotor=(0.00012+0.00002)+row.rotatingOutputInertiaKgM2/25;
        near(loads[0].motors[0].rotorInertia,expectedRotor,"PlanCheck reflects joint armature to motor once");
        near(loads[0].motors[0].ratio,5*selected.couplings[0].ratio,"rack/reducer speed Jacobian");
        var result=checker.checkSegments(new SizingAcceleration(),4,0.3);
        for(diagnostic in result.diagnostics)check(diagnostic.kind!=ServoPeakTorque&&diagnostic.kind!=ServoRatedTorque,"selected Y passes PlanCheck torque screen at 3 m/s²");
        var weak=new PlanCheck(model(machine,row,true),["y"],options).checkSegments(new SizingAcceleration(),4,0.3),named=0;
        for(diagnostic in weak.diagnostics)if(diagnostic.kind==ServoRatedTorque&&diagnostic.subject.indexOf("400W")>=0)named++;
        check(named==2,"both 400 W Y alternatives are rejected by name at 3 m/s²");
        // A follower's drag is torque at that shaft, and must be divided by its motor reduction.
        selected.couplings[0].drag=0.02;
        var withDrag=DriveLoads.forAxis(selected,"y");if(withDrag==null)throw "Missing Y drive load";
        near(withDrag.motors[0].drag,0.02/(5*0.95),"output drag reflected to the motor");
        options.tolerance=1e-6;
        var inaccurate=new PlanCheck(selected,["y"],options).checkSegments(new SizingAcceleration(),4,0.3),accuracy=false,assumed=false;
        for(diagnostic in inaccurate.diagnostics)if(diagnostic.kind==Accuracy) {
            accuracy=true;for(label in diagnostic.assumed)if(label.indexOf("hinge")>=0)assumed=true;
        }
        check(accuracy&&assumed,"PlanCheck accuracy includes named assumed mesh/reducer/hinge allowances");
    }
    public static function run():Void {
        analytical();
        for(length in [2600.0,3150.0]) {
            var machine=new SheetRouter(new SheetRouterSpec(length)),rows:Array<Dynamic> = [];
            for(axis in ["x","y","z"]) {
                var row=SheetRouterSizing.report(machine,axis);rows.push(row);
                var failures:Array<String> = row.failures;
                check(failures.length==0,axis+" selected sizing passes with 1.5 reserve: "+failures.join(", "));
                var input:LinearDriveInput=row.input;near(input.acceleration,3,"screen at requested acceleration");
                check(row.demand.inertiaRatio>0&&row.demand.rmsNm<=row.demand.peakNm,"finite inertia and RMS bounds");
                if(axis!="z") {
                    check(row.compliance.stiffnessNPerM>2e6,"drive stiffness exceeds the whole-machine target before structure is added");
                    check(row.compliance.backlashM<0.0001,"reported reversal allowance is below 0.1 mm");
                    check(row.driveAccuracy.cuttingAndReversalM<=0.0001 && row.driveAccuracy.reservedAccelerationAndReversalM<=0.0002,
                        "static and accelerating drive budgets include reversal");
                    var radius=cast(machine.component(axis=="x"?"pinionX":"pinionYLeft"),HelicalPinion).pitchDiameter()/2000;
                    near(input.metresPerRadian,radius/(axis=="x"?7.5:5),"selected axis ratio");
                } else {
                    check(row.screw.criticalRpm>=row.demand.motorRpm,"selected Z rapid below fixed/free critical speed");
                    check(row.screw.alternative20CriticalRpm<row.screw.requested15MMinRpm,"20 mm alone also cannot reach original 15 m/min target");
                }
            }
            var x=rows[0],y=rows[1],z=rows[2];
            check(y.movingMassKg>x.movingMassKg&&x.movingMassKg>z.movingMassKg,"moving masses include nested slides");
            var weak=LinearDriveSizing.demand(cast y.input,ServoMotor.model("GENERIC-HV-SERVO-400W").rating),named=false;
            for(failure in weak.failures)if(failure.indexOf("400W")>=0)named=true;
            check(named,"400 W Y fails CAD-mass duty screen by name");
            planner(machine,y);
            var definition=machine.definition(),old=machine.component("pinionXPreload"),before=0.0;
            for(coupling in definition.couplings)if(coupling.id=="pinionX-rack") {
                var stiffness=coupling.stiffness,backlash=coupling.backlash;
                if(stiffness==null||backlash==null)throw "Missing assembled rack allowances";
                before=stiffness;near(before*1000,x.compliance.stiffnessNPerM,"assembled compliance reaches the planner in SI");
                near(backlash/1000,x.compliance.backlashM,"assembled backlash reaches the planner in SI");
            }
            machine.replaceComponent("pinionXPreload",new PreloadedPinionMount());
            for(coupling in machine.definition().couplings)if(coupling.id=="pinionX-rack")check(coupling.stiffness!=null&&(cast coupling.stiffness:Float)<before/50,"part edit rebuilds hinge stiffness");
            machine.replaceComponent("pinionXPreload",old);
            machine.replaceComponent("pinionYLeftPreload",new PreloadedPinionMount(new ComponentValues().setNumber("springForce",10)));
            var inadequate=SheetRouterSizing.report(machine,"y"),preloadRejected=false,failures:Array<String> = inadequate.failures;
            for(failure in failures)if(failure.indexOf("preload:")==0)preloadRejected=true;
            check(preloadRejected,"preload below reserved peak separating force is rejected");
            for(record in machine.describe().machine.transmissions)if(record.coupling=="pinionX-rack") {
                var saved:machinekit.assembly.MachineAssemblyDescription.TransmissionRecord=haxeon.wire.JsonWire.decode(haxeon.wire.JsonWire.encode(record));
                check(saved.rackReducer==record.rackReducer&&saved.rackPreload==record.rackPreload,"rack allowance owners survive wire roundtrip");
                var copy=machinekit.assembly.DriveSystem.copyTransmission(record,id->"nested/"+id);
                check(copy.rackReducer=="nested/pinionXReducer"&&copy.rackPreload=="nested/pinionXPreload","rack allowance owners survive namespace copying");
            }
        }
        Sys.println("Sheet router SR4: "+assertions+" assertions passed");
    }
    public static function main():Void {run();if(Sys.args().indexOf("--checks-only")<0)SheetRouterSizing.main();}
}

/** A short cutting-acceleration chunk used by the live PlanCheck too. */
private class SizingAcceleration implements StreamSegments {
    public function new() {}
    public function count():Int return 1;
    public function jointCount():Int return 1;
    public function startNs(index:Int):Int64 return Int64.ofInt(0);
    public function durationNs(index:Int):Int64 return Int64.ofInt(100000000);
    public function degree(index:Int):Int return 2;
    public function coefficient(index:Int,joint:Int,power:Int):Float return power==2?1.5:0.0;
}
