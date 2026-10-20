import machinekit.gantry.Gantry;
import machinekit.gantry.GantrySpec;
import machinekit.gantry.GantrySpec.GantryDrive;
import machinekit.gantry.GantrySpec.GantryMotor;
import machinekit.gantry.GantrySpec.GantryStructure;
import machinekit.gantry.GantrySpec.GantryOptions;
import machinekit.component.*;
import machinekit.motion.*;
import machinekit.transmission.*;
import machinekit.structural.RectTube;
import machinekit.structural.RectTubeMember;
import materia.assembly.AssemblyFrames as F;
import cadkit.modeling.AssemblyState;

class SheetRouterNewAxesChecks {
    static var assertions=0;
    static function check(ok:Bool,label:String):Void {assertions++;if(!ok)throw label;}
    static function near(a:Float,b:Float,label:String):Void check(Math.isFinite(a)&&Math.abs(a-b)<1e-7,label+': '+a+' vs '+b);
    static function reject(fn:Void->Void,label:String):Void {var failed=false;try fn() catch(_:Dynamic) failed=true;check(failed,label);}
    static function fixture(?options:GantryOptions,?x:GantryDrive,?y:GantryDrive,?z:GantryDrive):Gantry {
        var rack:GantryDrive=HelicalRack(1.5,20,20,Math.acos(3*Math.PI/10));
        var screw:GantryDrive=Screw(new LeadScrewThread(machinekit.motion.LeadScrewThread.LeadScrewThreadFamily.Ball,16,5,1,machinekit.motion.LeadScrewThread.LeadScrewHand.RightHand,13));
        return new Gantry(new GantrySpec(300,200,100,x==null?rack:x,y==null?rack:y,z==null?screw:z,true,"HGR20",23,
            "HFS5-4040","HFS5-4040",false,None,0.5,new machinekit.power.DcVoltage(560),16,0,0,0,0,options));
    }
    static function options():GantryOptions return {
        motor:Servo("GENERIC-HV-SERVO-750W"),motorX:Servo("GENERIC-HV-SERVO-400W"),motorZ:BrakedServo("400"),
        reducer:new PlanetaryReducer(),reducerX:new PlanetaryReducer(new ComponentValues().setNumber("bore",14)),preload:new PreloadedPinionMount(),
        structure:Steel(new RectTube(80,80,4),new RectTube(100,80,4)),cableChain:new CableChain()
    };
    public static function run():Int {
        var gantry=fixture(options());
        gantry.check().throwIfErrors();check(true,'servo/steel assembly validates');
        check(Std.isOfType(gantry.component('frameFront'),RectTubeMember),'steel frame uses steel tube members');
        check(gantry.component('beam').materialSpec()=='steel','steel beam material');
        check(gantry.component('motorZ').motorBrakeSpec()!=null,'Z brake remains a typed physical capability');
        var scene=machinekit.assembly.AssemblyPreview.scene(gantry,'sr2-servo-gantry',256);
        var definition=scene.assemblyDefinition;
        if(definition==null)throw 'No compiled assembly';
        var motors=definition.actuators,couplings=definition.couplings;
        if(motors==null||couplings==null)throw 'No compiled drives';
        check(motors.length==4,'four compiled servo axes');
        var y=0;
        for(motor in motors) {
            check(motor.drive=='servo','servo drive family');
            if(motor.id=='motorZ') {check(motor.gearRatio==null,'direct Z screw');continue;}
            near(cast motor.gearRatio,5,'installed reducer ratio');near(cast motor.gearEfficiency,0.95,'installed reducer efficiency');
            var inertia=motor.rotorInertia;check(inertia!=null && inertia>0,'motor plus reducer inertia');
            for(coupling in couplings) if(coupling.target==motor.joint) {
                near(Math.abs(coupling.ratio),Math.PI/50,'100 mm/revolution rack');
                if(coupling.source=='y')y++;
            }
        }
        check(y==2,'both Y motors retain one linear leader for skew grouping');
        for(joint in definition.joints)if(joint.id=='y')near(cast joint.limits.rackingTolerance,0.5,'shared Y skew bound');
        var state=new AssemblyState(definition);
        var home=state.worldConnector('flange','face');
        state.setJoint('x',300);state.setJoint('y',200);state.setJoint('z',100);state.forwardKinematics();
        var far=state.worldConnector('flange','face');near(far.x-home.x,300,'servo FK X');near(far.y-home.y,200,'servo FK Y');near(far.z-home.z,-100,'servo FK Z');
        for(binding in gantry.cableChainBindings) {
            var first=gantry.cableChainEnd(binding.axis,0),last=gantry.cableChainEnd(binding.axis,binding.chain.n('travel'));
            var direction=gantry.axisDirection(binding.axis),travel=binding.chain.n('travel');
            near(last.x-first.x,direction[0]*travel,'chain end X follows axis');near(last.y-first.y,direction[1]*travel,'chain end Y follows axis');
            var parent=gantry.zeroPose(binding.carrier);near(first.x,parent.x,'chain starts at carriage X');near(first.y,parent.y,'chain starts at carriage Y');near(first.z,parent.z,'chain starts at carriage Z');
        }
        check(gantry.cableChainBindings.length==2,'both long-axis cable chains assembled');
        for(id in ['pinionXReducer','pinionYLeftReducer','pinionYRightReducer','pinionXPreload','pinionXOutputShaft','motorXPlate']) {
            var part=gantry.component(id).geometry();check(part.valid()&&part.volume()>0,'new axis hardware geometry '+id);part.close();
        }
        var source:PowerSupply=cast gantry.component('powerSupply');check(source.regenerativePower>0,'supply regeneration derived from motor curves');
        var model=cadbridge.AssemblySimulationBridge.toRobotModel(definition,cadbridge.AssemblyPhysicalPartView.fromSceneArtifact(scene),scene.assemblyState).model;
        check(model.validate().length==0,'servo gantry robot model validates');
        near(model.coupledLimits('y').requireVelocity(),5.0/3,'750 W servo speed through 5:1 reducer');
        check(model.coupledLimits('y').requireAcceleration()>0,'motor-derived acceleration includes physical mass');
        var smaller=options();smaller.motor=Servo('GENERIC-HV-SERVO-400W');smaller.reducer=new PlanetaryReducer(new ComponentValues().setNumber('bore',14));
        var revised=fixture(smaller);revised.check().throwIfErrors();
        near(AxisMotor.of(revised.component('motorX')).shaftDiameter,14,'400 W shaft fits matching reducer');
        var reducedScrew=options();reducedScrew.preload=null;
        var screw=new LeadScrewThread(machinekit.motion.LeadScrewThread.LeadScrewThreadFamily.MetricTrapezoidal,12,3);
        var screwMachine=fixture(reducedScrew,Screw(screw),Screw(screw));screwMachine.check().throwIfErrors();
        check(screwMachine.component('screwXReducer').reducerSpec()!=null,'servo screw reducer stage');
        var ball=new LeadScrewThread(machinekit.motion.LeadScrewThread.LeadScrewThreadFamily.Ball,16,5,1,machinekit.motion.LeadScrewThread.LeadScrewHand.RightHand,13);
        var ballMachine=fixture(reducedScrew,Screw(ball),Screw(ball));ballMachine.check().throwIfErrors();
        check(ballMachine.component('screwXFixedBracketRiser')!=null,'reduced ball screw bearing clears reducer housing');
        var bad=options();bad.reducer=new PlanetaryReducer(new ComponentValues().setNumber('bore',14));
        reject(()->fixture(bad),'reducer shaft mismatch rejected');
        var starved=options();starved.supplyCurrent=new machinekit.power.DcCurrent(1);
        reject(()->{var model=new cadkit.modeling.AssemblyModel('mm');fixture(starved).addTo(model,'');},'undersized shared supply rejected');
        var belt=options();reject(()->fixture(belt,Belt(TimingBeltProfile.GT2,20,9)),'unsupported reducer/belt combination rejected');
        Sys.println('SR2 servo/steel checks: '+assertions+' assertions passed');return assertions;
    }
}
