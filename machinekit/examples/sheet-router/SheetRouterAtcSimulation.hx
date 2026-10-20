import SheetRouterAtc.RouterAtcAction;
import SheetRouterAtc.RouterAtcSignals;
import cadkit.modeling.AssemblyState;

typedef RouterAtcRunResult = {
    var tool:Int;
    var elapsedS:Float;
    var estimatedS:Float;
    var relativeTimeError:Float;
    var axisMotionS:Float;
    var analyticAxisMotionS:Float;
    var nativePoses:Int;
    var completedMoves:Int;
    var maximumTransferJumpMm:Float;
    var maximumGeometrySpacingMm:Float;
    var fault:Null<String>;
    var loaded:Int;
    var ack:Bool;
    var releaseCommand:Bool;
    var spindleRunCommand:Bool;
    var motionAborted:Bool;
    var recoveryRequired:Bool;
    var pocketOwner:Array<Int>;
}

/** Deterministic reference plant: independent pose-derived occupancy/presence,
 * timed pneumatic feedback, native Ruckig axis profiles and a latched probe.
 */
class SheetRouterAtcSimulation {
    public final atc:SheetRouterAtc;
    public var now(default,null):Float;
    public var q(default,null):Array<Float> = [0,0,0];
    public var nativeChecks(default,null):Int=0;
    public var maximumTransferJumpMm(default,null):Float=0;
    public var estimatedMotionS(default,null):Float=0;
    public var nativeMotionS(default,null):Float=0;
    public var completedMoves(default,null):Int=0;
    public var maximumGeometrySpacingMm(default,null):Float=0;
    final sampler:String;
    final useNative:Bool;
    final injected:String;
    final assembly:Null<SheetRouterAtcAssembly>;
    final state:Null<AssemblyState>;
    final clearance:Null<GantryClearanceChecks>;
    var startS:Float=0;
    var serial:Int=-1;
    var motionSerial:Int=-1;
    var motion:Null<SheetRouterAtcTiming>;
    var motionStart:Float=0;
    var motionRecorded:Bool=false;
    var instantDone:Bool=true;
    var measuring:Int=0;
    var probeLatched:Bool=false;
    var latchedGaugeZ:Float=Math.NaN;
    var latchedAt:Float=-1;
    var latchedSerial:Int=-1;
    var cover:Float;
    var shoe:Float;
    var releaseState:Bool=false;
    var releaseChanged:Float=-10;
    var lastQ:Array<Float> = [Math.NaN,0,0];
    var lastCover:Float=Math.NaN;
    var lastShoe:Float=Math.NaN;
    var lastOwner:Int=-1;
    var geometryStepMm:Float=50;
    final pocketCoordinates:Map<Int,Array<Float>> = new Map();

    public function new(atc:SheetRouterAtc,sampler:String,useNative:Bool=true,injected:String="",startS:Float=0,
        ?assembly:SheetRouterAtcAssembly,?state:AssemblyState,?clearance:GantryClearanceChecks) {
        this.atc=atc;this.sampler=sampler;this.useNative=useNative;this.injected=injected;this.now=startS;
        this.assembly=assembly;this.state=state;this.clearance=clearance;
        if(state!=null)q=[state.joint("x"),state.joint("y"),state.joint("z")];
        cover=atc.coverClose?atc.machine.rack.n("coverStroke"):0;
        shoe=atc.shoeRaise?0:atc.machine.shoe.n("liftStroke");
        for(t in atc.tools)pocketCoordinates.set(t.number,atc.pocketGauge(t.number));
    }
    static function distance(a:Array<Float>,b:Array<Float>):Float {
        var n=0.0;for(i in 0...3)n+=(a[i]-b[i])*(a[i]-b[i]);return Math.sqrt(n);
    }
    function truthLength(number:Int):Float return atc.tool(number).nominalGaugeTipMm+(number==3?0.003:number==1?-0.004:0);
    function root(number:Int,loaded:Int):Array<Float> return number==loaded?q:pocketCoordinates.get(number);
    function signals():RouterAtcSignals {
        var occupied=[for(i in 0...atc.pocketOwner.length)false],present=false;
        for(t in atc.tools) {
            if(injected=="missing_tool"&&t.number==3)continue;
            var here=root(t.number,atc.loaded);
            for(p in atc.tools)if(distance(here,pocketCoordinates.get(p.number))<0.1)occupied[p.pocket-1]=true;
            if(distance(here,q)<0.1)present=true;
        }
        if(injected=="occupied_return_pocket")occupied[atc.tool(1).pocket-1]=true;
        var clamped=!atc.release&&now-releaseChanged>=0.15;
        if(injected=="clamp_timeout"&&atc.loaded==0)clamped=false;
        var stopped=now-startS>=0.5;
        var triggered=false;
        if(atc.loaded!=0) {
            var end=atc.probeTarget(atc.loaded),edge=end[2]-1-(truthLength(atc.loaded)-atc.tool(atc.loaded).nominalGaugeTipMm);
            triggered=Math.abs(q[0]-end[0])<0.1&&Math.abs(q[1]-end[1])<0.1&&q[2]>=edge-1e-8;
        }
        if(injected=="setter_stuck")triggered=true;
        if(injected=="setter_no_trigger")triggered=false;
        return {sampledAtS:injected=="stale_observation"?now-0.2:now,
            vfdZeroSpeed:stopped&&injected!="vfd_at_speed",
            spindleStopped:stopped&&injected!="independent_running",
            airPressurePa:injected=="air_low"||(injected=="release_pressure_loss"&&releaseState)?500000:600000,
            clamped:clamped,released:atc.release&&now-releaseChanged>=0.15,toolPresent:present,
            coverOpen:cover<=1e-8,coverClosed:cover>=atc.machine.rack.n("coverStroke")-1e-8,shoeRaised:shoe<=1e-8,
            pocketOccupied:occupied,axesAtTarget:instantDone,motionSerial:injected=="wrong_motion_ack"?motionSerial-1:motionSerial,
            axisLeaseHeld:injected!="axis_lease_missing"&&!(injected=="axis_lease_lost"&&releaseState),
            setterTriggered:triggered,setterMotionSerial:injected=="stale_setter_edge"?latchedSerial-1:latchedSerial,
            setterEdgeAtS:latchedAt,setterGaugeZMm:latchedGaugeZ};
    }
    public function feedback():RouterAtcSignals return signals();
    function prepareMotion():Void {
        if(serial==atc.actionSerial)return;
        serial=atc.actionSerial;
        switch atc.action {
            case Move(_,target,feed):
                measuring=0;motionSerial=serial;instantDone=false;motionStart=now;motionRecorded=false;
                geometryStepMm=feed<50?5:Math.abs(target[2]-q[2])>1e-8?10:50;
                if(useNative) {motion=new SheetRouterAtcTiming(q,target,feed,sampler);estimatedMotionS+=motion.estimatedS;}
                else {q=target.copy();motion=null;instantDone=true;}
            case Measure(number):
                measuring=number;probeLatched=false;motionSerial=serial;instantDone=false;motionStart=now;motionRecorded=false;
                var target=atc.probeTarget(number);
                if(useNative) {
                    motion=new SheetRouterAtcTiming(q,target,1,sampler);
                    var stop=1/Math.sqrt(20000);estimatedMotionS+=SheetRouterAtcTiming.estimate(5+stop,1);
                } else {
                    q=target.copy();
                    if(injected!="setter_no_trigger") {
                        q[2]-=1+(truthLength(number)-atc.tool(number).nominalGaugeTipMm);
                        probeLatched=true;latchedGaugeZ=atc.machine.gaugeZero.z-q[2];latchedAt=now;latchedSerial=serial;
                    }
                    motion=null;instantDone=true;
                }
            case _:
        }
    }
    function move(dt:Float):Void {
        if(motion==null)return;
        var active:SheetRouterAtcTiming=cast motion;
        var before=q.copy();q=active.at(now-motionStart);
        if(measuring!=0&&!probeLatched&&injected!="setter_no_trigger") {
            var edge=atc.probeTarget(measuring);edge[2]-=1+truthLength(measuring)-atc.tool(measuring).nominalGaugeTipMm;
            if(before[2]<edge[2]&&q[2]>=edge[2]) {
                latchedAt=now-dt+dt*(edge[2]-before[2])/(q[2]-before[2]);latchedSerial=motionSerial;
                latchedGaugeZ=atc.machine.gaugeZero.z-edge[2];probeLatched=true;
                nativeMotionS+=latchedAt-motionStart;
                var stop=1/Math.sqrt(20000),end=edge.copy();end[2]+=stop;
                active=new SheetRouterAtcTiming(edge,end,1,sampler,1);motion=active;motionStart=latchedAt;motionRecorded=false;
                q=active.at(now-motionStart);
            }
        }
        instantDone=now-motionStart>=active.durationS;
        if(instantDone&&!motionRecorded) {nativeMotionS+=active.durationS;completedMoves++;motionRecorded=true;}
    }
    function checkGeometry(force:Bool=false):Void {
        if(assembly==null||state==null||clearance==null)return;
        var step=measuring!=0?1.0:geometryStepMm;
        if(!force&&Math.isFinite(lastQ[0])&&distance(lastQ,q)<step&&Math.abs(lastCover-cover)<50&&Math.abs(lastShoe-shoe)<10&&lastOwner==atc.loaded)return;
        state.setJoint("x",q[0]);state.setJoint("y",q[1]);state.setJoint("z",q[2]);
        state.setJoint("rackCoverLift",cover);state.setJoint("shoeLift",shoe);assembly.place(state,atc);
        try {clearance.check(state);assembly.verifySpecial(state,atc);}catch(error:Dynamic) {throw 'ATC clearance at '+q+' action='+atc.action+' loaded='+atc.loaded+': '+error;}
        if(Math.isFinite(lastQ[0]))maximumGeometrySpacingMm=Math.max(maximumGeometrySpacingMm,distance(lastQ,q));
        nativeChecks++;if(nativeChecks%50==0) {Sys.println('SR7 geometry $nativeChecks: '+atc.action+' q='+q);Sys.stdout().flush();}
        lastQ=q.copy();lastCover=cover;lastShoe=shoe;lastOwner=atc.loaded;
    }
    public function run(number:Int):RouterAtcRunResult {
        startS=now;var initialShoe=shoe;checkGeometry(true);
        atc.request(number,now,signals(),q);
        var dt=0.01;
        while(!atc.complete&&atc.fault==null&&now-startS<120) {
            prepareMotion();var oldLoaded=atc.loaded;
            now+=dt;
            var desiredCover=atc.coverClose?atc.machine.rack.n("coverStroke"):0;
            if(injected!="cover_timeout")cover+=Math.max(-200*dt,Math.min(200*dt,desiredCover-cover));
            var desiredShoe=atc.shoeRaise?0:atc.machine.shoe.n("liftStroke");shoe+=Math.max(-200*dt,Math.min(200*dt,desiredShoe-shoe));
            if(releaseState!=atc.release) {releaseState=atc.release;releaseChanged=now;}
            var wasDone=instantDone,wasLatched=probeLatched;move(dt);var observation=signals();atc.tick(now,observation);
            if(oldLoaded!=atc.loaded)for(t in atc.tools)maximumTransferJumpMm=Math.max(maximumTransferJumpMm,distance(root(t.number,oldLoaded),root(t.number,atc.loaded)));
            if(maximumTransferJumpMm>1e-5)throw "Tool body jumped during ownership transfer";
            if(atc.release&&(!observation.vfdZeroSpeed||!observation.spindleStopped||observation.airPressurePa<600000))throw 'Unsafe drawbar release output: $injected';
            checkGeometry(oldLoaded!=atc.loaded||(!wasDone&&instantDone)||(!wasLatched&&probeLatched));
        }
        if(now-startS>=120)throw 'ATC simulation did not terminate: '+atc.action;
        checkGeometry(true);
        var estimate=estimatedMotionS+0.5+initialShoe/200+2*atc.machine.rack.n("coverStroke")/200+0.3;
        var elapsed=now-startS,error=estimate>0?Math.abs(elapsed-estimate)/estimate:0;
        if(useNative&&atc.complete&&error>0.1)throw 'ATC change-time estimate exceeds 10%: $estimate / $elapsed';
        return {tool:number,elapsedS:elapsed,estimatedS:estimate,relativeTimeError:error,axisMotionS:nativeMotionS,
            analyticAxisMotionS:estimatedMotionS,nativePoses:nativeChecks,completedMoves:completedMoves,
            maximumTransferJumpMm:maximumTransferJumpMm,maximumGeometrySpacingMm:maximumGeometrySpacingMm,
            fault:atc.fault,loaded:atc.loaded,ack:atc.acknowledge('cnc.tool_change.$number',atc.requestSerial,now,signals()),
            releaseCommand:atc.release,spindleRunCommand:atc.spindleRun,motionAborted:atc.motionAbort,
            recoveryRequired:atc.recoveryRequired,pocketOwner:atc.pocketOwner.copy()};
    }
}
