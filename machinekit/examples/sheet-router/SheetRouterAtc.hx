import materia.assembly.AssemblyFrames as F;

typedef RouterTool = {
    var number:Int;
    var pocket:Int;
    var diameterMm:Float;
    var nominalGaugeTipMm:Float;
    var measuredGaugeTipMm:Null<Float>;
    var description:String;
    var holderId:String;
    var cutterId:String;
    /** Completed cutting cycles, updated by the host after a successful job. */
    var lifeCount:Int;
}
typedef RouterAtcSignals = {
    var sampledAtS:Float;
    var vfdZeroSpeed:Bool;
    var spindleStopped:Bool;
    var airPressurePa:Float;
    var clamped:Bool;
    var released:Bool;
    var toolPresent:Bool;
    var coverOpen:Bool;
    var coverClosed:Bool;
    var shoeRaised:Bool;
    var pocketOccupied:Array<Bool>;
    var axesAtTarget:Bool;
    var axisLeaseHeld:Bool;
    var motionSerial:Int;
    var setterTriggered:Bool;
    var setterMotionSerial:Int;
    var setterEdgeAtS:Float;
    /** Gauge Z latched by the setter edge, in machine world mm. */
    var setterGaugeZMm:Float;
}
enum RouterAtcAction {
    Set(name:String,value:Bool);
    Wait(name:String,timeoutS:Float);
    Move(name:String,q:Array<Float>,feedMmPerS:Float);
    ReturnTool(number:Int);
    TakeTool(number:Int);
    Measure(number:Int);
    Finish(number:Int);
}

/** Router-specific exchange policy. A host owns axis execution and sensor IO;
 * tool ownership changes only at confirmed physical transfer stages.
 */
class SheetRouterAtc {
    public static function decodeTools(value:Dynamic):Array<RouterTool> {
        var rows:Array<Dynamic> = cast value;
        return [for(row in rows) {
            number:cast row.number,pocket:cast row.pocket,diameterMm:cast row.diameterMm,
            nominalGaugeTipMm:cast row.nominalGaugeTipMm,
            measuredGaugeTipMm:row.measuredGaugeTipMm==null?null:cast row.measuredGaugeTipMm,
            description:cast row.description,
            holderId:row.holderId==null?"nominal-ISO30-ER32":cast row.holderId,
            cutterId:row.cutterId==null?"unqualified-cutter":cast row.cutterId,
            lifeCount:row.lifeCount==null?0:cast row.lifeCount
        }];
    }
    public final machine:SheetRouter;
    public final tools:Array<RouterTool>;
    public final pocketOwner:Array<Int>;
    public var loaded(default,null):Int;
    public var target(default,null):Int=0;
    public var action(default,null):Null<RouterAtcAction>;
    public var actionSerial(default,null):Int=0;
    public var actionStartedS(default,null):Float=0;
    public var fault(default,null):Null<String>;
    public var complete(default,null):Bool=false;
    public var release(default,null):Bool=false;
    public var coverClose(default,null):Bool=true;
    public var shoeRaise(default,null):Bool=false;
    public var spindleRun(default,null):Bool=false;
    public var motionAbort(default,null):Bool=false;
    public var recoveryRequired(default,null):Bool=false;
    public var measurementMotionSerial(default,null):Int=0;
    public var requestSerial(default,null):Int=0;
    public var trace(default,null):Array<String> = [];
    final queue:Array<RouterAtcAction> = [];
    var previousTime:Float=-1;
    var measurementStartS:Float=0;
    final checkpoint:Null<String->Void>;
    public static inline var minimumAirPa:Float=600000;
    public static inline var maximumObservationAgeS:Float=0.1;
    public static inline var taperClearanceMm:Float=55;

    public function new(machine:SheetRouter,tools:Array<RouterTool>,loaded:Int=1,?checkpoint:String->Void) {
        this.machine=machine;this.tools=tools;this.loaded=loaded;this.checkpoint=checkpoint;
        pocketOwner=[for(i in 0...machine.rack.parameterCount("pockets"))0];
        var numbers:Map<Int,Bool> = new Map();
        for(t in tools) {
            if(t.lifeCount<0||t.holderId==null||t.holderId.length==0||t.cutterId==null||t.cutterId.length==0
                ||t.number<=0||numbers.exists(t.number)||t.pocket<1||t.pocket>pocketOwner.length||pocketOwner[t.pocket-1]!=0
                ||!Math.isFinite(t.nominalGaugeTipMm)||t.nominalGaugeTipMm<=0||t.nominalGaugeTipMm>machine.routerSpec.longestTool
                ||!Math.isFinite(t.diameterMm)||t.diameterMm<=0)throw "Invalid or duplicate router tool-table entry";
            if(t.measuredGaugeTipMm!=null&&(!Math.isFinite(cast t.measuredGaugeTipMm)||Math.abs((cast t.measuredGaugeTipMm:Float)-t.nominalGaugeTipMm)>0.5))throw "Invalid measured tool length";
            numbers.set(t.number,true);pocketOwner[t.pocket-1]=t.number;
        }
        if(loaded!=0)pocketOwner[tool(loaded).pocket-1]=0;
    }
    public function tool(number:Int):RouterTool {
        for(t in tools)if(t.number==number)return t;
        throw 'Unknown router tool T$number';
    }
    public function pocketGauge(number:Int):Array<Float> {
        var t=tool(number),fork=machine.component('fork${t.pocket}');
        var p=F.compose(machine.zeroPose('fork${t.pocket}'),fork.connector("pocket").frame);
        return checked(machine.gaugeCoordinates(F.translation(p.x,p.y,p.z+8)));
    }
    public function approachGauge(number:Int,z:Float):Array<Float> {
        var t=tool(number),fork=machine.component('fork${t.pocket}');
        var p=F.compose(machine.zeroPose('fork${t.pocket}'),fork.connector("approach").frame);
        var q=machine.gaugeCoordinates(F.translation(p.x,p.y,0));q[2]=z;return checked(q);
    }
    function checked(q:Array<Float>):Array<Float> {
        if(q.length!=3||!Math.isFinite(q[0])||!Math.isFinite(q[1])||!Math.isFinite(q[2])
            ||q[0]<0||q[0]>1600||q[1]<0||q[1]>machine.routerSpec.workY+300||q[2]<0||q[2]>200)throw 'ATC pose outside axis limits: '+q;
        return q;
    }
    function fail(name:String):Void {
        fault=name;release=false;spindleRun=false;motionAbort=true;recoveryRequired=true;
        complete=false;trace.push('FAULT:$name');queue.resize(0);action=null;
    }
    function observe(now:Float,s:RouterAtcSignals):Bool {
        if(!Math.isFinite(now)||now<previousTime||!Math.isFinite(s.sampledAtS)||s.sampledAtS>now||now-s.sampledAtS>maximumObservationAgeS) {
            fail("stale_or_invalid_observation");return false;
        }
        previousTime=now;
        if(s.pocketOccupied.length!=pocketOwner.length||!Math.isFinite(s.airPressurePa)||s.airPressurePa<0) {fail("invalid_sensor_observation");return false;}
        if(s.clamped&&s.released) {fail("drawbar_feedback_conflict");return false;}
        if(release&&(!s.vfdZeroSpeed||!s.spindleStopped||s.airPressurePa<minimumAirPa)) {fail("release_interlock_lost");return false;}
        var openingDrawbar=switch action {case Wait("released",_):true;case _:false;};
        if(release&&!s.released&&!openingDrawbar) {fail("release_feedback_lost");return false;}
        if(action!=null) {
            if(!s.axisLeaseHeld) {fail("axis_lease_lost");return false;}
            var stopping=switch action {case Wait("spindle_stop",_):true;case _:false;};
            if(!stopping&&(!s.vfdZeroSpeed||!s.spindleStopped)) {fail("spindle_restart_during_atc");return false;}
            if(loaded!=0&&!release&&(!s.clamped||!s.toolPresent)) {fail("clamp_feedback_lost");return false;}
            var raising=switch action {case Wait("shoe_raised",_):true;case _:false;};
            if(shoeRaise&&!s.shoeRaised&&!raising) {fail("shoe_feedback_lost");return false;}
            var opening=switch action {case Wait("cover_open",_):true;case _:false;};
            if(!coverClose&&!s.coverOpen&&!opening) {fail("cover_feedback_lost");return false;}
        }
        return true;
    }
    function persist():Bool {
        if(checkpoint==null) {fail("persistence_unavailable");return false;}
        try {checkpoint(stateJson());return true;}catch(error:Dynamic) {fail("persistence_failed");return false;}
    }
    public function request(number:Int,now:Float,s:RouterAtcSignals,currentQ:Array<Float>):Void {
        if(recoveryRequired||action!=null||queue.length>0)throw "ATC needs an idle, recovered machine";
        fault=null;complete=false;target=number;trace=[];requestSerial++;
        if(!observe(now,s))return;
        if(!s.axisLeaseHeld) {fail("axis_lease_missing");return;}
        if(checkpoint==null) {fail("persistence_unavailable");return;}
        var incoming:RouterTool;
        try {incoming=tool(number);}catch(error:Dynamic) {fail("unknown_tool");return;}
        if(number!=loaded&&(!s.pocketOccupied[incoming.pocket-1]||pocketOwner[incoming.pocket-1]!=number)) {fail("missing_tool");return;}
        if(loaded!=0&&s.pocketOccupied[tool(loaded).pocket-1]) {fail("occupied_return_pocket");return;}
        for(i in 0...pocketOwner.length)if(s.pocketOccupied[i]!=(pocketOwner[i]!=0)) {fail("inventory_mismatch");return;}
        if(loaded!=0&&(!s.clamped||!s.toolPresent)) {fail("outgoing_tool_unconfirmed");return;}
        if(number==loaded) {
            if(incoming.measuredGaugeTipMm==null) {fail("tool_length_unmeasured");return;}
            complete=persist();return;
        }
        checked(currentQ);
        if(currentQ[1]>machine.routerSpec.workY) {fail("change_start_outside_work_area");return;}
        queue.push(Set("spindle",false));queue.push(Wait("spindle_stop",10));queue.push(Wait("air",2));
        queue.push(Set("shoe_raise",true));queue.push(Wait("shoe_raised",3));
        queue.push(Move("retract",[currentQ[0],currentQ[1],0],10000/60));
        queue.push(Set("cover_close",false));queue.push(Wait("cover_open",5));
        if(loaded!=0) {
            var seat=pocketGauge(loaded),entry=approachGauge(loaded,0);
            queue.push(Move("return_approach",entry,40000/60));
            queue.push(Move("return_above",[seat[0],seat[1],seat[2]-taperClearanceMm],40000/60));
            queue.push(Move("return_lower",seat,1000/60));
            queue.push(Wait('pocket_occupied:${tool(loaded).pocket}',2));
            queue.push(Set("release",true));queue.push(Wait("released",2));queue.push(ReturnTool(loaded));
            queue.push(Move("withdraw_taper",[seat[0],seat[1],seat[2]-taperClearanceMm],1000/60));
            queue.push(Wait("spindle_empty",2));
            queue.push(Move("return_lift_clear",[seat[0],seat[1],0],10000/60));
            queue.push(Move("return_back_out",approachGauge(loaded,0),40000/60));
        } else {queue.push(Set("release",true));queue.push(Wait("released",2));}
        var seat=pocketGauge(number),high=seat[2]-taperClearanceMm;
        queue.push(Move("pickup_approach",approachGauge(number,0),40000/60));
        queue.push(Move("pickup_above",[seat[0],seat[1],high],1000/60));
        queue.push(Wait('pocket_occupied:${incoming.pocket}',2));
        queue.push(Move("insert_taper",seat,1000/60));
        queue.push(Set("release",false));queue.push(Wait("clamped",2));queue.push(TakeTool(number));
        queue.push(Move("lift_from_fork",[seat[0],seat[1],high],1000/60));
        queue.push(Wait('pocket_empty:${incoming.pocket}',2));
        queue.push(Move("pickup_raise",[seat[0],seat[1],0],10000/60));
        queue.push(Move("leave_fork",approachGauge(number,0),40000/60));
        var setter=F.compose(machine.zeroPose("setter"),machine.component("setter").connector("trigger").frame);
        var q=checked(machine.gaugeCoordinates(F.translation(setter.x,setter.y,setter.z+incoming.nominalGaugeTipMm+5)));
        queue.push(Move("setter_approach",[q[0],q[1],0],40000/60));queue.push(Move("setter_lower",q,10000/60));
        queue.push(Wait("setter_clear",1));queue.push(Measure(number));queue.push(Wait("probe_stopped",2));
        queue.push(Move("setter_retract",[q[0],q[1],0],10000/60));
        queue.push(Move("park",[0,0,0],40000/60));queue.push(Set("cover_close",true));queue.push(Wait("cover_closed",5));
        queue.push(Finish(number));if(persist())advance(now,s);
    }
    function advance(now:Float,s:RouterAtcSignals):Void {
        while(queue.length>0&&fault==null) {
            action=queue.shift();actionStartedS=now;actionSerial++;trace.push(Std.string(action));
            switch action {
                case Set(name,value):
                    switch name {
                        case "release":
                            if(value&&(!s.vfdZeroSpeed||!s.spindleStopped||s.airPressurePa<minimumAirPa)) {fail("release_interlock_denied");return;}
                            release=value;
                        case "spindle":spindleRun=value;
                        case "cover_close":coverClose=value;
                        case "shoe_raise":shoeRaise=value;
                        case _:throw 'Unknown ATC output $name';
                    }
                case ReturnTool(number):pocketOwner[tool(number).pocket-1]=number;loaded=0;if(!persist())return;
                case TakeTool(number):
                    if(!s.clamped||s.released||!s.toolPresent) {fail("incoming_tool_unconfirmed");return;}
                    loaded=number;pocketOwner[tool(number).pocket-1]=0;if(!persist())return;
                case Finish(number):
                    if(loaded!=number||release||!s.clamped||!s.toolPresent||tool(number).measuredGaugeTipMm==null) {fail("completion_unconfirmed");return;}
                    action=null;if(persist())complete=true;return;
                case Measure(_):measurementStartS=now;measurementMotionSerial=actionSerial;return;
                case _:return;
            }
        }
        action=null;
    }
    public function tick(now:Float,s:RouterAtcSignals):Void {
        if(fault!=null||action==null)return;
        if(!observe(now,s))return;
        var done=false;
        switch action {
            case Wait(name,timeout):
                done=switch name {
                    case "spindle_stop":s.vfdZeroSpeed&&s.spindleStopped;
                    case "air":Math.isFinite(s.airPressurePa)&&s.airPressurePa>=minimumAirPa;
                    case "shoe_raised":s.shoeRaised;
                    case "cover_open":s.coverOpen&&!s.coverClosed;
                    case "cover_closed":s.coverClosed&&!s.coverOpen;
                    case "released":s.released&&!s.clamped;
                    case "clamped":s.clamped&&!s.released&&s.toolPresent;
                    case "spindle_empty":!s.toolPresent;
                    case "setter_clear":!s.setterTriggered;
                    case "probe_stopped":s.axesAtTarget&&s.motionSerial==measurementMotionSerial;
                    case _:
                        var parts=name.split(":");
                        if(parts.length!=2)false;else {
                            var pocket=Std.parseInt(parts[1]);
                            if(pocket==null||pocket<1||pocket>s.pocketOccupied.length)false;
                            else if(parts[0]=="pocket_occupied")s.pocketOccupied[pocket-1];
                            else if(parts[0]=="pocket_empty")!s.pocketOccupied[pocket-1];else false;
                        }
                };
                if(!done&&now-actionStartedS>=timeout) {fail(name+"_timeout");return;}
            case Move(name,_,_):
                done=s.axesAtTarget&&s.motionSerial==actionSerial;
                if(!done&&now-actionStartedS>60) {fail(name+"_timeout");return;}
            case Measure(number):
                if(s.setterTriggered&&s.setterMotionSerial==actionSerial&&s.setterEdgeAtS>=measurementStartS&&s.setterEdgeAtS<=now) {
                    var trigger=F.compose(machine.zeroPose("setter"),machine.component("setter").connector("trigger").frame);
                    var length=s.setterGaugeZMm-trigger.z;
                    if(!Math.isFinite(length)||Math.abs(length-tool(number).nominalGaugeTipMm)>0.5) {fail("tool_length_out_of_range");return;}
                    tool(number).measuredGaugeTipMm=length;done=true;
                } else if(s.axesAtTarget&&s.motionSerial==actionSerial) {fail("setter_no_trigger");return;}
                else if(now-measurementStartS>10) {fail("setter_timeout");return;}
            case _:throw "ATC immediate action remained active";
        }
        if(done)advance(now,s);
    }
    /** Only an active, freshly requested sequence may command cover/shoe motion.
     * The host always applies release/spindle OFF and motionAbort on a fault,
     * but must not project default actuator targets from a loaded checkpoint.
     */
    public function actuatorCommandsEnabled():Bool return action!=null&&fault==null&&!recoveryRequired;
    /** WaitInput acknowledgement stays false through every unconfirmed stage. */
    public function acknowledge(channel:String,request:Int,now:Float,s:RouterAtcSignals):Bool {
        if(!complete||fault!=null||request!=requestSerial||channel!='cnc.tool_change.$loaded')return false;
        if(!observe(now,s))return false;
        if(!s.clamped||s.released||!s.toolPresent) {fail("completion_feedback_lost");return false;}
        for(i in 0...pocketOwner.length)if(s.pocketOccupied[i]!=(pocketOwner[i]!=0)) {fail("inventory_mismatch");return false;}
        return true;
    }
    public function stateJson():String return haxe.Json.stringify({version:1,loaded:loaded,pocketOwner:pocketOwner,tools:tools,
        recoveryRequired:recoveryRequired||action!=null||queue.length>0,target:target,requestSerial:requestSerial,actionSerial:actionSerial},null,"  ");
    /** Probe only 1 mm past the coarse nominal trigger. Tools must be preset
     * within ±0.5 mm; even the longest allowed tool retains 1 mm setter stroke.
     */
    public function probeTarget(number:Int):Array<Float> {
        var p=F.compose(machine.zeroPose("setter"),machine.component("setter").connector("trigger").frame);
        return checked(machine.gaugeCoordinates(F.translation(p.x,p.y,p.z+tool(number).nominalGaugeTipMm-1)));
    }
    public static function fromJson(machine:SheetRouter,json:String,checkpoint:String->Void):SheetRouterAtc {
        var saved:Dynamic=haxe.Json.parse(json);
        if(saved.version!=1||(saved.recoveryRequired!=true&&saved.recoveryRequired!=false))throw "Unsupported or incomplete router tool-state version";
        var result=new SheetRouterAtc(machine,decodeTools(saved.tools),cast saved.loaded,checkpoint);
        var dynamicOwners:Array<Dynamic> = cast saved.pocketOwner;
        var owners:Array<Int> = [for(value in dynamicOwners)(cast value:Int)];
        if(owners.length!=result.pocketOwner.length)throw "Tool-state pocket count mismatch";
        for(i in 0...owners.length)if(owners[i]!=result.pocketOwner[i])throw "Tool-state ownership mismatch";
        result.recoveryRequired=saved.recoveryRequired==true;
        if(result.recoveryRequired)result.fail("recovery_required");
        result.requestSerial=saved.requestSerial==null?0:cast saved.requestSerial;
        result.actionSerial=saved.actionSerial==null?0:cast saved.actionSerial;
        return result;
    }
    public function minimumReachAboveBoardMm(number:Int):Float {
        var t=tool(number),length=t.measuredGaugeTipMm;
        if(length==null)throw "Tool must be measured before a cutting reach check";
        return machine.routerSpec.longestTool-(cast length:Float);
    }
}
