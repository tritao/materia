import cadkit.modeling.AssemblyState;
import materia.assembly.AssemblyFrames as F;
import SheetRouterAtc.RouterTool;
import cnckit.CncController;
import sys.FileSystem;
import sys.io.File;

/** Native cover/setter sweeps; rack transfer checks are added with the ATC policy. */
class SheetRouterAtcChecks {
    static var assertions:Int=0;
    static function check(ok:Bool,message:String):Void {assertions++;if(!ok)throw 'SR7: $message';}
    static function tools():Array<RouterTool> {
        var payload:Dynamic=haxe.Json.parse(File.getContent("machinekit/examples/sheet-router/atc/tool-table.json"));
        var result=SheetRouterAtc.decodeTools(payload.tools);result[0].measuredGaugeTipMm=120;return result;
    }
    static function policy():Void {
        var machine=new SheetRouter();
        var cases=[
            {inject:"missing_tool",expected:"missing_tool"},
            {inject:"occupied_return_pocket",expected:"occupied_return_pocket"},
            {inject:"clamp_timeout",expected:"clamped_timeout"},
            {inject:"vfd_at_speed",expected:"spindle_stop_timeout"},
            {inject:"independent_running",expected:"spindle_stop_timeout"},
            {inject:"air_low",expected:"air_timeout"},
            {inject:"release_pressure_loss",expected:"release_interlock_lost"},
            {inject:"cover_timeout",expected:"cover_open_timeout"},
            {inject:"stale_observation",expected:"stale_or_invalid_observation"},
            {inject:"wrong_motion_ack",expected:"retract_timeout"},
            {inject:"setter_stuck",expected:"setter_clear_timeout"},
            {inject:"setter_no_trigger",expected:"setter_no_trigger"},
            {inject:"stale_setter_edge",expected:"setter_no_trigger"},
            {inject:"axis_lease_missing",expected:"axis_lease_missing"},
            {inject:"axis_lease_lost",expected:"axis_lease_lost"}
        ];
        var results:Array<Dynamic> = [];
        for(c in cases) {
            var saved="",atc=new SheetRouterAtc(machine,tools(),1,json->saved=json);
            var sim=new SheetRouterAtcSimulation(atc,"",false,c.inject),result=sim.run(3);
            check(atc.fault==c.expected,'${c.inject}: expected ${c.expected}, got '+atc.fault+' trace='+atc.trace);
            check(!atc.release&&!atc.spindleRun&&atc.motionAbort&&atc.recoveryRequired&&!result.ack&&!atc.actuatorCommandsEnabled(),'${c.inject} leaves safe outputs, held motion and no acknowledgement');
            if(c.inject=="clamp_timeout")check(atc.loaded==0&&atc.pocketOwner[0]==1&&atc.pocketOwner[2]==3,"clamp timeout retains both tools in their pockets");
            results.push({injection:c.inject,result:result});
        }
        var writes=0,saved="",failed=new SheetRouterAtc(machine,tools(),1,json->{writes++;if(writes==4)throw "injected disk failure";saved=json;});
        var disk=new SheetRouterAtcSimulation(failed,"",false).run(3);
        check(failed.fault=="persistence_failed"&&failed.loaded==3&&!disk.ack,"completed physical exchange is not acknowledged when the final durable commit fails");
        var recovered=SheetRouterAtc.fromJson(machine,saved,json->{}),denied=false;
        check(recovered.recoveryRequired,"interrupted state is not silently resumed");
        check(recovered.motionAbort&&!recovered.actuatorCommandsEnabled()&&!recovered.release&&!recovered.spindleRun,"loading an interrupted checkpoint holds motion and cannot apply default actuator targets");
        var outside=new SheetRouterAtc(machine,tools(),1,json->{}),outsidePlant=new SheetRouterAtcSimulation(outside,"",false);
        check(!outside.actuatorCommandsEnabled(),"construction does not arm actuator targets");
        outside.request(3,0,outsidePlant.feedback(),[100,machine.routerSpec.workY+100,0]);
        check(outside.fault=="change_start_outside_work_area"&&outside.motionAbort&&!outside.actuatorCommandsEnabled(),"an exchange cannot start from an unvalidated service-bay pose");
        try {new SheetRouterAtcSimulation(recovered,"",false).run(1);}catch(error:Dynamic) {denied=true;}
        check(denied,"in-progress disk state needs explicit physical recovery");
        var controller=new CncController(),noop=new SheetRouterAtc(machine,tools(),1,json->{
            var data:Dynamic=haxe.Json.parse(json);for(t in SheetRouterAtc.decodeTools(data.tools))if(t.measuredGaugeTipMm!=null)controller.setToolLength(t.number,(cast t.measuredGaugeTipMm:Float)/1000);
        });
        var noChangePlant=new SheetRouterAtcSimulation(noop,"",false),noChange=noChangePlant.run(1);
        check(noChange.ack&&!noop.release&&Math.abs(controller.toolLength(1)-0.12)<1e-9,"same-tool acknowledgement still applies the measured H value");
        check(!noop.acknowledge("cnc.tool_change.1",noop.requestSerial-1,noChangePlant.now,noChangePlant.feedback()),"a previous request cannot acknowledge a new barrier");
        var lostClamp=noChangePlant.feedback();lostClamp.clamped=false;
        check(!noop.acknowledge("cnc.tool_change.1",noop.requestSerial,noChangePlant.now,lostClamp)&&noop.fault=="completion_feedback_lost","completion feedback is revalidated at acknowledgement");
        var unmeasured=tools();unmeasured[0].measuredGaugeTipMm=null;
        var unknownLength=new SheetRouterAtc(machine,unmeasured,1,json->{});
        new SheetRouterAtcSimulation(unknownLength,"",false).run(1);
        check(unknownLength.fault=="tool_length_unmeasured"&&!unknownLength.complete,"same-tool unmeasured request fails without hanging");
        var cold=new SheetRouterAtc(machine,tools(),0,json->{}),coldResult=new SheetRouterAtcSimulation(cold,"",false).run(3);
        check(coldResult.ack&&cold.loaded==3&&cold.pocketOwner[0]==1&&cold.pocketOwner[2]==0,"empty spindle pickup confirms released drawbar before insertion");
        File.saveContent("build/sheet-router-work/sr7/faults.json",haxe.Json.stringify({status:"PASS",cases:results,persistenceFailure:disk,assertions:assertions},null,"  ")+"\n");
        Sys.println('SR7 policy: $assertions assertions pass');Sys.stdout().flush();
    }
    static function sequence():Void {
        var sampler=Sys.getEnv("SHEET_ROUTER_PLANNER_SAMPLE");if(sampler==null)sampler="build/sheet-router-work/sr7/planner-sample";
        var reports:Array<Dynamic> = [];
        var selectedBed=Sys.args().indexOf("--bed2600")>=0?2600:Sys.args().indexOf("--bed3150")>=0?3150:0;
        var evidencePath=selectedBed==0?"build/sheet-router-work/sr7/sequence.json":'build/sheet-router-work/sr7/sequence-y$selectedBed.json';
        var expectedChanges=selectedBed==0?4:2;
        for(bed in [2600.0,3150.0]) {
            if(selectedBed!=0&&selectedBed!=bed)continue;
            var machine=new SheetRouter(new SheetRouterSpec(bed)),inventory=tools();
            var controller=new CncController(),statePath='build/sheet-router-work/sr7/state-y${Std.int(bed)}.json';
            var atc=new SheetRouterAtc(machine,inventory,1,SheetRouterAtcState.checkpoint(statePath,controller));
            var physical=new SheetRouterAtcAssembly(machine,inventory),state=new AssemblyState(physical.definition);
            var clearance=new GantryClearanceChecks(machine,physical.definition,physical.interfaces);
            var time=0.0;
            try {
                var poses:Array<Dynamic> = [];
                function pose(fixture:SheetRouterAtc,q:Array<Float>,label:String,cover:Float=0):Void {
                    state.setJoint("x",q[0]);state.setJoint("y",q[1]);state.setJoint("z",q[2]);
                    state.setJoint("shoeLift",0);state.setJoint("rackCoverLift",cover);physical.place(state,fixture);
                    try {clearance.check(state);physical.verifySpecial(state,fixture);}catch(error:Dynamic) {throw 'ATC pocket sweep Y=$bed $label q='+q+': '+error;}
                    assertions++;poses.push({label:label,q:q,cover:cover,loaded:fixture.loaded});
                }
                {
                    for(x in [0.0,800.0,1600.0])for(y in [0.0,bed/2,bed])pose(atc,[x,y,200],"full-Z cutting with closed cover",600);
                    Sys.println('SR7 Y=$bed: '+poses.length+' closed-cover cutting poses pass');Sys.stdout().flush();
                    if(Sys.args().indexOf("--cutting-only")>=0) {clearance.close();physical.close();continue;}
                }
                var fixedY=0.0;for(binding in machine.cableChainBindings)if(binding.axis=="y")fixedY=binding.pose.y;
                for(offset in [-120.0,-60.0,0.0,60.0,120.0])pose(atc,[0,fixedY+offset,0],"Y-chain bracket transit");
                if(Sys.args().indexOf("--transit-only")>=0) {
                    Sys.println('SR7 Y=$bed: '+poses.length+' cutting/bracket poses pass');Sys.stdout().flush();
                    clearance.close();physical.close();continue;
                }
                for(t in inventory) {
                    var returning=new SheetRouterAtc(machine,inventory,t.number,json->{}),empty=new SheetRouterAtc(machine,inventory,0,json->{});
                    var seat=atc.pocketGauge(t.number),entry=atc.approachGauge(t.number,seat[2]-55);
                    pose(returning,entry,'return high approach T${t.number}');
                    for(lift in [55.0,27.5,0.0])pose(returning,[seat[0],seat[1],seat[2]-lift],'vertical return/lift T${t.number} $lift');
                    pose(empty,atc.approachGauge(t.number,seat[2]-55),'pickup approach T${t.number}');
                    for(lift in [55.0,27.5,0.0])pose(empty,[seat[0],seat[1],seat[2]-lift],'pickup taper T${t.number} $lift');
                }
                var empty=new SheetRouterAtc(machine,inventory,0,json->{});
                for(cover in [0.0,150.0,300.0,450.0,600.0])pose(empty,[0,0,0],"filled-rack cover",cover);
                File.saveContent('build/sheet-router-work/sr7/pocket-poses-y${Std.int(bed)}.json',haxe.Json.stringify({status:"PASS",poses:poses},null,"  ")+"\n");
                Sys.println('SR7 Y=$bed: '+poses.length+' filled-rack/pocket poses pass');Sys.stdout().flush();
                // First exchange starts at full Z after a representative cut.
                state.setJoint("x",800);state.setJoint("y",bed/2);state.setJoint("z",200);
                for(number in [3,1]) {
                    var sim=new SheetRouterAtcSimulation(atc,sampler,true,"",time,physical,state,clearance),result=sim.run(number);time=sim.now+0.1;
                    check(atc.complete&&atc.loaded==number&&result.ack,'Y$bed T$number confirmed and acknowledged');
                    var expected=number==3?120.003:119.996;
                    check(Math.abs(controller.toolLength(number)*1000-expected)<1e-7,'Y$bed measured H$number applied in SI before acknowledgement');
                    check(Math.abs(controller.toolForH(number).diameter*1000-atc.tool(number).diameterMm)<1e-8,"tool diameter applied without inventing flute reach");
                    var compiled=cnckit.CncCompiler.compileDetailed('G21 G90\nT$number M6\nG43 H$number\nG0 Z10\nM2\n',controller);
                    check(compiled.diagnostics.length==0,"next CNC segment compiles without diagnostics");
                    var applied=false;
                    for(op in compiled.program.ops)switch op {
                        case toolpathkit.path.ToolpathOp.ToolLengthOffset(h,length,_):if(h==number&&Math.abs(length-expected/1000)<1e-9)applied=true;
                        case _:
                    }
                    check(applied,"next CNC segment compiles with the newly measured G43 H value");
                    check(result.relativeTimeError<=0.1&&result.maximumTransferJumpMm<1e-5,"timing and continuous body handoff");
                    var restored=SheetRouterAtcState.load(machine,statePath,new CncController());
                    check(restored.loaded==number&&!restored.recoveryRequired,"completed inventory survives atomic persistence");
                    reports.push({workY:bed,result:result,trace:atc.trace.copy(),measuredGaugeTipMm:atc.tool(number).measuredGaugeTipMm});
                    File.saveContent(evidencePath,haxe.Json.stringify({status:reports.length==expectedChanges?"PASS":"IN_PROGRESS",reports:reports,assertions:assertions},null,"  ")+"\n");
                    Sys.println('SR7 Y=$bed T$number: '+haxe.Json.stringify(result));Sys.stdout().flush();
                }
                clearance.close();physical.close();
            }catch(error:Dynamic) {clearance.close();physical.close();throw error;}
        }
        if(Sys.args().indexOf("--transit-only")<0&&Sys.args().indexOf("--cutting-only")<0)File.saveContent(evidencePath,haxe.Json.stringify({status:reports.length==expectedChanges?"PASS":"IN_PROGRESS",reports:reports,assertions:assertions},null,"  ")+"\n");
    }
    static function coverAndSetter():Void {
        var assertions=0;
        for(bed in [2600.0,3150.0]) {
            var machine=new SheetRouter(new SheetRouterSpec(bed)),definition=machine.definition();
            var state=new AssemblyState(definition);
            var clearance=new GantryClearanceChecks(machine,definition,[
                {a:"shoeLiftCylinder",b:"shoeLiftRod"},{a:"rackCoverLiftCylinder",b:"rackCoverLiftRod"},
                {a:"longestTool",b:"setter"}]);
            try {
                state.setJoint("x",0);state.setJoint("y",0);state.setJoint("z",0);state.setJoint("shoeLift",0);
                for(i in 0...13) {
                    state.setJoint("rackCoverLift",i*50);state.forwardKinematics();clearance.check(state);assertions++;
                }
                state.setJoint("rackCoverLift",0);
                var trigger=state.worldConnector("setter","trigger");
                var target=machine.gaugeCoordinates(F.translation(trigger.x,trigger.y,trigger.z+machine.routerSpec.longestTool));
                if(target[0]<0||target[0]>1600||target[1]<0||target[1]>bed+300||target[2]<0||target[2]>200)throw 'Unreachable setter on $bed';
                state.setJoint("x",target[0]);state.setJoint("y",target[1]);
                for(i in 0...21) {
                    state.setJoint("z",target[2]*i/20);state.forwardKinematics();clearance.check(state);assertions++;
                }
                Sys.println('SR7 cover/setter Y=$bed: 34 native poses pass');Sys.stdout().flush();
                clearance.close();
            } catch(error:Dynamic) {clearance.close();throw error;}
        }
        Sys.println('SR7 cover/setter: $assertions assertions pass');
    }
    public static function main():Void {
        if(!FileSystem.exists("build/sheet-router-work/sr7"))FileSystem.createDirectory("build/sheet-router-work/sr7");
        if(Sys.args().indexOf("--cover-only")>=0) {coverAndSetter();return;}
        if(Sys.args().indexOf("--sequence-only")<0)policy();
        if(Sys.args().indexOf("--policy-only")>=0)return;
        sequence();
    }
}
