import TendingCyclePlan.TendingOperation;
import cnckit.controller.RobotInterface;
import cnckit.controller.RobotInterfaceInput;
import cnckit.controller.RobotInterfaceOutputs;
import machinekit.robotics.DualArmGripperTool;

/** Discrete process simulation consuming geometric certificates and the real CNC PLC. No hardware I/O. */
class TendingElectricCycleSimulation {
  final cases:Map<String,Dynamic>;
  final held:Map<String,Int> = [];
  final delivered:Map<Int,Bool> = [];
  final finished:Map<Int,Bool> = [];
  final collected:Map<Int,Bool> = [];
  final plc=new RobotInterface();
  final events:Array<Dynamic> = [];
  var part:Null<Int> =null;
  var doorOpen=false;
  var closing=false;
  var clamped=false;
  var running=false;
  var inside=false;
  var parked=true;
  var cuts=0;
  var guards=0;
  var injected=false;
  public function new(cases:Map<String,Dynamic>,doorCertified:Bool) {if(!doorCertified)throw "Door sweep certificate is required";this.cases=cases;}
  static function side(tool:String):String return tool==DualArmGripperTool.TOOL_CHANNEL_A ? "a" : tool==DualArmGripperTool.TOOL_CHANNEL_B ? "b" : throw "Unknown tool";
  function require(value:Bool,message:String):Void {if(!value)throw message;}
  function tick(request:Int=0,complete:Bool=false):RobotInterfaceOutputs {
    var output=plc.update(new RobotInterfaceInput(request==1,request==2,request==3,request==4,request==5,
      !inside,false,doorOpen,!doorOpen,clamped,!clamped,part!=null,!running,!running,complete,false),0.01);
    require(output.alarm==0,'PLC interlock alarm ${output.alarm}');return output;
  }
  function route(station:String,tool:String):Dynamic {
    var row:Dynamic=cases.get(station+"/"+side(tool));require(row!=null,"Missing route");
    require(row.transfer.found==true && row.approachStateTransferClear==true,"Uncertified transfer");
    require(row.constrainedApproachClear==true || row.requiresGuardedContact==true,"Blocked approach");
    require(row.withdrawal!=null && row.withdrawal.freeEdgesClear==true,"Blocked withdrawal");
    require(row.jawStroke!=null && row.jawStroke.clear==true,"Uncertified jaw stroke");return row;
  }
  function guard(row:Dynamic,separating:Bool,inject:Bool=false):Void {
    var needed=separating ? row.withdrawal.guardedEdges.length>0 : row.requiresGuardedContact==true;
    if(!needed && !inject)return;
    var limits:Dynamic=separating ? row.guardLimits.withdrawal : row.guardLimits.approach;
    var expected:Array<String> = cast row.guardLimits.expectedContacts;
    var action=new TendingGuardedAction(expected,Math.max(0.12,limits.maxTravelMm),90,null,null,
      !Reflect.hasField(row.guardLimits.approach,"requireContactAtArrival") || row.guardLimits.approach.requireContactAtArrival==true);
    action.observe(0.01,0,inject ? ["unexpected robot/fixture"] : separating ? [expected[0]] : [],0,false,separating);
    if(!action.stopped)action.observe(0.01,Math.min(action.maxTravelMm,limits.maxTravelMm),separating ? [] : [expected[0]],0,true,separating);
    if(inject)injected=true;
    require(action.complete && !action.stopped,'Guarded action stopped: ${action.failure}');guards++;
  }
  function withdraw(row:Dynamic):Void {guard(row,true);inside=false;parked=true;tick();}
  public function run(injectLoadFailure:Bool=false,injectGraspFailure:Bool=false):Dynamic {
    var production=TendingCyclePlan.electricBearingBlocks();tick();
    try {
      for(operation in production.operations()) {
        switch(operation) {
          case Collect(slot,tool,_,_):
            require(parked && !closing && !doorOpen && !held.exists(side(tool)) && !collected.exists(slot),"Invalid collect state");
            var row=route('infeedTray-${production.rowOf(slot)}-${production.columnOf(slot)}',tool);
            guard(row,false);held.set(side(tool),slot);collected.set(slot,true);withdraw(row);
          case OpenDoor:
            require(parked && !running && !closing,"Door open requires parked robot and stopped CNC");
            tick(1);doorOpen=true;tick();
          case Unclamp:
            require(doorOpen && !running,"Unclamp interlock");tick(4);clamped=false;tick();
          case Load(tool,fromTool):
            require(parked && doorOpen && !closing && !clamped && part==null && held.exists(side(tool)) && fromTool==null,"Invalid electric load state");
            var row=route("vise-load",tool);inside=true;parked=false;tick();
            guard(row,false,injectLoadFailure && !injected);
            part=held.get(side(tool));held.remove(side(tool)); // Full release stroke certified before changing ownership.
          case LeaveVise(tool):
            require(inside && part!=null && !clamped && !held.exists(side(tool)),"Release must precede withdrawal");withdraw(route("vise-load",tool));
          case Clamp:
            require(parked && doorOpen && part!=null && !running && !closing,"Clamp must follow withdrawal and precede door closure");
            tick(3);clamped=true;tick();
          case RequestDoorClose:
            require(parked && !inside && doorOpen && !closing && !running && (part==null || clamped),"Unsafe door request");
            tick(2);closing=true;
          case DoorClosed:
            require(closing && parked,"Door confirmation without parked robot/request");doorOpen=false;closing=false;tick();
          case StartCycle:
            require(parked && !doorOpen && !closing && clamped && part!=null && !running,"Invalid CNC start");
            require(tick(5).startCycle,"PLC refused cycle start");running=true;cuts++;tick();
          case CycleReady:
            require(running && clamped && part!=null,"Cycle completion without stock");finished.set(cast part,true);running=false;
            require(tick(0,true).cycleComplete,"PLC did not acknowledge cut completion");tick();
          case Unload(slot,tool):
            require(parked && doorOpen && !running && clamped && part==slot && finished.exists(slot) && !held.exists(side(tool)),"Invalid unload state");
            var row=route("vise-seat",tool);inside=true;parked=false;tick();guard(row,false);
            // Grasp feedback completes while the vise still holds the part; no motion until unclamped.
            if(injectGraspFailure && !injected){injected=true;throw "grasp confirmation timeout";}
            events.push({phase:"grasp confirmed before unclamp",slot:slot,tool:side(tool)});
            tick(4);clamped=false;tick();held.set(side(tool),slot);part=null;tick();
          case Retreat(tool):
            require(inside && held.exists(side(tool)) && !clamped,"Pickup withdrawal before confirmed grasp/unclamp");withdraw(route("vise-seat",tool));
          case Deliver(slot,tool,_,returnToPark):
            require(parked && !closing && !doorOpen && held.get(side(tool))==slot && held.exists(side(tool)) && finished.exists(slot) && !delivered.exists(slot) && returnToPark,"Invalid delivery state");
            var row=route('outfeedTray-${production.rowOf(slot)}-${production.columnOf(slot)}',tool);
            guard(row,false);held.remove(side(tool));delivered.set(slot,true);withdraw(row);
        }
        events.push({operation:Std.string(operation),parked:parked,doorOpen:doorOpen,clamped:clamped,
          machineStock:part,heldA:held.get("a"),heldB:held.get("b"),running:running});
      }
    } catch(error:Dynamic) {
      if(injectGraspFailure && injected) {
        require(clamped && part==0 && !held.exists("a") && held.exists("b") && held.get("b")==1 && cuts==1,
          "Grasp failure opened the vise or lost a part");
        return {passed:true,injectedGraspFailureStopped:true,viseRemainsClamped:true,companionPayloadRetained:true,
          automaticRetry:false,events:events};
      }
      if(!injectLoadFailure || !injected)throw error;
      require(held.exists("a") && held.get("a")==0 && part==null && cuts==0 && !clamped,"Contact fault lost or released payload");
      return {passed:true,injectedFaultStopped:true,payloadRetained:true,automaticRetry:false,events:events};
    }
    require(!injectLoadFailure && !injectGraspFailure && cuts==4 && part==null && !running && parked && !doorOpen && !held.exists("a") && !held.exists("b"),"Cycle did not finish in a safe final model state");
    for(slot in [0,1,2,3])require(delivered.exists(slot) && finished.exists(slot),"Production lost a part");
    return {passed:true,machiningCycles:cuts,deliveredParts:4,operations:production.operations().length,guardedActions:guards,
      finalRobotParked:parked,plcAlarms:0,events:events,
      scope:"Discrete stock ownership, jaw-state completion, certified access/withdrawal, guarded action supervisor and real CNC PLC; no rigid-body dynamics or hardware I/O"};
  }
}
