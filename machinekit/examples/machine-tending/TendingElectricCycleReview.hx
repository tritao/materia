import TendingCyclePlan.TendingOperation;
import machinekit.robotics.DualArmGripperTool;

/** Ordered production review. It never emits an executable robot program. */
class TendingElectricCycleReview {
  static function side(tool:String):String {
    if(tool==DualArmGripperTool.TOOL_CHANNEL_A)return "a";
    if(tool==DualArmGripperTool.TOOL_CHANNEL_B)return "b";
    throw "Unknown production tool";
  }
  static function distance(a:Dynamic,b:Dynamic):Float {
    var x:Float=a.x-b.x,y:Float=a.y-b.y,z:Float=a.z-b.z;return Math.sqrt(x*x+y*y+z*z);
  }
  static function guardLimits(row:Dynamic,raw:Dynamic,clearanceFirst:Bool=false):Dynamic {
    var path:Array<Dynamic> = cast raw.path,approach=0.0,withdrawal=0.0;
    if(row.requiresGuardedContact==true)for(i in Std.int(row.failedApproachEdge)+1...path.length)approach+=distance(path[i-1],path[i]);
    var edges:Array<Dynamic> = cast row.withdrawal.guardedEdges;
    for(edge in edges){var i=Std.int(edge);withdrawal+=distance(path[i-1],path[i]);}
    var expected:Array<String> = [];
    if(StringTools.startsWith(Std.string(row.station),"vise-"))for(member in ["fixedJaw","movingJaw","parallel0","parallel1","endStop"])expected.push("mill/mill/vise/"+member);
    else {
      var pieces=Std.string(row.station).split("-");
      for(x in [-1,1])for(y in [-1,1])expected.push(pieces[0]+"/seat-r"+pieces[1]+"-c"+pieces[2]+"-x"+x+"-y"+y);
    }
    var supports=StringTools.startsWith(Std.string(row.station),"vise-")
      ? ["mill/mill/vise/parallel0","mill/mill/vise/parallel1"] : expected.copy();
    var supportSearch:Dynamic=null;
    if(clearanceFirst && StringTools.startsWith(Std.string(row.station),"outfeedTray-") && row.activeStockOwner=="tool") {
      if(path.length<2)throw "Support placement requires a final approach segment";
      var finalTravel=distance(path[path.length-2],path[path.length-1]);
      if(!Math.isFinite(finalTravel) || finalTravel<=0)throw "Invalid final support approach distance";
      var searchTravel=Math.min(3.0,finalTravel);
      supportSearch={expectedContacts:supports.copy(),entryFraction:1-searchTravel/finalTravel,
        minimumPresearchClearanceMm:1.0,sampleTravelMm:searchTravel,commandTravelMm:searchTravel+0.12,
        forceLimitN:5.0,contactThresholdN:0.05,commandSpeedMmPerSecond:0.05,speedLimitMmPerSecond:0.5,
        totalPointSpeedLimitMmPerSecond:1.0,timeoutSeconds:150.0,numericalPenetrationLimitMm:0.001,stableSamples:10,
        returnCorridorClear:row.withdrawal.freeEdgesClear==true,returnPolicy:"reverse executed search samples",
        simulationOnly:true,hardwareQualified:false};
    }
    return {expectedContacts:expected,beforePickup:clearanceFirst && row.station=="vise-seat" ?
      {maximumPointMotionMm:0.12,positionToleranceMm:1.0,orientationToleranceRad:0.001,
        extraction:{clearanceXmm:0.5,maximumStepMm:0.05,slowSeparationDistanceMm:0.75,commandSpeedMmPerSecond:0.05,
          forceLimitN:70.0,speedLimitMmPerSecond:0.5,totalPointSpeedLimitMmPerSecond:1.0,timeoutSeconds:90.0,
          airRejoinClearanceMm:5.0,numericalPenetrationLimitMm:0.001,simulationOnly:true,hardwareQualified:false},
        simulationOnly:true,hardwareQualified:false} : null,
      beforeRelease:supportSearch,afterRelease:{supportContacts:supports,positionToleranceMm:1.0,
      datum:row.station=="vise-load" ? "supported-load" : "seat",requireSupportLoad:true,
      orientationConstraint:row.station=="vise-load" ? "support-normal" : "full-pose",
      orientationToleranceRad:0.001,stableSamples:10,timeoutSeconds:2.0,minimumSupportForceN:0.01,simulationOnly:true},
      afterClamp:{supportContacts:supports,positionToleranceMm:1.0,datum:"seat",requireSupportLoad:false,orientationConstraint:"full-pose",
      orientationToleranceRad:0.001,stableSamples:10,timeoutSeconds:2.0,minimumSupportForceN:0.01,simulationOnly:true},approach:{maxTravelMm:approach+0.12,requireContactAtArrival:row.activeStockOwner!="tool",
      supportConfirmationAfterRelease:row.activeStockOwner=="tool"},withdrawal:{maxTravelMm:withdrawal+0.12},
      pathToleranceReserveMm:0.12,numericalPenetrationLimitMm:0.001,timeoutSeconds:90,
      forceLimitN:null,speedLimitMmPerSecond:null,hardwareQualified:false,
      failureResponse:"latched robot stop; hold payload and gripper state; inhibit next operation; no automatic retry or retreat"};
  }
  public static function main():Void {
    TendingCyclePlan.TendingCyclePlanChecks.run();
    var root=DhGripperPreview.directory()+"/../machine-tending/build/";
    var clearanceFirst=Sys.args().indexOf("--clearance-first")>=0;
    var files=[root+"electric-expert-vise-setback-50"+(clearanceFirst ? "-clearance-first" : "")+"/planner-report.json",
      root+"electric-expert-access-setback-50-trays/planner-report.json"];
    var cases:Map<String,Dynamic> = [],sources:Array<Dynamic> = [],geometry:Null<String> = null,park:Array<Float> = [];
    for(file in files) {
      var text=sys.io.File.getContent(file),report:Dynamic=haxe.Json.parse(text);
      var key=haxe.Json.stringify(report.provenance);
      if(geometry!=null && geometry!=key)throw "Cycle review requires one current collision geometry";
      geometry=key;
      if(report.traySetbackMm!=50)throw "Cycle review requires the common 50 mm layout";
      var proposal=sys.io.File.getContent(Std.string(report.sourceReport));
      if(haxe.crypto.Sha256.encode(proposal)!=report.sourceReportSha256)throw "Proposal report changed after validation";
      sources.push({path:file,sha256:haxe.crypto.Sha256.encode(text)});
      var raw:Dynamic=haxe.Json.parse(proposal),rawRows:Array<Dynamic> = cast raw.results;
      var processFile=haxe.io.Path.directory(file)+"/process-report.json",processText=sys.io.File.getContent(processFile),process:Dynamic=haxe.Json.parse(processText);
      if(haxe.Json.stringify(process.provenance)!=geometry || process.sourceReportSha256!=report.sourceReportSha256)
        throw "Process checks and route checks do not share geometry/proposals";
      sources.push({path:processFile,sha256:haxe.crypto.Sha256.encode(processText)});
      var processRows:Array<Dynamic> = cast process.results,processCases:Map<String,Dynamic> = [];
      for(value in processRows)processCases.set(Std.string(value.station)+"/"+Std.string(value.side),value.jawStroke);
      if(process.doorSweep.clear!=true)throw "Door stroke collides with parked robot/tool";
      var rows:Array<Dynamic> = cast report.results;
      for(row in rows) {
        var reference=Std.string(row.station)+"/"+Std.string(row.side);
        row.jawStroke=processCases.get(reference);
        if(row.jawStroke==null || row.jawStroke.clear!=true)throw "Full jaw stroke is blocked: "+reference;
        var input=[for(value in rawRows)if(value.station==row.station && value.side==row.side && value.ikStatus=="sampled route solved and clear")value][0];
        if(input==null)throw "Missing guarded path source";
        // Pickup starts with real stock resting on its named support faces. A clear
        // geometric edge still needs bounded contact separation at its first step.
        if(row.withdrawal.stockOwner=="tool") {
          var path:Array<Dynamic> = cast input.path;
          var guards:Array<Int> = cast row.withdrawal.guardedEdges;
          var terminal=path.length-1;
          if(terminal<1)throw "Pickup needs a departure edge";
          if(guards.indexOf(terminal)<0)guards.unshift(terminal);
          row.withdrawal.supportSeparationRequired=true;
          row.withdrawal.complete=false;
        }
        if(clearanceFirst && row.station=="vise-load" && row.locating!=null)row.locating.restGapLimitMm=0.01;
        row.guardLimits=guardLimits(row,input,clearanceFirst);
        row.runtimeProfile=TendingElectricRuntimeProfile.bind(row,input);
        if(clearanceFirst && row.station=="vise-load" && row.runtimeProfile.locating==null)
          throw "Clearance-first review requires both locating corridors";
        cases.set(Std.string(row.station)+"/"+Std.string(row.side),row);
        if(row.transfer.found==true) {
          var path:Array<Array<Float>> = cast row.transfer.waypoints;
          if(path.length<2)throw "Transfer needs entry and park";
          if(park.length==0)park=path[0].copy();
          var doorPark:Array<Float> = cast process.doorSweep.park;
          if(path[0].length!=6 || doorPark.length!=6)throw "Park needs six finite coordinates";
          for(j in 0...6) {
            if(!Math.isFinite(path[0][j]) || Math.abs(path[0][j]-park[j])>1e-9)throw "Production transfers do not share the same park";
            if(!Math.isFinite(doorPark[j]) || Math.abs(doorPark[j]-park[j])>1e-9)throw "Door stroke was checked at a different park";
          }
        }
      }
    }
    var stations=["vise-seat","vise-load"];
    for(tray in ["infeedTray","outfeedTray"])for(r in 0...2)for(c in 0...3)stations.push(tray+"-"+r+"-"+c);
    for(station in stations)for(side in ["a","b"])if(!cases.exists(station+"/"+side))throw "Full 28-case coverage is required";
    var production=TendingCyclePlan.electricBearingBlocks(),steps:Array<Dynamic> = [],blocked:Array<String> = [],cuts=0;
    for(operation in production.operations()) {
      var station:Null<String> = null,tool:Null<String> = null,withdrawal=false;
      switch(operation) {
        case Collect(slot,selected,_,_): station="infeedTray-"+Std.int(slot/production.columns)+"-"+(slot%production.columns);tool=selected;
        case Deliver(slot,selected,_,_): station="outfeedTray-"+Std.int(slot/production.columns)+"-"+(slot%production.columns);tool=selected;
        case Load(selected,_): station="vise-load";tool=selected;
        case Unload(_,selected): station="vise-seat";tool=selected;
        case LeaveVise(selected): station="vise-load";tool=selected;withdrawal=true;
        case Retreat(selected): station="vise-seat";tool=selected;withdrawal=true;
        case StartCycle: cuts++;
        case _: {}
      }
      var reference:Null<String> = null,row:Dynamic=null;
      if(station!=null) {
        reference=station+"/"+side(tool);row=cases.get(reference);
        if(row==null)blocked.push("Missing case "+reference);
        else if(withdrawal) {
          if(row.withdrawal==null || row.withdrawal.freeEdgesClear!=true)blocked.push("Withdrawal blocked: "+reference);
        } else {
          if(row.transfer.found!=true || row.approachStateTransferClear!=true ||
            (row.constrainedApproachClear!=true && row.requiresGuardedContact!=true))blocked.push("Access blocked: "+reference);
          switch(operation) {
            case Collect(_,_,_,_) | Deliver(_,_,_,_):
              if(row.withdrawal==null || row.withdrawal.freeEdgesClear!=true)blocked.push("Tray withdrawal blocked: "+reference);
            case _: {}
          }
        }
      }
      steps.push({index:steps.length,operation:Std.string(operation),caseReference:reference,
        phase:withdrawal ? "withdrawal" : station==null ? "machine handshake" : "access",
        guardedContact:row!=null && row.requiresGuardedContact==true,
        guardedWithdrawal:row!=null && row.withdrawal!=null ? row.withdrawal.guardedEdges : null});
    }
    if(cuts!=4)throw "Production review lost a machining cycle";
    var fingerEnvelopeChecks=TendingJawStrokeAudit.checkConstruction();
    var simulation:Dynamic=blocked.length==0 ? new TendingElectricCycleSimulation(cases,true).run() : {passed:false};
    var guardedFaultChecks=TendingGuardedAction.checks();
    var faultSimulation:Dynamic=blocked.length==0 ? new TendingElectricCycleSimulation(cases,true).run(true) : {passed:false};
    var graspFaultSimulation:Dynamic=blocked.length==0 ? new TendingElectricCycleSimulation(cases,true).run(false,true) : {passed:false};
    var policies:Array<Dynamic> = [for(key in cases.keys()){caseReference:key,limits:cases.get(key).guardLimits}];
    var runtimeProfiles:Array<Dynamic> = [for(key in cases.keys())cases.get(key).runtimeProfile];
    var runtimeChecks=TendingElectricRuntimeChecks.run(runtimeProfiles);
    var guardedRejections=0;
    for(profile in runtimeProfiles) {
      var edges:Array<Dynamic> = cast profile.incoming;
      var guarded=false;for(edge in edges)if(edge.kind=="guardedJointEdge")guarded=true;
      if(guarded || profile.seating!=null) {var rejected=false;try TendingElectricRuntimeProfile.ordinaryWaypoints(profile) catch(_:Dynamic)rejected=true;
        if(!rejected)throw "Guarded path escaped into ordinary playback";guardedRejections++;}
    }
    var report={programReady:false,placementStrategy:clearanceFirst ? "clearance-first and guarded locating" : "legacy zero-clearance loading",runtimeProfiles:runtimeProfiles,runtimeChecks:runtimeChecks,guardedPlaybackRejections:guardedRejections,modelSimulationReady:blocked.length==0 && simulation.passed==true,
      simulation:simulation,fingerEnvelopeChecks:fingerEnvelopeChecks,guardedFaultChecks:guardedFaultChecks,payloadHoldFaultSimulation:faultSimulation,graspConfirmationFaultSimulation:graspFaultSimulation,guardedPolicies:policies,
      accessAndFreeWithdrawalReady:blocked.length==0,machiningCycles:cuts,sources:sources,provenance:haxe.Json.parse(geometry),
      park:park,connectionPolicy:"Reverse the validated transfer after withdrawal to its entry; connect subsequent transfers through the common park, retaining each door-state policy",
      accessBlockers:blocked,steps:steps,requiredProcessChecks:[
        "Hardware-qualified force and speed limits plus contact/gripper feedback bindings are required",
        "Released stock must settle on every named support before withdrawal or clamp",
        "The guarded supervisor is tested in the model; physical contact sensing/compliance must be commissioned",
        "Flexible cable motion, grasp retention, dynamics and supplier robot/connector dimensions remain unqualified"],
      scope:"Discrete four-part electric sequence, full jaw/door sweeps, stock ownership and real CNC PLC interlocks; no physics playback or hardware I/O"};
    sys.io.File.saveContent(root+(clearanceFirst ? "electric-cycle-review-clearance-first.json" : "electric-cycle-review.json"),haxe.Json.stringify(report,null,"  "));
    Sys.println('Reviewed ${steps.length} production operations, $cuts cuts, ${blocked.length} access/withdrawal blockers; model simulation passed=${simulation.passed}; programReady=false');
  }
}
