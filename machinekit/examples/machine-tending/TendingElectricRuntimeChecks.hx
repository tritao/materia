import TendingElectricEdgeExecutor.TendingElectricEdgeDriver;
import TendingElectricEdgeExecutor.TendingElectricEdgeFeedback;

/** Deterministic driver contract checks, deliberately separate from physics playback. */
class TendingElectricRuntimeChecks {
  public static function run(profiles:Array<Dynamic>):Dynamic {
    var passed=0,edgesChecked=0;
    for(profile in profiles)for(direction in ["incoming","outgoing"]) {
      var edges:Array<Dynamic> = cast Reflect.field(profile,direction);
      var driver=new ElectricContractDriver(edges);
      var runner=new TendingElectricEdgeExecutor(edges,driver);
      for(_ in 0...edges.length+1)runner.tick(0.01);
      if(!runner.complete || runner.stopped || driver.stops!=0)throw "Electric feedback contract failed";
      edgesChecked+=edges.length;passed++;
    }
    var start=[0.0,0,0,0,0,0],end=[0.1,0,0,0,0,0];
    var policy={expectedContacts:["fixture/seat"],approach:{maxTravelMm:1.0},withdrawal:{maxTravelMm:1.0},timeoutSeconds:1.0,forceLimitN:null,speedLimitMmPerSecond:null};
    for(mode in ["unexpected contact","missing contact","false arrival","driver failure","retention failure","excess penetration","timeout","bad entry"]) {
      var edges:Array<Dynamic> = [{kind:"guardedJointEdge",from:start,to:end,contactPolicy:policy,separating:false}];
      var driver=new ElectricContractDriver(edges,mode);
      var runner=new TendingElectricEdgeExecutor(edges,driver);
      runner.tick(mode=="timeout" ? 2 : 0.01);
      if(!runner.stopped || runner.complete || driver.stops!=1)throw "Electric failure did not stop/hold: "+mode;
      runner.tick(0.01);
      if(driver.stops!=1 || driver.starts>1)throw "Electric failure retried: "+mode;
    }
    var placementPolicy={expectedContacts:["fixture/seat"],approach:{maxTravelMm:1.0,requireContactAtArrival:false},
      withdrawal:{maxTravelMm:1.0},timeoutSeconds:1.0,forceLimitN:null,speedLimitMmPerSecond:null};
    var placing:Array<Dynamic> = [{kind:"guardedJointEdge",from:start,to:end,contactPolicy:placementPolicy,separating:false}];
    var unloadedSupport=new ElectricContractDriver(placing,"missing contact");
    var positioned=new TendingElectricEdgeExecutor(placing,unloadedSupport);positioned.tick(0.01);
    if(!positioned.complete || positioned.stopped || unloadedSupport.stops!=0)
      throw "Placement position incorrectly requires support load before release";
    var grouped:Array<Dynamic> = [{kind:"guardedJointEdge",from:start,to:end,contactPolicy:policy,separating:true},
      {kind:"guardedJointEdge",from:end,to:start,contactPolicy:policy,separating:true}];
    var cumulativeDriver=new ElectricContractDriver(grouped,"cumulative travel");
    var cumulative=new TendingElectricEdgeExecutor(grouped,cumulativeDriver);
    cumulative.tick(0.01);cumulative.tick(0.01);
    if(!cumulative.stopped || cumulative.complete || cumulativeDriver.stops!=1)throw "Guarded travel budget reset between edges";
    return {passed:true,roundTrips:passed/2,edgesChecked:edgesChecked,preReleaseSupportLoadOptional:true,latchedFaultChecks:9,
      scope:"Deterministic feedback-driver contract; no physics, vendor I/O or qualified hardware limits"};
  }
}

private class ElectricContractDriver implements TendingElectricEdgeDriver {
  final edges:Array<Dynamic>;
  final fault:String;
  var index=0;
  var q:Array<Float>;
  public var stops=0;
  public var starts=0;
  public function new(edges:Array<Dynamic>,fault:String="") {
    this.edges=edges;this.fault=fault;var initial:Array<Float> = cast edges[0].from;q=initial.copy();if(fault=="bad entry")q[0]+=1;
  }
  public function joints():Array<Float> return q.copy();
  public function start(target:Array<Float>):Void {starts++;if(fault=="driver failure")throw "Injected driver fault";q=target.copy();}
  public function stopAndHold():Void {stops++;}
  public function feedback():TendingElectricEdgeFeedback {
    if(fault=="retention failure")throw "Injected loss of bilateral grasp";
    var edge:Dynamic=edges[index++];
    var contacts:Array<String> = [];
    if(edge.kind=="guardedJointEdge" && edge.separating!=true) {var expected:Array<String> = cast edge.contactPolicy.expectedContacts;contacts=[expected[0]];}
    if(fault=="unexpected contact")contacts=["robot/unexpected"];
    if(fault=="missing contact")contacts=[];
    var measured=q.copy();if(fault=="false arrival")measured[0]+=0.1;
    return {joints:measured,arrived:fault!="timeout",contacts:contacts,travelMm:fault=="cumulative travel" ? 0.6 : 0,
      penetrationMm:fault=="excess penetration" ? 0.002 : 0};
  }
}
