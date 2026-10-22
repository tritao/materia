package machinekit.assembly;
import cadkit.modeling.AssemblyState;
import machinekit.component.*;
import machinekit.motion.*;
import machinekit.standard.*;
import materia.assembly.AssemblyFrames as F;
import materia.assembly.AssemblyRecord.AssemblyFrame;

/** Complete fixed/floating SFU20 axis. Purchased parts use their catalogue geometry;
 * only base, rail risers, carriage and motor bracket are fabricated.
 * DSG depths (12/10 mm), 16 mm race root and 6 mm retaining-nut height are explicit reference assumptions.
 */
class SupportedSfu20Axis extends MachineAssembly {
 public final layout:Sfu20AxisLayout;
 public final screw:SfuDrawingScrew;
 public final nut:SfuNut;
 public final housing:DsgNutHousing;
 public final fixed:VallderScrewSupport;
 public final floating:VallderScrewSupport;
 public final rail:LinearRail;
 public final block:LinearRailBlock;
 public final hardware:Array<{id:String,host:String,grip:Float,engagement:Float,depth:Float}> = [];
 public final threadedPairs:Array<String> = [];
 public function new(?layout:Sfu20AxisLayout) {
  super();if(layout==null)layout=new Sfu20AxisLayout();this.layout=layout;
  var l=layout;
  screw=new SfuDrawingScrew("SFU2005",l.screwLength,16);nut=new SfuNut("SFU2005",16);
  housing=new DsgNutHousing("DSG20H",12,10);fixed=new VallderScrewSupport("BK15");floating=new VallderScrewSupport("BF15");
  rail=LinearRail.metric("HGH20CA",l.railLength,LinearRail.MEASURED);block=LinearRailBlock.metric("HGH20CA",LinearRailBlock.SUPPLIER);
  addComponent("base",new Sfu20AxisPart("base",l));
  attach("fixed",fixed,"base",{x:0,y:28,z:l.fixedCentreZ(),qx:0,qy:1,qz:0,qw:0});
  attach("floating",floating,"base",F.translation(0,28,l.floatingCentreZ()));
  attach("motorBracket",new Sfu20AxisPart("motorBracket",l),"base",F.identity());
  attach("motor",NemaStepper.model("23HS22-2804S"),"base",F.translation(0,28,0));
  addComponent("screw",screw);addMemberConnector("base","screwSeat",Solids.axial(0,28,40));addMemberConnector("screw","installation",Solids.axial(0,0,0));
  addMate("screwTurn","continuous","base","screwSeat","screw","installation");
  attach("coupling",new ShaftCoupling(6.35,12,25,40,[{z:6.0,angle:Math.PI/2},{z:34.0,angle:Math.PI/2}]),"screw",F.translation(0,0,-31));
  attach("retainingNut",new FinePitchNut(),"screw",F.translation(0,0,20));
  attach("outputRing",RetainingRing.forShaft(15),"screw",F.translation(0,0,screw.grooveStart+0.075));
  for(side in ["Left","Right"]) {
   var x=side=="Left" ? -l.railSpacing/2 : l.railSpacing/2;
   attach("riser"+side,new Sfu20AxisPart("riser"+side,l),"base",F.translation(x,0,140));
   attach("rail"+side,rail,"base",F.translation(x,37.5,140));
  }
  addComponent("carriage",new Sfu20AxisPart("carriage",l));
  addMemberConnector("base","travelSeat",Solids.axial(0,50,200));addMemberConnector("carriage","slide",Solids.axial(0,0,0));
  addMateOnAxis("travel","prismatic","base","travelSeat","carriage","slide",{x:0,y:1,z:0},0,{lower:0,upper:l.travel,velocity:null,effort:null});
  attach("housing",housing,"carriage",{x:0,y:0,z:-20,qx:0,qy:0,qz:1,qw:0});
  attach("nut",nut,"carriage",{x:0,y:-22,z:-40,qx:0,qy:0,qz:-Math.sqrt(0.5),qw:Math.sqrt(0.5)});
  for(side in ["Left","Right"]) {
   var x=side=="Left" ? -l.railSpacing/2 : l.railSpacing/2;
   attach("block"+side,block,"carriage",F.translation(x,-12.5,0));
   addConstraint("guide"+side,"prismatic","rail"+side,"axis","block"+side,"rail");
  }
  addCoupling("lead","travel","screwTurn",2*Math.PI/screw.thread.signedLead());
  for(z in [-7.5,7.5])for(x in [-27.0,27])bolt('bk-$x-$z',"M6",45,"base",F.alongY(x,31.5,l.fixedCentreZ()+z,0,1,0),"base",31.5,14);
  for(x in [-27.0,27])bolt('bf-$x',"M6",45,"base",F.alongY(x,31.5,l.floatingCentreZ(),0,1,0),"base",31.5,14);
  for(x in [-38.0,38])bolt('motorFoot-$x',"M6",20,"base",F.alongY(x,8,30,0,1,0),"base",8,14);
  for(side in [-1,1])for(z in [150.0,140+l.railLength-10])bolt('riser-$side-$z',"M6",25,"base",F.alongY(side*(l.railSpacing/2+16),13.5,z,0,1,0),"base",13.5,14);
  for(side in ["Left","Right"]) {
   var x=side=="Left" ? -l.railSpacing/2 : l.railSpacing/2;
   for(z in rail.holePositions)bolt('rail-$side-$z',"M5",20,"base",F.alongY(x,29,140+z,0,1,0),"riser"+side,9,12);
   for(dx in [-16.0,16])for(z in [-18.0,18])bolt('block-$side-$dx-$z',"M5",12,"carriage",F.alongY(x+dx,6,z,0,1,0),"block"+side,6,6);
  }
  for(x in [-24.0,24])for(z in [-12.0,12])bolt('housing-$x-$z',"M6",25,"carriage",F.alongY(x,14,z,0,1,0),"housing",14,12);
  for(p in housing.flangePattern())bolt('nut-${hardware.length}',"M6",20,"carriage",F.alongY(-p.x,-22-p.y,-30,0,0,-1),"housing",10,10);
  for(x in [-23.57,23.57])for(y in [-23.57,23.57])bolt('motor-$x-$y',"M4",10,"base",F.alongY(x,28+y,-5,0,0,-1),"motorBracket",5,6);
  // Radial screws deliberately contact shaft envelopes; only their shaft-contact pairs are exempted.
  attach("clampMotor",SocketHeadCapScrew.metric("M4",10),"coupling",fastenerPose(F.alongY(0,12.5,6,0,1,0)));
  attach("clampScrew",SocketHeadCapScrew.metric("M4",8),"coupling",fastenerPose(F.alongY(0,12.5,34,0,1,0)));
  threadedPairs.push(pair("clampMotor","coupling"));threadedPairs.push(pair("clampScrew","coupling"));
  threadedPairs.push(pair("clampMotor","motor"));threadedPairs.push(pair("clampScrew","screw"));
 }
 // Connector joint axes are +Y; fastener CAD axes are +Z.
 static function fastenerPose(seat:AssemblyFrame):AssemblyFrame return F.compose(seat,F.inverse(Solids.axial(0,0,0)));
 static function pair(a:String,b:String):String return Reflect.compare(a,b)<0 ? a+"/"+b : b+"/"+a;
 function attach(id:String,component:MachineComponent,parent:String,pose:AssemblyFrame):Void {
  addComponent(id,component);addMemberConnector(parent,id+"Seat",pose);addMemberConnector(id,"installation",F.identity());addMate(id+"Fixed","fixed",parent,id+"Seat",id,"installation");
 }
 function bolt(id:String,size:String,length:Float,parent:String,seat:AssemblyFrame,host:String,grip:Float,depth:Float):Void {
  var p=SocketHeadCapScrew.metric(size,length),engagement=length-grip;
  if(engagement<p.diameter || engagement>depth+1e-9)throw "Invalid SFU20 hardware engagement: "+id;
  attach(id,p,parent,fastenerPose(seat));hardware.push({id:id,host:host,grip:grip,engagement:engagement,depth:depth});threadedPairs.push(pair(id,host));
 }
 public function setTravel(state:AssemblyState,value:Float):Void {
  if(!Math.isFinite(value) || value<0 || value>layout.travel)throw "Travel outside SFU20 axis limits";state.setJoint("travel",value);
 }
 /** Every component pair, including hardware, except explicitly listed threaded/contact pairs.
  * Samples include eight shaft phases to expose rotating clamp-head clearance.
  */
 public function installationClearances():Array<{travel:Float,a:String,b:String,intersectionMm3:Float}> {
  var parts=components(),state=new AssemblyState(definition()),cache=new PosedParts(),memo:Map<String,Float> = [],rows:Array<{travel:Float,a:String,b:String,intersectionMm3:Float}> = [];
  var stops=[0.0,0.625,1.25,1.875,2.5,3.125,3.75,4.375,layout.travel/2,layout.travel-0.625,layout.travel];
  try {
   for(travel in stops)if(travel>=0 && travel<=layout.travel) {
    setTravel(state,travel);var poses=solvedPoses(state);
    for(i in 0...parts.length)for(j in i+1...parts.length) {
     var a=parts[i],b=parts[j];if(threadedPairs.indexOf(pair(a.id,b.id))>=0)continue;
     var pa=poses.get(a.id),pb=poses.get(b.id);if(pa==null || pb==null)throw "Missing SFU20 pose";
     var relative=F.compose(F.inverse(pa),pb),key=pair(a.id,b.id)+":"+rounded(relative),volume=memo.get(key);
     if(volume==null){volume=cache.overlap(a.component,pa,b.component,pb);memo.set(key,volume);}
     rows.push({travel:travel,a:a.id,b:b.id,intersectionMm3:volume});
    }
   }
   cache.close();return rows;
  }catch(e:Dynamic){cache.close();throw e;}
 }
 static function rounded(p:AssemblyFrame):String return [p.x,p.y,p.z,p.qx,p.qy,p.qz,p.qw].map(v -> Std.string(Math.ffloor(v*1000000+0.5))).join(",");
}
