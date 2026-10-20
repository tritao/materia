package machinekit.milling;

import machinekit.assembly.MachineAssembly;
import machinekit.component.MachineComponent;
import machinekit.motion.LeadScrew;
import machinekit.motion.LeadScrewThread;
import machinekit.standard.SocketHeadCapScrew;
import machinekit.standard.FlatWasher;
import machinekit.structural.SlideInSlotNut;
import materia.assembly.AssemblyFrames as F;
import materia.assembly.AssemblyRecord.AssemblyFrame;

/** RÖHM RKE92 / 178411 external drawing reference, not imported supplier CAD.
 * Nominal screw/slide reconstruction is kinematic only: the manufacturer's
 * mechanical-hydraulic force multiplier is not simulated or assigned a rating.
 * X across jaws, Y from front of body toward drive, Z from mounting plane.
 */
class Rke92Vise extends MachineAssembly {
 public static inline var JAW_WIDTH:Float=92;
 public static inline var OPENING:Float=138;
 public static inline var BODY_LENGTH:Float=276;
 public static inline var FIXED_FACE:Float=80;
 public static inline var BED_HEIGHT:Float=80;
 public static inline var JAW_HEIGHT:Float=32;
 public static inline var PARALLEL_HEIGHT:Float=26;
 public static inline var SCREW_LEAD:Float=4; // nominal internal reconstruction, not supplier data
 public final seat:AssemblyFrame;
 public final stockWidth:Float;
 public final stockDepth:Float;
 final zero:Map<String,AssemblyFrame> = [];
 public function new(stockWidth:Float=60,stockDepth:Float=60) {
  super();
  if(!Math.isFinite(stockWidth+stockDepth)||stockWidth<=0||stockWidth>JAW_WIDTH||stockDepth<30||stockDepth>OPENING)
   throw "RKE92 stock must fit the jaw width and opening";
  this.stockWidth=stockWidth;this.stockDepth=stockDepth;
  seat=F.translation(0,FIXED_FACE+stockDepth/2,BED_HEIGHT+PARALLEL_HEIGHT);
  fixed("body",new Rke92VisePart("body"),F.identity(),null);
  fixed("fixedJaw",new Rke92VisePart("jaw-back",39),F.translation(0,55,BED_HEIGHT),"body");
  fixed("fixedPlate",new Rke92VisePart("jaw-plate"),F.translation(0,FIXED_FACE,BED_HEIGHT),"fixedJaw");
  var reverse=[-1.0,0,0, 0,-1,0, 0,0,1];
  var slide=F.fromRotationMatrix(0,FIXED_FACE+stockDepth+25,BED_HEIGHT,reverse);
  addComponent("slide",new Rke92VisePart("slide",31),slide);zero.set("slide",slide);
  addMemberConnector("body","jawGuide",F.translation(0,FIXED_FACE+25,BED_HEIGHT));
  addMemberConnector("slide","guide",F.fromRotationMatrix(0,0,0,reverse));
  addMateOnAxis("opening","prismatic","body","jawGuide","slide","guide",{x:0,y:1,z:0},stockDepth,
   {lower:0,upper:OPENING,velocity:null,effort:null});
  fixed("movingPlate",new Rke92VisePart("jaw-plate"),F.fromRotationMatrix(0,FIXED_FACE+stockDepth,BED_HEIGHT,reverse),"slide");
  for(jaw in ["fixedPlate","movingPlate"])for(x in [-31.5,31.5]) {
   var head=F.fromRotationMatrix(x,-6.4,14,[1,0,0, 0,0,1, 0,-1,0]);
   fixed(jaw+"Bolt"+x,SocketHeadCapScrew.metric("M6",30),F.compose(zero.get(jaw),head),jaw);
  }
  for(side in [-1,1])fixed("parallel"+side,new MillPanel(80,5,PARALLEL_HEIGHT),
   F.translation(0,FIXED_FACE+(side<0?4:stockDepth-4),BED_HEIGHT),"body");
  fixed("stop",new Rke92VisePart("stop"),F.translation(-stockWidth/2-3,seat.y,BED_HEIGHT),"body");
  fixed("stopBolt",SocketHeadCapScrew.metric("M4",12),F.translation(-stockWidth/2-9,seat.y,BED_HEIGHT+6),"stop");
  var shaft=F.fromRotationMatrix(0,FIXED_FACE,48,[1,0,0, 0,0,1, 0,-1,0]);
  var screw=new LeadScrew(new LeadScrewThread(MetricTrapezoidal,20,SCREW_LEAD),196);
  addComponent("screw",screw,shaft);zero.set("screw",shaft);
  addMemberConnector("body","screwSeat",shaft);addMemberConnector("screw","seat",F.identity());
  addMateOnAxis("screw-turn","continuous","body","screwSeat","screw","seat",{x:0,y:0,z:1},stockDepth*2*Math.PI/SCREW_LEAD);
  addCoupling("nominal-jaw-lead","opening","screw-turn",2*Math.PI/SCREW_LEAD);
  fixed("drive",new Rke92VisePart("drive"),F.translation(0,BODY_LENGTH,48),"screw");
  // Local X=+/-80 maps onto the mill's +/-80 mm table slots after installation.
  var washer=FlatWasher.metric("M6");
  for(side in [-1,1])for(y in [30.0,200.0]) {
   var tag=side+"-"+y,flip=side<0?[-1.0,0,0,0,-1,0,0,0,1]:[1.0,0,0,0,1,0,0,0,1];
   fixed("clamp"+tag,new Rke92VisePart("clamp"),F.fromRotationMatrix(side*70,y,12,flip),"body");
   fixed("heel"+tag,new MillPanel(16,24,12),F.translation(side*92,y,0),"body");
   fixed("washer"+tag,washer,F.translation(side*80,y,22),"body");
   fixed("mountBolt"+tag,SocketHeadCapScrew.metric("M6",35),F.translation(side*80,y,22+washer.thickness),"body");
   fixed("slotNut"+tag,new SlideInSlotNut(17.4,24,5.8,9.6,2,"M6"),F.translation(side*80,y,-6.2),"body");
  }
  for(y in [30.0,170.0])fixed("key"+y,new Rke92VisePart("key"),F.translation(0,y,0),"body");
  addMemberConnector("body","stockSeat",seat);exposeConnector("stockSeat","body","stockSeat");
  exposeConnector("mount","body","mount");
 }
 function fixed(id:String,c:MachineComponent,p:AssemblyFrame,parent:Null<String>):Void {
  addComponent(id,c,p);zero.set(id,p);if(parent==null)return;
  addMemberConnector(parent,"to-"+id,F.compose(F.inverse(zero.get(parent)),p));
  addMemberConnector(id,"seat",F.identity());
  addMate("mount-"+id,"fixed",parent,"to-"+id,id,"seat");
 }
}
