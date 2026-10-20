package machinekit.milling;
import machinekit.assembly.MachineAssembly;
import machinekit.component.MachineComponent;
import machinekit.standard.*;
import machinekit.structural.SlideInSlotNut;
import materia.assembly.AssemblyFrames as F;
import materia.assembly.AssemblyRecord.AssemblyFrame;
/** Supplier workholding on a manual zero-point plate. Pneumatic AR155-I is the mill default; CV15075 remains available. */
class ZppWorkholding extends MachineAssembly {
 public static inline var ADAPTER:Float=20;
 public static inline var PLATE:Float=30;
 public static inline var PNEUMATIC_SEAT:Float=126;
 public static inline var RELEASE_CLEARANCE:Float=2; // per jaw, packaging preview assumption; not a rated pneumatic stroke
 public final pneumatic:Bool;
 public static inline var PARALLELS:Float=12;
 public final seat:AssemblyFrame;
 final zero:Map<String,AssemblyFrame> = [];
 public function new(stockWidth:Float=60,stockDepth:Float=60,pneumatic:Bool=true) {
  super();
  this.pneumatic=pneumatic;
  if(!Math.isFinite(stockWidth+stockDepth)||stockWidth<=0||stockDepth<=0)
   throw "Workholding requires finite positive blank dimensions";
  if(pneumatic) {
   if(stockWidth>130||stockDepth<59.5||stockDepth>61.5)
    throw "The reviewed AR155-I hard-jaw preset accepts 59.5–61.5 mm blanks; other sizes require a reviewed jaw setup";
  }else if(stockWidth<50||stockWidth>75||stockDepth<10||stockDepth>90)
   throw "CV15075 forward jaws require a 10–90 mm opening; these parallels support 50–75 mm wide blanks";
  var plateHeight=pneumatic?PLATE:25,base=ADAPTER+plateHeight;
  seat=F.translation(0,0,base+(pneumatic?PNEUMATIC_SEAT:ZppComponent.BED+PARALLELS));
  fixed("body",new MillFixtureAdapter(ADAPTER,pneumatic),F.identity(),null);
  fixed("plate",new ZppComponent(pneumatic?"plate96":"plate"),F.translation(0,0,ADAPTER),"body");
  var mountHalf=pneumatic?64:41;
  for(x in [-mountHalf,mountHalf])for(y in [-mountHalf,mountHalf])
   fixed('plateBolt$x-$y',SocketHeadCapScrew.metric("M10",pneumatic?30:25),F.translation(x,y,ADAPTER+(pneumatic?16:12)),"plate");
  fixed("vise",new ZppComponent(pneumatic?"ar-body":"body"),F.translation(0,0,base),"plate");
  if(!pneumatic)fixed("screw",new ZppComponent("screw"),F.translation(0,0,base),"vise");
  var rawGap=pneumatic?59.5:ZppComponent.RAW_GAP,delta=(stockDepth-rawGap)/2;
  for(side in [-1,1]) {
   var id=side<0?"negative-jaw":"positive-jaw";
   var pose=F.translation(side*delta,0,base);
   addComponent(id,new ZppComponent(pneumatic?(side<0?"ar-negative-carrier":"ar-positive-carrier"):id),pose);zero.set(id,pose);
   addMemberConnector("vise",id+"Guide",F.translation(-side*rawGap/2,0,0));
   addMemberConnector(id,"guide",F.identity());
   addMateOnAxis(side<0?"opposite-jaw":"half-opening","prismatic","vise",id+"Guide",id,"guide",{x:side,y:0,z:0},stockDepth/2,
    {lower:pneumatic?stockDepth/2:0,upper:pneumatic?stockDepth/2+RELEASE_CLEARANCE:45,velocity:null,effort:null});
   if(pneumatic)for(kind in ["soft-jaw","hard-jaw"])
    fixed((side<0?"negative-":"positive-")+kind,new ZppComponent("ar-"+(side<0?"negative-":"positive-")+kind),pose,id);
  }
  addCoupling("self-centering","half-opening","opposite-jaw",1);
  var studHalf=pneumatic?48:26;
  for(x in [-studHalf,studHalf])for(y in [-studHalf,studHalf]) {
   var z=base+5.08;
   fixed('pullStud$x-$y',new ZppComponent(pneumatic?"stud96":"stud"),F.translation(x,y,z),"vise");
   fixed('studLink$x-$y',new ZppComponent(pneumatic?"stud-link96":"stud-link"),F.translation(x,y,z),"vise");
  }
  if(!pneumatic)for(side in [-1,1])fixed("parallel"+side,new MillPanel(stockDepth-0.2,5,PARALLELS),
   F.translation(0,side*(stockWidth/2-2.5),base+ZppComponent.BED),"vise");
  if(pneumatic) {
   exposePort("airA","vise","airA");exposePort("airB","vise","airB");
   addSwitch("clamped-position","half-opening",{lower:stockDepth/2,upper:stockDepth/2+0.05,velocity:null,effort:null},0.01);
   addSwitch("released-position","half-opening",{lower:stockDepth/2+RELEASE_CLEARANCE-0.05,upper:stockDepth/2+RELEASE_CLEARANCE,velocity:null,effort:null},0.01);
  }
  var washer=FlatWasher.metric("M6");
  for(x in pneumatic?[-85,85]:[-70,70])for(y in [-80,80]) {
   var tag=x+"-"+y;
   fixed("washer"+tag,washer,F.translation(x,y,ADAPTER),"body");
   fixed("tableBolt"+tag,SocketHeadCapScrew.metric("M6",35),F.translation(x,y,ADAPTER+washer.thickness),"body");
   fixed("slotNut"+tag,new SlideInSlotNut(17.4,24,5.8,9.6,2,"M6"),F.translation(x,y,-6.2),"body");
  }
  addMemberConnector("body","stockSeat",seat);exposeConnector("stockSeat","body","stockSeat");
  exposeConnector("mount","body","mount");
 }
 function fixed(id:String,c:MachineComponent,p:AssemblyFrame,parent:Null<String>):Void {
  addComponent(id,c,p);zero.set(id,p);if(parent==null)return;
  addMemberConnector(parent,"to-"+id,F.compose(F.inverse(zero.get(parent)),p));
  addMemberConnector(id,"seat",F.identity());addMate("mount-"+id,"fixed",parent,"to-"+id,id,"seat");
 }
}
