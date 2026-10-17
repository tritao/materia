import cadkit.modeling.AssemblyState;
import gearkit.*;
import machinekit.assembly.*;
import machinekit.assembly.AxisBuilder;
import machinekit.component.*;
import machinekit.motion.*;
import machinekit.hobbing.*;
import machinekit.power.*;
import materia.assembly.AssemblyFrames as F;
import materia.assembly.AssemblyRecord.AssemblyFrame;
import materia.assembly.AssemblyDefinition.AssemblyJointType;

typedef HobberTravel={id:String,lower:Float,upper:Float,initial:Float};

/** Six independent physical axes. There is deliberately no B/C mechanical gear coupling.
    CAD and prismatic coordinates mm; rotary coordinates rad. Nested joint IDs are explicit. */
class FlexsplineHobber extends MachineAssembly {
  public static final C="table/C";
  public static final B="spindle/B";
  public static final A="swivel/A";
  public final spec:GearSpec;
  public final hob:Hob;
  public final mandrel:ExpandingMandrel;
  public final bed:MineralCastBed;
  public final column:MineralCastColumn;
  public final table:RotaryTable;
  public final spindle:HobSpindle;
  public final swivel:SwivelHead;
  public final travel:Array<HobberTravel>;
  public final workOrigin:AssemblyFrame;
  public final precisionMounts:Array<{name:String,parent:String}> = [];
  final zero:Map<String,AssemblyFrame> = [];
  var driverNumber=0;
  public function new(?spec:GearSpec,gashes:Int=12){
    super();this.spec=spec==null?new GearSpec(0.3,200,8,20*Math.PI/180):spec;
    if(this.spec.internal)throw "external hobbing machine requires external gear";
    var profile=new ToothProfile(this.spec);hob=Hob.fromDesign(new HobDesign(ConjugateRack.of(profile),1,32,gashes),this.spec);
    bed=new MineralCastBed();column=new MineralCastColumn();
    includeAt("bed",bed,F.identity());
    var tableMotor=new TorqueMotor(new ComponentValues().setNumber("torqueConstant",2).setNumber("backEmfConstant",2/Math.sqrt(3)));
    table=new RotaryTable(tableMotor);
    includeAt("table",table,F.translation(0,-160,130));
    fixed("table-mount","bed/tablePlate","top","table/housing","mount");precisionMounts.push({name:"table-mount",parent:"bed/tablePlate"});
    includeAt("column",column,F.translation(0,180,140));
    fixed("column-mount","bed/columnPlate","top","column/basePlate","mount");precisionMounts.push({name:"column-mount",parent:"bed/columnPlate"});
    var diameter=2*(this.spec.rootRadius()-0.6);
    mandrel=new ExpandingMandrel(new ComponentValues().setNumber("outside",diameter-0.05).setNumber("workSeat",75).setNumber("height",90));
    fixAt("mandrel",mandrel,F.translation(0,-160,248),"table/platen");
    workOrigin=F.translation(0,-160,323);
    var blank=new TablePlaten(new ComponentValues().setNumber("outside",2*this.spec.tipRadius()).setNumber("bore",2*(this.spec.rootRadius()-0.6)).setNumber("height",this.spec.faceWidth));
    fixAt("blank",blank,workOrigin,"mandrel");
    var minX=Math.max(30,mandrel.n("outside")/2+hob.design.outsideDiameter/2+0.2);
    var centre=this.spec.pitchRadius()+hob.design.pitchDiameter/2+this.spec.profileShift*this.spec.module;
    if(centre<minX||centre>100)throw "gear/fixture exceeds radial machine capacity";
    travel=[{id:"X",lower:minX,upper:100,initial:centre},{id:"Y",lower:-10,upper:10,initial:0},{id:"Z",lower:-10,upper:60,initial:0}];
    // Z rails mount on the column's cast-in ground steel, never on the casting body.
    for(side in [-1,1]){
      var id=side<0?"railZLeft":"railZRight",insert=side<0?"column/railLeft":"column/railRight";
      fixAt(id,LinearRail.metric("HGR15",500),AxisBuilder.orient(side*55,80,165,[0.0,-1,0],[0.0,0,1]),insert);
      precisionMounts.push({name:id+"-mount",parent:insert});
    }
    var zPlate=new HobberBracket();addComponent("Zplate",zPlate);zero.set("Zplate",F.translation(0,40,403));
    moving("Z",Prismatic,"railZLeft","Zplate",workOrigin,[0.0,0,1],0,-10,60);
    for(side in [-1,1]){
      var name=side<0?"Left":"Right";
      fixAt("blockZ"+name,LinearRailBlock.metric("HGR15"),AxisBuilder.orient(side*55,65,443,[0.0,-1,0],[0.0,0,1]),"Zplate");
      fixAt("railX"+name,LinearRail.metric("HGR15",300),AxisBuilder.orient(-80,side*22,418,[0.0,0,1],[1.0,0,0]),"Zplate");
    }
    addComponent("Xbridge",new HobberBracket(new ComponentValues().setBoolean("cantilever",true)));
    zero.set("Xbridge",F.translation(0,-160,323));
    moving("X",Prismatic,"railXLeft","Xbridge",workOrigin,[1.0,0,0],centre,minX,100);
    for(side in [-1,1])fixAt(side<0?"blockXLeft":"blockXRight",LinearRailBlock.metric("HGR15"),
      AxisBuilder.orient(0,side*22,430,[0.0,0,1],[1.0,0,0]),"Xbridge");
    var swivelMotor=new TorqueMotor(new ComponentValues().setNumber("ratedSpeed",Math.PI/12).setNumber("maxSpeed",Math.PI/6)
      .setNumber("torqueConstant",2).setNumber("backEmfConstant",2/Math.sqrt(3)));
    swivel=new SwivelHead(swivelMotor,null,Math.PI/12);
    // Local swivel +Z -> global -X; local +X -> global +Y. Pivot lies on the hob centre line.
    includeAt("swivel",swivel,F.fromRotationMatrix(200,-160,323,[0.0,0,-1,1,0,0,0,-1,0]));
    fixExisting("swivel-mount","Xbridge","swivel/housing");
    var e0=[-1.0,0,0,0,0,-1,0,-1,0];
    fixAt("railY",LinearRail.metric("HGR15",180),F.fromRotationMatrix(60,-80,373,e0),"swivel/head");
    addComponent("Yplate",new HobberBracket(new ComponentValues().setNumber("width",80).setNumber("depth",80).setNumber("height",20)));
    zero.set("Yplate",F.fromRotationMatrix(60,-160,373,e0));
    moving("Y",Prismatic,"railY","Yplate",workOrigin,[0.0,-1,0],0,-10,10);
    fixAt("blockY",LinearRailBlock.metric("HGR15"),F.fromRotationMatrix(60,-160,373,e0),"Yplate");
    var rated=4000*2*Math.PI/60;
    var spindleMotor=new TorqueMotor(new ComponentValues().setNumber("outside",62).setNumber("bore",25).setNumber("height",100)
      .setNumber("continuousTorque",1500/rated).setNumber("peakTorque",3000/rated).setNumber("ratedSpeed",rated).setNumber("maxSpeed",8000*2*Math.PI/60)
      .setNumber("rotorInertia",0.0002).setNumber("coggingTorque",0.003).setInteger("poleCount",4).setInteger("coggingOrder",12)
      .setNumber("torqueConstant",0.4).setNumber("backEmfConstant",0.4/Math.sqrt(3)).setNumber("phaseResistance",0.25));
    spindle=new HobSpindle(spindleMotor,new HobArbor(new ComponentValues().setNumber("height",120)));
    includeAt("spindle",spindle,F.fromRotationMatrix(0,80,323,e0));fixExisting("spindle-mount","Yplate","spindle/housing");
    fixAt("hob",hob,F.fromRotationMatrix(0,-140,323,e0),"spindle/arbor");
    fixAt("outboard",new OutboardSupport(new ComponentValues().setNumber("height",30)),F.fromRotationMatrix(0,-205,323,e0),"Yplate");
    var supply=new ThreePhaseRectifier(new AcMains(ThreePhase,new AcRmsVoltage(400),50),new DcCurrent(40),6,240,120,100,10000);
    fixAt("powerSupply",supply,F.translation(-250,130,130),"bed/liftLeft");exposePort("mains","powerSupply","mains");
    drive("X","Zplate","Xbridge",[1.0,0,0],F.translation(0,70,443),300);
    drive("Z","column/railRight","Zplate",[0.0,0,1],F.translation(85,60,443),350);
    drive("Y","swivel/head","Yplate",[0.0,-1,0],F.translation(90,-160,373),250);
    bind("table-drive",C,"table/motor");bind("swivel-drive",A,"swivel/motor");bind("hob-drive",B,"spindle/motor",true);
    bindMotorBrake("swivel-drive","swivel/clamp","swivel.clampRelease","swivel.clampReleased",1000,30);
    FoundationIntegrity.check(bed).throwIfErrors();FoundationIntegrity.check(column).throwIfErrors();FoundationIntegrity.checkMounts(this,precisionMounts).throwIfErrors();
    exposeConnector("work","mandrel","work");exposeConnector("hobCentre","hob","centre");
  }
  function includeAt(id:String,a:MachineAssembly,pose:AssemblyFrame):Void {
    include(id,a,pose);var positions=a.solvedPoses();for(m in a.components()){var p=positions.get(m.id);if(p==null)throw "missing package pose";zero.set(id+"/"+m.id,F.compose(pose,p));}
  }
  function fixed(id:String,parent:String,pc:String,child:String,cc:String):Void addMate(id,Fixed,parent,pc,child,cc);
  function fixAt(id:String,p:MachineComponent,pose:AssemblyFrame,parent:String):Void {addComponent(id,p);zero.set(id,pose);fixExisting(id+"-mount",parent,id);}
  function fixExisting(id:String,parent:String,child:String):Void {
    var p=required(parent),c=required(child),frame=F.alongY(c.x,c.y,c.z,0,0,1);
    addMemberConnector(parent,id+"Parent",F.compose(F.inverse(p),frame));addMemberConnector(child,id+"Child",F.compose(F.inverse(c),frame));
    addMate(id,Fixed,parent,id+"Parent",child,id+"Child");
  }
  function required(id:String):AssemblyFrame {var p=zero.get(id);if(p==null)throw 'missing zero pose $id';return p;}
  function moving(id:String,type:AssemblyJointType,parent:String,child:String,origin:AssemblyFrame,d:Array<Float>,initial:Float,lower:Float,upper:Float):Void {
    var frame=F.alongY(origin.x,origin.y,origin.z,d[0],d[1],d[2]);
    addMemberConnector(parent,id+"Parent",F.compose(F.inverse(required(parent)),frame));
    addMemberConnector(child,id+"Child",F.compose(F.inverse(required(child)),frame));
    addMateOnAxis(id,type,parent,id+"Parent",child,id+"Child",{x:0,y:1,z:0},initial,{lower:lower,upper:upper,velocity:null,effort:null,overtravel:5});
  }
  function drive(id:String,parent:String,carriage:String,d:Array<Float>,nutOrigin:AssemblyFrame,length:Float):Void {
    var motor=ServoMotor.model("GENERIC-HV-SERVO-400W"),screw=new BallScrew(length),nut=new BallNut();
    var start=[nutOrigin.x-d[0]*100,nutOrigin.y-d[1]*100,nutOrigin.z-d[2]*100];
    var up=Math.abs(d[2])>0.5?[0.0,1,0]:[0.0,0,1];
    var mp=AxisBuilder.orient(start[0]-d[0]*motor.rating.shaftLength,start[1]-d[1]*motor.rating.shaftLength,start[2]-d[2]*motor.rating.shaftLength,up,d);
    fixAt(id+"motor",motor,mp,parent);
    addComponent(id+"screw",screw);zero.set(id+"screw",AxisBuilder.orient(start[0],start[1],start[2],up,d));
    var f=F.alongY(start[0],start[1],start[2],d[0],d[1],d[2]);
    addMemberConnector(id+"motor",id+"turnParent",F.compose(F.inverse(required(id+"motor")),f));
    addMemberConnector(id+"screw",id+"turnChild",F.compose(F.inverse(required(id+"screw")),f));
    fixAt(id+"nut",nut,AxisBuilder.orient(nutOrigin.x-d[0]*nut.bodyLength/2,nutOrigin.y-d[1]*nut.bodyLength/2,nutOrigin.z-d[2]*nut.bodyLength/2,up,d),carriage);
    var ratio=addTransmission(id+"lead",id,id+"screw-turn",Transmission.LeadScrew(id+"screw",id+"nut"),Opposite);
    var initial=travel.filter(t->t.id==id)[0].initial;
    addMate(id+"screw-turn",Continuous,id+"motor",id+"turnParent",id+"screw",id+"turnChild",ratio*initial);
    var near=ScrewSupportUnit.bk12(),far=ScrewSupportUnit.bf12();
    fixAt(id+"fixed",near,AxisBuilder.orient(start[0]+d[0]*37.5,start[1]+d[1]*37.5,start[2]+d[2]*37.5,up,d),parent);
    fixAt(id+"floating",far,AxisBuilder.orient(start[0]+d[0]*(length-12.5),start[1]+d[1]*(length-12.5),start[2]+d[2]*(length-12.5),up,d),parent);
    supportScrew(id+"lead",near.support,far.support);
    var coupling=new ShaftCoupling(motor.rating.shaftDiameter,12);
    if(motor.rating.shaftLength<coupling.length/2)throw "motor shaft cannot engage half coupling";
    fixAt(id+"coupling",coupling,AxisBuilder.orient(start[0]-d[0]*coupling.length/2,start[1]-d[1]*coupling.length/2,start[2]-d[2]*coupling.length/2,up,d),id+"screw");
    fixAt(id+"motor-shaft",motor.shaft(),mp,id+"screw");
    fixAt(id+"scale",new LinearScale(length,10000,true),AxisBuilder.orient(start[0]+30,start[1]+30,start[2],up,d),parent);
    addEncoder(id+"position",id,id+"scale");bind(id+"drive",id+"screw-turn",id+"motor");
  }
  function bind(id:String,joint:String,motor:String,process:Bool=false):Void {
    driverNumber++;var driver=id+"-driver";
    fixAt(driver,new MotorDriver("GENERIC-400V-SERVO-AMP",new PhaseRmsCurrent(20)),F.translation(-280+driverNumber*45,230,140),"bed/liftLeft");
    connectPorts(id+"-power","powerSupply","power"+driverNumber,driver,"power");
    addMotor(id,joint,motor,driver,1,null,process?{speedChannel:"hob.speed",directionChannel:"hob.direction",radiansPerSpeedUnit:2*Math.PI/60}:null);
  }
  public function hobToWork(state:AssemblyState):AssemblyFrame {
    var work=F.compose(state.worldPose("mandrel"),F.translation(0,0,mandrel.n("workSeat")));
    var cutter=F.compose(state.worldPose("hob"),F.translation(0,0,hob.n("height")/2));
    return F.compose(F.inverse(work),cutter);
  }
}
