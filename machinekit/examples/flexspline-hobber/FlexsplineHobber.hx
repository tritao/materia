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
  /** Work-frame +X (radial) -> machine +Y; work-frame -Y (hob) -> machine +X. */
  public static final MACHINE_FROM_WORK=F.fromRotationMatrix(0,0,637,[0.0,-1,0,1,0,0,0,0,1]);
  public static final C="table/C";
  public static final B="spindle/B";
  public static final A="swivel/A";
  public final spec:GearSpec;
  public final hob:Hob;
  public final mandrel:ExpandingMandrel;
  public final bed:MineralCastBed;
  public final column:MineralCastColumn;
  public final table:machinekit.assembly.MachineAssembly;
  public final tableSpec:machinekit.motion.RotaryAxisSpec;
  public final spindle:HobSpindle;
  public final swivel:SwivelHead;
  public final travel:Array<HobberTravel>;
  public final workOrigin:AssemblyFrame;
  public final precisionMounts:Array<{name:String,parent:String}> = [];
  public final mountContacts:Array<{parent:String,child:String}> = [];
  final zero:Map<String,AssemblyFrame> = [];
  var driverNumber=0;
  public function new(?spec:GearSpec,gashes:Int=12,detailedHob:Bool=false,?rotaryAxis:machinekit.motion.PrecisionRotaryAxis){
    super();this.spec=spec==null?new GearSpec(0.3,200,8,20*Math.PI/180):spec;
    if(this.spec.internal)throw "external hobbing machine requires external gear";
    var profile=new ToothProfile(this.spec);hob=Hob.fromDesign(new HobDesign(ConjugateRack.of(profile),1,32,gashes),this.spec,40,16,false,detailedHob);
    bed=new MineralCastBed();column=new MineralCastColumn();
    includeAt("bed",bed,F.translation(HobberXLayout.front().bedCentre,0,-590));
    var footCentres=FoundationLayout.feet();
    for(i in 0...footCentres.length){
      var p=footCentres[i],x=p.x+HobberXLayout.front().bedCentre,y=p.y;
      var roof=-590+machinekit.structural.CastingFootPocket.ROOF,plateBottom=roof-machinekit.structural.CastingFootPocket.PLATE_THICKNESS;
      fixAt("footSeat"+i,new machinekit.structural.LevelingSeat(),F.translation(x,y,plateBottom),"bed/casting",F.alongY(x,y,roof,0,0,1));
      var nutValues=new ComponentValues().setToken("variant","nut");
      fixAt("footNut"+i,new machinekit.structural.LevelingSeat(nutValues),F.translation(x,y,plateBottom-19),"footSeat"+i,F.alongY(x,y,plateBottom,0,0,1));
      fixAt("footLocknut"+i,new machinekit.structural.LevelingSeat(nutValues),F.translation(x,y,plateBottom-38),"footNut"+i,F.alongY(x,y,plateBottom-19,0,0,1));
      fixAt("foot"+i,new machinekit.structural.LevelingFoot(new ComponentValues().setNumber("recessDepth",machinekit.structural.CastingFootPocket.ROOF)),F.translation(x,y,-637),"footNut"+i);
    }
    var installed:machinekit.motion.PrecisionRotaryAxis=rotaryAxis==null?new machinekit.motion.PurchasedRotaryTable():rotaryAxis;
    table=installed.assembly();tableSpec=installed.specification();
    var tableBase=FoundationLayout.TABLE_FACE-tableSpec.number("height"),seat=FoundationLayout.TABLE_SEAT;
    if(tableBase<seat)throw "Rotary-axis height exceeds reserved table well; resize foundation seat before installation";
    var tableParent="bed/tablePlate";
    if(tableBase>seat){
      fixAt("tableAdapter",new machinekit.motion.PurchasedRotaryPart(new ComponentValues().setToken("variant","adapter").setNumber("adapterHeight",tableBase-seat).setNumber("adapterOutside",tableSpec.number("housingDiameter"))),F.translation(0,0,seat),tableParent);
      precisionMounts.push({name:"tableAdapter-mount",parent:tableParent});tableParent="tableAdapter";
    }
    includeAt("table",table,F.translation(0,0,tableBase));
    fixed("table-mount",tableParent,"top","table/"+tableSpec.mountPart,"mount");
    if(tableParent=="bed/tablePlate")precisionMounts.push({name:"table-mount",parent:tableParent});
    for(name in table.portNames()){var port=table.port(name,"table");exposePort("table."+name,port.instanceId,port.portName);}
    var diameter=2*(this.spec.rootRadius()-0.6),workSeat=363-FoundationLayout.TABLE_FACE;
    mandrel=new ExpandingMandrel(new ComponentValues().setNumber("outside",diameter-0.05).setNumber("workSeat",workSeat)
      .setNumber("height",workSeat+Math.max(16,this.spec.faceWidth+8)+2).setNumber("sleeveStart",workSeat).setNumber("sleeveLength",Math.max(16,this.spec.faceWidth+8))
      .setNumber("stopDiameter",diameter+0.6).setBoolean("separateSleeve",true).setNumber("baseDiameter",Math.max(tableSpec.number("facePcd")+30,diameter+10)).setNumber("baseHeight",25).setNumber("baseBore",40).setNumber("baseBoreDepth",20).setNumber("basePilotDiameter",tableSpec.number("centeringDiameter")).setNumber("basePilotDepth",tableSpec.number("centeringDiameter")>0?3:0).setNumber("baseBoltPcd",tableSpec.number("facePcd")));
    fixAt("mandrel",mandrel,F.translation(0,0,FoundationLayout.TABLE_FACE),"table/"+tableSpec.workPart,F.alongY(0,20,FoundationLayout.TABLE_FACE,0,0,1));
    fixAt("mandrelSleeve",new MandrelSleeve(new ComponentValues().setNumber("outside",diameter-0.05).setNumber("wall",mandrel.parameterNumber("sleeveWall")).setNumber("height",mandrel.parameterNumber("sleeveLength"))),F.translation(0,0,FoundationLayout.TABLE_FACE+mandrel.parameterNumber("sleeveStart")),"mandrel");
    workOrigin=F.compose(MACHINE_FROM_WORK,F.translation(0,0,363));
    var blank=new FlexsplineCupBlank(new ComponentValues().setNumber("outside",2*this.spec.tipRadius()).setNumber("bore",diameter).setNumber("bodyOutside",2*this.spec.rootRadius()).setNumber("bandWidth",this.spec.faceWidth));
    fixAt("blank",blank,F.translation(0,0,363),"mandrel");
    var minX=Math.max(25,mandrel.parameterNumber("outside")/2+hob.design.outsideDiameter/2+0.2);
    var cutDistance=this.spec.pitchRadius()+hob.design.pitchDiameter/2+this.spec.profileShift*this.spec.module;
    // Construction frames are at X=0; the joint coordinate supplies radial infeed once.
    var centre=0.0;
    if(cutDistance<minX||cutDistance>100||this.spec.faceWidth>20)throw "gear/fixture exceeds hobber capacity";
    travel=[{id:"X",lower:minX,upper:HobberXLayout.RETRACT,initial:cutDistance},{id:"Y",lower:-10,upper:10,initial:0},{id:"Z",lower:-10,upper:190,initial:0}];
    for(side in [-1,1]){
      var name=side<0?"Left":"Right",insert=side<0?"bed/railLeft":"bed/railRight";
      fixAt("railX"+name,new HiwinHgr25Rail(HobberXLayout.front().railLength),AxisBuilder.orient(HobberXLayout.front().railStart,side*FoundationLayout.BED_RAIL_CENTRE,242,[0.0,0,1],[1.0,0,0]),insert,F.alongY(HobberXLayout.front().railStart,side*FoundationLayout.BED_RAIL_CENTRE,220,0,0,1));
      precisionMounts.push({name:"railX"+name+"-mount",parent:insert});
    }
    addComponent("Xbridge",new HobberSaddle("GE-X-carriage"));zero.set("Xbridge",F.translation(centre,0,0));
    moving("X",Prismatic,"railXLeft","Xbridge",F.translation(0,0,242),[1.0,0,0],cutDistance,minX,HobberXLayout.RETRACT);
    for(side in [-1,1])for(row in [HobberXLayout.BLOCK_FRONT,HobberXLayout.BLOCK_REAR])fixAt("blockX"+(side<0?"Left":"Right")+(row==HobberXLayout.BLOCK_FRONT?"":"Front"),new SupplierRailBlock("HGR25"),
      AxisBuilder.orient(centre+row,side*FoundationLayout.BED_RAIL_CENTRE,242,[0.0,0,1],[1.0,0,0]),"Xbridge",F.alongY(centre+row,side*FoundationLayout.BED_RAIL_CENTRE,260,0,0,1));
    includeAt("column",column,F.fromRotationMatrix(centre+286,0,290,[0.0,-1,0,1,0,0,0,0,1]));
    fixExisting("column-mount","Xbridge","column/basePlate",F.alongY(centre+326,0,280,0,0,1));
    for(side in [-1,1]){
      var name=side<0?"Left":"Right",insert=side<0?"column/railLeft":"column/railRight";
      fixAt("railZ"+name,LinearRail.metric("HGR15",480),AxisBuilder.orient(centre+161,side*145,295,[-1.0,0,0],[0.0,0,1]),insert,F.alongY(centre+176,side*145,295,-1,0,0));
      precisionMounts.push({name:"railZ"+name+"-mount",parent:insert});
    }
    addComponent("Zplate",new HobberSaddle("GE-Z-saddle"));zero.set("Zplate",F.translation(centre,0,363));
    moving("Z",Prismatic,"railZLeft","Zplate",F.translation(centre+161,0,363),[0.0,0,1],0,-10,190);
    for(side in [-1,1])for(row in [0,110])fixAt("blockZ"+(side<0?"Left":"Right")+(row==0?"":"Upper"),new SupplierRailBlock("HGR15"),
      AxisBuilder.orient(centre+161,side*145,363+row,[-1.0,0,0],[0.0,0,1]),"Zplate",F.alongY(centre+148,side*145,363+row,-1,0,0));
    var swivelMotor=new TorqueMotor(new ComponentValues().setNumber("height",20).setNumber("ratedSpeed",Math.PI/12).setNumber("maxSpeed",Math.PI/6)
      .setNumber("torqueConstant",2).setNumber("backEmfConstant",2/Math.sqrt(3)));
    swivel=new SwivelHead(swivelMotor,null,Math.PI/12);
    includeAt("swivel",swivel,F.fromRotationMatrix(centre+135,0,363,[0.0,0,-1,1,0,0,0,-1,0]));
    fixExisting("swivel-mount","Zplate","swivel/housing",F.alongY(centre+135,0,363,1,0,0));
    var e0=[-1.0,0,0,0,0,-1,0,-1,0];
    fixAt("swivelPlate",new HobberSaddle("GE-swivel-plate"),F.translation(centre,0,363),"swivel/head",F.alongY(centre+100,0,363,-1,0,0));
    for(side in [-1,1])fixAt(side<0?"railYLeft":"railYRight",LinearRail.metric("HGR15",220),
      AxisBuilder.orient(centre+70,110,363+side*60,[-1.0,0,0],[0.0,-1,0]),"swivelPlate",F.alongY(centre+85,110,363+side*60,-1,0,0));
    addComponent("Yplate",new HobberSaddle("GE-Y-saddle"));zero.set("Yplate",F.translation(centre,0,363));
    moving("Y",Prismatic,"railYLeft","Yplate",F.translation(centre+70,0,363),[0.0,-1,0],0,-10,10);
    for(side in [-1,1])for(row in [-40,40])fixAt("blockY"+(side<0?"Left":"Right")+(row<0?"":"Front"),new SupplierRailBlock("HGR15"),
      AxisBuilder.orient(centre+70,row,363+side*60,[-1.0,0,0],[0.0,-1,0]),"Yplate",F.alongY(centre+57,row,363+side*60,-1,0,0));
    var rated=4000*2*Math.PI/60;
    var spindleMotor=new TorqueMotor(new ComponentValues().setNumber("outside",62).setNumber("bore",25).setNumber("height",100)
      .setNumber("continuousTorque",1500/rated).setNumber("peakTorque",3000/rated).setNumber("ratedSpeed",rated).setNumber("maxSpeed",8000*2*Math.PI/60)
      .setNumber("rotorInertia",0.0002).setNumber("coggingTorque",0.003).setInteger("poleCount",4).setInteger("coggingOrder",12)
      .setNumber("torqueConstant",0.4).setNumber("backEmfConstant",0.4/Math.sqrt(3)).setNumber("phaseResistance",0.25));
    spindle=new HobSpindle(spindleMotor,new HobArbor(new ComponentValues().setNumber("height",175).setNumber("hobCentreOffset",60).setNumber("supportDiameter",15).setNumber("supportLength",22)));
    includeAt("spindle",spindle,F.fromRotationMatrix(centre,240,363,e0));fixExisting("spindle-mount","Yplate","spindle/housing",F.alongY(centre+45,210,363,1,0,0));
    fixAt("hob",hob,F.fromRotationMatrix(centre,20,363,e0),"spindle/arbor");
    fixAt("outboard",new OutboardSupport(new ComponentValues().setNumber("height",30).setNumber("seatLength",9)),F.fromRotationMatrix(centre,-85,363,e0),"Yplate",F.alongY(centre+30,-100,363,1,0,0));
    fixAt("outboardBearing",new FloatingRadialBearing(),F.fromRotationMatrix(centre,-95.5,363,e0),"outboard");
    mountContacts.push({parent:"outboard",child:"outboardBearing"});mountContacts.push({parent:"spindle/arbor",child:"outboardBearing"});
    var supply=new ThreePhaseRectifier(new AcMains(ThreePhase,new AcRmsVoltage(400),50),new DcCurrent(40),6,240,120,100,15000);
    fixAt("electricalPanel",new HobberSaddle("electrical-panel"),HobberElectricalLayout.panelPose(),"bed/liftLeft");
    fixAt("powerSupply",supply,HobberElectricalLayout.devicePose(0,130),"electricalPanel");exposePort("mains","powerSupply","mains");
    fixAt("XdriveFrame",new HobberSaddle("GE-X-drive"),F.translation(centre,0,0),"bed/driveSeat",F.alongY((HobberXLayout.front().motorFace+HobberXLayout.front().deckStart)/2,0,140,0,0,1));
    precisionMounts.push({name:"XdriveFrame-mount",parent:"bed/driveSeat"});
    fixAt("ZdriveFrame",new HobberSaddle("GE-Z-drive"),F.translation(centre,0,0),"column/driveSeat",F.alongY(centre+HobberZLayout.SEAT_FACE_X,0,540,-1,0,0));
    precisionMounts.push({name:"ZdriveFrame-mount",parent:"column/driveSeat"});
    fixAt("YdriveFrame",new HobberSaddle("GE-Y-drive"),F.translation(centre,0,363),"swivelPlate",F.alongY(centre+85,-105,458,-1,0,0));
    drive("X","XdriveFrame","Xbridge",[-1.0,0,0],F.translation(centre+HobberXLayout.NUT_DATUM,0,180),HobberXLayout.front().screwLength,HobberXLayout.front().nutStation);
    drive("Z","ZdriveFrame","Zplate",[0.0,0,-1],F.translation(centre+HobberZLayout.AXIS_X,0,363+HobberZLayout.NUT_OFFSET_Z),HobberZLayout.SHAFT_LENGTH,HobberZLayout.NUT_STATION);
    drive("Y","YdriveFrame","Yplate",[0.0,-1,0],F.translation(centre+70,0,508),250);
    fixAt("chipTray",new HobberSaddle("GE-chip-tray",new ComponentValues().setNumber("tableFaceDiameter",tableSpec.number("faceDiameter")).setNumber("tableHousingDiameter",tableSpec.number("housingDiameter")).setNumber("tableFaceHeight",FoundationLayout.TABLE_FACE).setNumber("tableSeatHeight",tableBase)),F.identity(),"bed/tablePlate");
    fixAt("coolantPort",new HobberSaddle("GE-coolant"),F.translation(centre,0,363),"Yplate",F.alongY(centre+45,40,435,1,0,0));
    // Enclosure envelope reserved; panels omitted for the current open inspection layout.
    bind("table-drive",C,"table/"+tableSpec.drivePart);bind("swivel-drive",A,"swivel/motor");bind("hob-drive",B,"spindle/motor",true);
    // Feedback belongs to the outer drives; the cartridge encoders are also exposed as sensors.
    addEncoder("table-feedback",C,"table/"+tableSpec.encoderPart,"table-drive");
    addEncoder("hob-master",B,"spindle/encoder","hob-drive");
    bindMotorBrake("swivel-drive","swivel/clamp","swivel.clampRelease","swivel.clampReleased",1000,30);
    FoundationIntegrity.check(bed).throwIfErrors();FoundationIntegrity.check(column).throwIfErrors();FoundationIntegrity.checkMounts(this,precisionMounts).throwIfErrors();
    exposeConnector("work","mandrel","work");exposeConnector("hobCentre","hob","centre");
  }
  function includeAt(id:String,a:MachineAssembly,pose:AssemblyFrame):Void {
    include(id,a,F.compose(MACHINE_FROM_WORK,pose));var positions=a.solvedPoses();for(m in a.components()){var p=positions.get(m.id);if(p==null)throw "missing package pose";zero.set(id+"/"+m.id,F.compose(pose,p));}
  }
  function fixed(id:String,parent:String,pc:String,child:String,cc:String):Void addMate(id,Fixed,parent,pc,child,cc);
  function fixAt(id:String,p:MachineComponent,pose:AssemblyFrame,parent:String,?seat:AssemblyFrame):Void {addComponent(id,p);zero.set(id,pose);fixExisting(id+"-mount",parent,id,seat);}
  function fixExisting(id:String,parent:String,child:String,?seat:AssemblyFrame):Void {
    var p=required(parent),c=required(child),frame=seat==null?F.alongY(c.x,c.y,c.z,0,0,1):seat;
    if(seat!=null)mountContacts.push({parent:parent,child:child});
    addMemberConnector(parent,id+"Parent",F.compose(F.inverse(p),frame));addMemberConnector(child,id+"Child",F.compose(F.inverse(c),frame));
    addMate(id,Fixed,parent,id+"Parent",child,id+"Child");
  }
  function required(id:String):AssemblyFrame {var p=zero.get(id);if(p==null)throw 'missing zero pose $id';return p;}
  function moving(id:String,type:AssemblyJointType,parent:String,child:String,origin:AssemblyFrame,d:Array<Float>,initial:Float,lower:Float,upper:Float):Void {
    var frame=F.alongY(origin.x,origin.y,origin.z,d[0],d[1],d[2]);
    addMemberConnector(parent,id+"Parent",F.compose(F.inverse(required(parent)),frame));
    addMemberConnector(child,id+"Child",F.compose(F.inverse(required(child)),frame));
    addMateOnAxis(id,type,parent,id+"Parent",child,id+"Child",{x:0,y:1,z:0},initial,{lower:lower,upper:upper,velocity:null,effort:null,acceleration:500,overtravel:5});
  }
  function drive(id:String,parent:String,carriage:String,d:Array<Float>,nutOrigin:AssemblyFrame,length:Float,nutStation:Float=100):Void {
    var motor=ServoMotor.model(id=="Z"?"GENERIC-HV-SERVO-750W":"GENERIC-HV-SERVO-400W"),screw=new BallScrew(length,id=="Z"?60:BallScrew.INPUT_JOURNAL_LENGTH,true),nut=new BallNut();
    var start=[nutOrigin.x-d[0]*nutStation,nutOrigin.y-d[1]*nutStation,nutOrigin.z-d[2]*nutStation];
    var up=id=="Z"?[-1.0,0,0]:id=="Y"?[1.0,0,0]:[0.0,0,1];
    var mp=AxisBuilder.orient(start[0]-d[0]*motor.rating.shaftLength,start[1]-d[1]*motor.rating.shaftLength,start[2]-d[2]*motor.rating.shaftLength,up,d);
    fixAt(id+"motor",motor,mp,parent,F.compose(mp,Solids.axial(0,0,0)));
    addComponent(id+"screw",screw);zero.set(id+"screw",AxisBuilder.orient(start[0],start[1],start[2],up,d));
    var f=F.alongY(start[0],start[1],start[2],d[0],d[1],d[2]);
    addMemberConnector(id+"motor",id+"turnParent",F.compose(F.inverse(required(id+"motor")),f));
    addMemberConnector(id+"screw",id+"turnChild",F.compose(F.inverse(required(id+"screw")),f));
    fixAt(id+"nut",nut,AxisBuilder.orient(nutOrigin.x-d[0]*nut.bodyLength/2,nutOrigin.y-d[1]*nut.bodyLength/2,nutOrigin.z-d[2]*nut.bodyLength/2,up,d),carriage,F.alongY(nutOrigin.x+d[0]*27.5,nutOrigin.y+d[1]*27.5,nutOrigin.z+d[2]*27.5,d[0],d[1],d[2]));
    var ratio=addTransmission(id+"lead",id,id+"screw-turn",Transmission.LeadScrew(id+"screw",id+"nut"),id=="X"?Same:Opposite);
    var initial=travel.filter(t->t.id==id)[0].initial;
    addMate(id+"screw-turn",Continuous,id+"motor",id+"turnParent",id+"screw",id+"turnChild",ratio*initial);
    var near=ScrewSupportUnit.bk12(),far=ScrewSupportUnit.bf12(),nearOffset=id=="Z"?47.5:37.5;
    fixAt(id+"fixed",near,AxisBuilder.orient(start[0]+d[0]*nearOffset,start[1]+d[1]*nearOffset,start[2]+d[2]*nearOffset,up,d),parent);
    fixAt(id+"floating",far,AxisBuilder.orient(start[0]+d[0]*(length-12.5),start[1]+d[1]*(length-12.5),start[2]+d[2]*(length-12.5),up,d),parent);
    supportScrew(id+"lead",near.support,far.support);
    var coupling=new ShaftCoupling(motor.rating.shaftDiameter,12);
    if(motor.rating.shaftLength<coupling.length/2)throw "motor shaft cannot engage half coupling";
    fixAt(id+"coupling",coupling,AxisBuilder.orient(start[0]-d[0]*coupling.length/2,start[1]-d[1]*coupling.length/2,start[2]-d[2]*coupling.length/2,up,d),id+"screw");
    fixAt(id+"motor-shaft",motor.shaft(),mp,id+"screw");
    var scaleSeat=switch id {
      case "X": AxisBuilder.orient(start[0],46,175,[0.0,1,0],d);
      case "Z": AxisBuilder.orient(start[0]+45,HobberZLayout.SCALE_Y,start[2],[-1.0,0,0],d);
      case "Y": AxisBuilder.orient(nutOrigin.x-24,start[1],548,[-1.0,0,0],d);
      default: throw "missing scale mounting datum";
    };
    fixAt(id+"scale",new LinearScale(length,10000,true),scaleSeat,parent);
    mountContacts.push({parent:parent,child:id+"fixed"});mountContacts.push({parent:parent,child:id+"floating"});mountContacts.push({parent:parent,child:id+"scale"});
    addEncoder(id+"position",id,id+"scale");bind(id+"drive",id+"screw-turn",id+"motor");
  }
  function bind(id:String,joint:String,motor:String,process:Bool=false):Void {
    driverNumber++;var driver=id+"-driver";
    fixAt(driver,new MotorDriver("GENERIC-400V-SERVO-AMP",new PhaseRmsCurrent(id=="table-drive"?tableSpec.number("peakCurrent"):20)),
      HobberElectricalLayout.devicePose(-140+((driverNumber-1)%3)*140,300+Std.int((driverNumber-1)/3)*250),"electricalPanel");
    connectPorts(id+"-power","powerSupply","power"+driverNumber,driver,"power");
    addMotor(id,joint,motor,driver,1,null,process?{speedChannel:"hob.speed",directionChannel:"hob.direction",radiansPerSpeedUnit:2*Math.PI/60}:null);
  }
  public function hobToWork(state:AssemblyState):AssemblyFrame {
    var work=F.compose(state.worldPose("mandrel"),F.translation(0,0,mandrel.parameterNumber("workSeat")));
    var cutter=F.compose(state.worldPose("hob"),F.translation(0,0,hob.parameterNumber("height")/2));
    return F.compose(F.inverse(work),cutter);
  }
}
