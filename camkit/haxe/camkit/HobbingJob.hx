package camkit;

import gearkit.GearSpec;
import gearkit.HobDesign;
import toolpathkit.path.*;
import toolpathkit.setup.Setup;
import toolpathkit.tool.Tool;
import toolpathkit.tool.ToolLibrary;

/** External spur-gear CAM for a right-hand hob. No machine solids, native CAD or stock subtraction.
    Gear/hob geometry is mm; exported machine-axis coordinates and feeds are metres and m/s.
    The setup is the logical X/Y/Z hobber axes, not a tool-tip Cartesian frame. */
class HobbingJob {
  public final spec:GearSpec;
  public final hob:HobDesign;
  public final usableHobLength:Float;
  public final rpm:Float;
  public final tableRpm:Float;
  public final axialFeed:Float; // mm/s
  public final fullDepth:Float;
  public final safeX:Float;
  public final passes:Array<HobbingPass> = [];
  public final program:ToolpathProgram;
  /** Nominal cutting-feed time, including geometric overrun and axial clearance on each pass.
      Positioning, acceleration, spindle ramps and measured barriers are additional. */
  public final cuttingSeconds:Float;
  final settings:HobbingSettings;
  final faceTop:Float;
  public function new(spec:GearSpec,hob:HobDesign,usableHobLength:Float,?settings:HobbingSettings,?limits:HobbingLimits) {
    if(spec==null||hob==null)throw "Hobbing needs a gear and hob";
    if(spec.internal)throw "HobbingJob supports external spur gears";
    if(Math.abs(spec.module-hob.rack.module)>1e-12||Math.abs(spec.pressureAngle-hob.rack.pressureAngle)>1e-12)
      throw "Hob and target gear need matching normal module and pressure angle";
    this.settings=settings==null?new HobbingSettings():settings.copy();
    var s=this.settings;
    for(v in [usableHobLength,s.cuttingSpeed,s.feedPerRevolution,s.clearance,s.crownChordTolerance,s.rapidFeed,s.infeed,s.signalTimeout])
      if(!Math.isFinite(v)||v<=0)throw "Hobbing lengths, feeds, speed and timeout must be positive and finite";
    if(!Math.isFinite(s.crown)||s.crown<0||!Math.isFinite(s.faceBottom))throw "Invalid hobbing crown or face datum";
    for(id in [s.xAxis,s.yAxis,s.zAxis,s.tiltJoint,s.hobJoint,s.tableJoint,s.readySignal,s.stoppedSignal])
      if(id==null||id.length==0)throw "Hobbing needs named axes and measured signals";
    var ids=[s.xAxis,s.yAxis,s.zAxis,s.tiltJoint,s.hobJoint,s.tableJoint];
    for(i in 0...ids.length)for(j in 0...i)if(ids[i]==ids[j])throw "Hobbing axes must be distinct";
    this.spec=spec;this.hob=hob;this.usableHobLength=usableHobLength;
    faceTop=s.faceBottom+spec.faceWidth;
    rpm=s.cuttingSpeed*1000/(Math.PI*hob.outsideDiameter);
    tableRpm=rpm*hob.starts/spec.teeth;
    axialFeed=s.feedPerRevolution*tableRpm/60;
    var blank=s.blankOuterRadius==null?spec.tipRadius():s.blankOuterRadius;
    if(!Math.isFinite(blank)||blank<spec.tipRadius())throw "Blank outer radius must cover the target gear";
    var finish=spec.pitchRadius()+hob.pitchDiameter/2+spec.profileShift*spec.module;
    fullDepth=blank+hob.outsideDiameter/2-finish;
    if(fullDepth<=0||s.crown>=fullDepth)throw "Hobbing crown must leave positive engagement";
    safeX=blank+hob.outsideDiameter/2+s.clearance;
    var depths=s.depths.length==0?[fullDepth]:s.depths.copy();
    if(depths.length>10000)throw "Too many hobbing passes";
    var previous=0.0;
    for(depth in depths){if(!Math.isFinite(depth)||depth<=previous||depth>fullDepth+1e-9)throw "Radial pass depths must increase to full tooth depth";previous=depth;}
    if(Math.abs(depths[depths.length-1]-fullDepth)>1e-9)throw "Last hobbing pass must reach full tooth depth";
    if(s.shifts.length!=0&&s.shifts.length!=depths.length)throw "Hob shift schedule needs one shift per pass";
    var nominalPhase=(Math.PI/spec.teeth-Math.PI/2)%(2*Math.PI/spec.teeth);
    if(nominalPhase<0)nominalPhase+=2*Math.PI/spec.teeth;
    var time=0.0;
    for(i in 0...depths.length){
      var shift=s.shifts.length==0?0.0:s.shifts[i];if(!Math.isFinite(shift))throw "Hob shifts must be finite";
      var distance=blank+hob.outsideDiameter/2-depths[i];
      var engagement=new HobbingEngagement(blank,hob.outsideDiameter/2,distance,-hob.leadAngle);
      if(Math.abs(shift)+engagement.axialHalfSpan>usableHobLength/2-1e-8)
        throw 'Hob too short for shift $shift mm on pass ${i+1}: needs half-length ${Math.abs(shift)+engagement.axialHalfSpan} mm';
      var bottom=s.faceBottom-engagement.overrun-s.clearance,top=faceTop+engagement.overrun+s.clearance;
      var start=s.climb?bottom:top,end=s.climb?top:bottom;
      var turns=(top-bottom)/s.feedPerRevolution,seconds=turns/tableRpm*60;
      passes.push(new HobbingPass(depths[i],shift,distance,engagement,start,end,turns,seconds,
        nominalPhase-shift*Math.cos(hob.leadAngle)/spec.pitchRadius()));time+=seconds;
    }
    cuttingSeconds=time;
    for(v in [rpm,tableRpm,axialFeed,safeX,cuttingSeconds])if(!Math.isFinite(v)||v<=0)throw "Hobbing derived speed/time exceeds finite range";
    program=emit();
    if(limits!=null)limits.check(this);
  }
  /** Programmed centre-thickness crown (mm), clamped to zero outside the blank face. */
  public function crownAt(z:Float):Float {
    if(z<=settings.faceBottom||z>=faceTop)return 0;
    var u=2*(z-settings.faceBottom)/spec.faceWidth-1;
    return settings.crown*(1-u*u);
  }
  function emit():ToolpathProgram {
    var s=settings,ops:Array<ToolpathOp> = [],span=Provenance.cam(0,"hobbing setup");
    ops.push(WaitSignal(s.readySignal,true,s.signalTimeout,span));
    ops.push(SetSetup("hobber-axes",span));
    // The startup witness includes a referenced B=0 and released gear. Clear radially before tilt/phase changes.
    ops.push(PositionJoint(s.xAxis,safeX/1000,span));
    ops.push(PositionJoint(s.tiltJoint,-hob.leadAngle,span));
    ops.push(PositionJoint(s.tableJoint,passes[0].tablePhase,span));
    ops.push(ElectronicGear(s.tableJoint,[new GearTerm(s.hobJoint,hob.starts/spec.teeth)],span));
    ops.push(Spindle(Clockwise,rpm,span));
    var current=new Point3(safeX/1000,passes[0].shift/1000,passes[0].zStart/1000);
    // Separate radial withdrawal from Y/Z repositioning on every pass.
    for(i in 0...passes.length){
      var pass=passes[i];span=Provenance.cam(i+1,'hob pass ${i+1}');
      ops.push(FeedPerRevolution(s.tableJoint,s.zAxis,0,span));
      ops.push(PositionJoint(s.xAxis,safeX/1000,span));
      ops.push(PositionJoint(s.yAxis,pass.shift/1000,span));
      ops.push(PositionJoint(s.zAxis,pass.zStart/1000,span));
      ops.push(PositionJoint(s.tableJoint,pass.tablePhase,span));
      current=new Point3(safeX/1000,pass.shift/1000,pass.zStart/1000);
      var entry=new Point3(pass.centreDistance/1000,current.y,current.z);
      ops.push(Move(Cut,Line(current,entry),s.infeed/1000,0,span));current=entry;
      ops.push(FeedPerRevolution(s.tableJoint,s.zAxis,s.feedPerRevolution/1000,span));
      var stations=[pass.zStart];
      if(s.crown>0){
        var segments=Std.int(Math.max(2,Math.ceil(Math.sqrt(2*s.crown/s.crownChordTolerance))));
        if(segments>10000)throw "Crown chord tolerance needs too many segments";
        for(j in 0...segments+1)stations.push(s.faceBottom+spec.faceWidth*j/segments);
      }
      stations.push(pass.zEnd);stations.sort(Reflect.compare);if(!s.climb)stations.reverse();
      for(z in stations){
        if(Math.abs(z-current.z*1000)<1e-10)continue;
        var next=new Point3((pass.centreDistance+crownAt(z))/1000,pass.shift/1000,z/1000);
        var speed=axialFeed/1000*next.distanceTo(current)/Math.abs(next.z-current.z);
        ops.push(Move(Cut,Line(current,next),speed,s.crown>0?s.crownChordTolerance/2000:0,span));current=next;
      }
      ops.push(FeedPerRevolution(s.tableJoint,s.zAxis,0,span));
      var retract=new Point3(safeX/1000,current.y,current.z);
      ops.push(Move(Retract,Line(current,retract),s.rapidFeed/1000,0,span));current=retract;
    }
    ops.push(Spindle(Off,0,span));ops.push(WaitSignal(s.stoppedSignal,true,s.signalTimeout,span));
    // Keep the stopped spindle/table relation: release without rebasing additive C would lose phase.
    ops.push(End(span));
    var tools=new ToolLibrary();tools.set(new Tool(1,usableHobLength/1000,hob.outsideDiameter/1000));
    return new ToolpathProgram(ops,tools,
      [new Setup("hobber-axes",new Point3(0,0,0))]);
  }
}
