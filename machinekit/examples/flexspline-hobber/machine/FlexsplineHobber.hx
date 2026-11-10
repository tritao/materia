import hobber.*;
import cadkit.modeling.AssemblyState;
import gearkit.*;
import machinekit.assembly.*;
import machinekit.assembly.AxisBuilder;
import machinekit.component.*;
import machinekit.control.BissMaster;
import machinekit.design.DesignRequirement;
import machinekit.motion.*;
import machinekit.hobbing.*;
import machinekit.power.*;
import materia.assembly.AssemblyFrames as F;
import materia.assembly.AssemblyRecord.AssemblyFrame;
import materia.assembly.AssemblyDefinition.AssemblyJointType;

typedef HobberTravel = {id:String, lower:Float, upper:Float, initial:Float};

/** Six independent physical axes. There is deliberately no B/C mechanical gear coupling.
    CAD and prismatic coordinates mm; rotary coordinates rad. Nested joint IDs are explicit. */
class FlexsplineHobber extends MachineAssembly {
  public final zBellows:Array<machinekit.cover.WayCover>;
  public final zBellowsMounts:Array<String>;
  /** Collapsed sleeve to cup bore, diametral. The H5 bore (58.050/+0.013) against a sleeve OD of 58.040 -0.005/0 gives
      0.010-0.028 mm, mean 0.019: enough lead-in for the robot and for the oil to close in the 0.5-1.5 MPa contact window
      (workholding/loads/README 2). The previous 0.050 mm could not hold the cup at 10 MPa oil. */
  public static final LOADING_CLEARANCE = 0.02;
  /** Pressed length beyond the tooth band, from the open edge upward (workholding/fem/README case 1). */
  public static final PRESSED_MARGIN = 8.0;
  public static final C = "table/C";
  public static final B = "spindle/B";
  /** The servo's own joint: the belt couples it to B, and the hob drive is bound on it. */
  public static final B_MOTOR = "spindle/motor-turn";
  public static final A = "swivel/A";
  public static final Y = HobberLayout.Y_JOINT;
  /**
   * ASSUMED holding torque of the A brake, N m at the output flange. ZeroErr has not published it (asked 2026-11-03); this is the
   * actuator's rated output torque (87 N m, ratio 100), a placeholder for a spring-applied holding brake of this class. Replace it from
   * the maker's answer; `HobberFrameChecks.swivelLoad` reports the demand against it.
   */
  public static final A_BRAKE_HOLDING = 87.0;
  /** ASSUMED acceleration limit of A, rad/s^2: the old torque-motor head used the same; not a vendor value. */
  public static final A_ACCELERATION:Float = Math.PI / 2;
  public final spec:GearSpec;
  public final hob:Hob;
  public final mandrel:ExpandingMandrel;
  public final bed:MineralCastBed;
  public final column:MineralCastColumn;
  public final table:machinekit.assembly.MachineAssembly;
  public final tableSpec:machinekit.motion.RotaryAxisSpec;
  public final spindle:HobSpindle;
  public final swivel:ErobActuator;
  public final yModule:ScrewLinearModule;
  public final travel:Array<HobberTravel>;
  public final workOrigin:AssemblyFrame;
  public final precisionMounts:Array<{name:String, parent:String}> = [];
  public final mountContacts:Array<{parent:String, child:String}> = [];
  public final units:Array<HobberUnit>;
  final zUnit:HobberZAxis;
  public function new(?spec:GearSpec,gashes:Int=12,detailedHob:Bool=false,?rotaryAxis:PrecisionRotaryAxis) {
    super();
    HobberComponents.install();
    var layout=HobberLayout.get();
    this.spec=spec==null?new GearSpec(0.3,200,8,new Involute(20*Math.PI/180)):spec;
    if(this.spec.internal)throw "external hobbing machine requires external gear";
    var foundation=new HobberFoundation();
    bed=foundation.bed; column=foundation.column;
    var work=new HobberTableWork(this.spec,rotaryAxis);
    table=work.table; tableSpec=work.tableSpec; mandrel=work.mandrel; workOrigin=work.workOrigin;
    var cutter=new HobberBSpindle(this.spec,gashes,detailedHob);
    spindle=cutter.spindle; hob=cutter.hob;
    var minX=Math.max(25,mandrel.parameterNumber("outside")/2+hob.design.outsideDiameter/2+0.2);
    var cutDistance=this.spec.pitchRadius()+hob.design.pitchDiameter/2+this.spec.profileShift*this.spec.module;
    if(cutDistance<minX||cutDistance>100||this.spec.faceWidth>20)throw "gear/fixture exceeds hobber capacity";
    travel=[{id:"X",lower:minX,upper:layout.xRetract,initial:cutDistance},
      {id:"Y",lower:-15,upper:15,initial:0},{id:"Z",lower:layout.zTravelMin,upper:layout.zTravelMax,initial:0}];
    var xUnit=new HobberXAxis(cutDistance,minX);
    zUnit=new HobberZAxis(); zBellows=zUnit.zBellows; zBellowsMounts=zUnit.zBellowsMounts;
    var head=new HobberAHead(); swivel=head.swivel;
    var yUnit=new HobberYStage(); yModule=yUnit.yModule;
    var rack=new HobberElectricalRack(tableSpec);
    units=[foundation,work,xUnit,zUnit,head,yUnit,cutter,rack];
    var owners=[for(unit in units) unit.key=>unit];
    for(unit in units) for(mount in unit.mounts) {
      var owner=owners.get(mount.owner);
      if(owner==null)throw "Missing hobber unit: "+mount.owner;
      owner.offer(mount);
    }
    var groups=["Bed and column","C table and work","X axis","Z axis","A swivel","Y axis","B spindle","Electrical"];
    for(i in 0...units.length) includeAsGroup(groups[i],units[i].key,units[i],null,true);
    for(unit in units) {
      for(contact in unit.mountContacts) mountContacts.push(contact);
      for(mount in unit.mounts) {
        var parent=connector(mount.owner+"/"+mount.id),child=connector(mount.child);
        addMate(mount.id,Fixed,parent.instanceId,parent.connectorName,child.instanceId,child.connectorName);
        if(mount.contact)mountContacts.push({parent:parent.instanceId,child:child.instanceId});
        if(mount.precision)precisionMounts.push({name:mount.id,parent:parent.instanceId});
      }
    }
    for(name in work.portNames()) { var port=work.port(name); exposePort(name,port.instanceId,port.portName); }
    for(name in rack.portNames()) { var port=rack.port(name); exposePort(name,port.instanceId,port.portName); }
    HobberControls.install(this);
    machinekit.structural.FoundationIntegrity.check(bed).throwIfErrors();
    machinekit.structural.FoundationIntegrity.check(column).throwIfErrors();
    machinekit.structural.FoundationIntegrity.checkMounts(this,precisionMounts,[for(mount in precisionMounts)mount.name=>mount.name]).throwIfErrors();
    deriveFasteners();
    checkJointLoads(HobberJointLoads.of(this));
    addBomItem(machinekit.cover.WayWiperCatalog.bomItem("WIPER-HG25-SCRAPER",8));
    addBomItem(machinekit.cover.WayWiperCatalog.bomItem("WIPER-HG15-SCRAPER",16));
    for(r in serviceRemovals())addDesignRequirement(r);
  }
  public function serviceRemovals():Array<DesignRequirement> return new HobberServiceAccess(this).serviceRemovals();
  public function processContacts():Array<{a:String,b:String}> return [{a:"blank",b:"hob"},{a:"mandrel",b:"hob"},{a:"mandrelSleeve",b:"hob"}]
    .concat([for(pair in yModule.stopContacts()){a:"Ymodule/"+pair.a,b:"Ymodule/"+pair.b}]).concat(zUnit.internalContacts(this));
  public function hobToWork(state:AssemblyState):AssemblyFrame {
    var work = F.compose(state.worldPose("mandrel"), F.translation(0, 0, mandrel.parameterNumber("workSeat")));
    var cutter = F.compose(state.worldPose("hob"), F.translation(0, 0, hob.parameterNumber("height") / 2));
    return F.compose(F.inverse(work), cutter);
  }
}
