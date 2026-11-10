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

typedef HobberMount = {
  id:String, owner:String, role:String, child:String, frame:AssemblyFrame,
  contact:Bool, precision:Bool, nativeParent:Bool
};

/** One machine-frame unit with named mating surfaces and service ports.
 * Each unit owns its parts; the containing machine joins only these interfaces. */
class HobberUnit extends MachineAssembly {
  public final key:String;
  public final layout:HobberLayout;
  public final heights:machinekit.hobbing.HobberLayout.HobberHeightStack;
  public final mounts:Array<HobberMount> = [];
  public final mountContacts:Array<{parent:String, child:String}> = [];
  final hosts:Map<String, {member:String, connector:Null<String>}> = [];

  public function new(key:String) {
    super();
    this.key = key;
    layout = HobberLayout.get();
    heights = layout.heights();
  }
  public function host(role:String, member:String, ?connector:String):Void hosts.set(role, {member:member, connector:connector});
  public function publish(name:String, member:String, frame:AssemblyFrame):Void {
    var connector = "interface-" + name.split("/").join("-");
    addMemberConnector(member, connector, F.compose(F.inverse(constructionPose(member)), frame));
    exposeConnector(name, member, connector);
  }
  /** Publish the receiving surface requested by another unit's interface. */
  public function offer(mount:HobberMount):String {
    var destination = hosts.get(mount.role);
    if (destination == null) throw 'Unit "$key" has no interface role "${mount.role}"';
    var name = key + "/" + mount.id;
    if (mount.nativeParent && destination.connector != null)
      exposeConnector(name, destination.member, destination.connector);
    else publish(name, destination.member, mount.frame);
    return name;
  }
  public function externalPart(id:String, part:MachineComponent, pose:AssemblyFrame, owner:String, role:String,
    ?seat:AssemblyFrame, precision:Bool = false):Void {
    addComponent(id, part, pose);
    externalMate(id + "-mount", id, owner, role,
      seat == null ? F.alongY(pose.x, pose.y, pose.z, 0, 0, 1) : seat, seat != null, precision);
  }
  public function externalMate(id:String, member:String, owner:String, role:String, frame:AssemblyFrame,
    contact:Bool = true, precision:Bool = false):Void {
    var name = key + "/" + id;
    publish(name, member, frame);
    mounts.push({id:id, owner:owner, role:role, child:name, frame:frame,
      contact:contact, precision:precision, nativeParent:false});
  }
  public function externalNativeMate(id:String, member:String, connector:String, owner:String, role:String,
    precision:Bool = false):Void {
    var name = key + "/" + id;
    exposeConnector(name, member, connector);
    mounts.push({id:id, owner:owner, role:role, child:name,
      frame:F.compose(constructionPose(member), memberConnectorFrame(member, connector)),
      contact:false, precision:precision, nativeParent:true});
  }
  public function fixed(id:String, parent:String, pc:String, child:String, cc:String):Void addMate(
    id,
    Fixed,
    parent,
    pc,
    child,
    cc
  );
  public function fixedPart(id:String, p:MachineComponent, pose:AssemblyFrame, parent:String, ?seat:AssemblyFrame):Void {
    addComponent(id, p, pose);
    fixedMembers(id + "-mount", parent, id, seat);
  }
  public function fixedMembers(id:String, parent:String, child:String, ?seat:AssemblyFrame):Void {
    var p = constructionPose(parent), c = constructionPose(child), frame = seat == null ? F.alongY(c.x, c.y, c.z, 0, 0, 1) : seat;
    if (seat != null) mountContacts.push({parent: parent, child: child});
    addMemberConnector(parent, id + "Parent", F.compose(F.inverse(p), frame));
    addMemberConnector(child, id + "Child", F.compose(F.inverse(c), frame));
    addMate(id, Fixed, parent, id + "Parent", child, id + "Child");
  }
  public function moving(
    id:String,
    type:AssemblyJointType,
    parent:String,
    child:String,
    origin:AssemblyFrame,
    d:Array<Float>,
    initial:Float,
    lower:Float,
    upper:Float
  ):Void {
    var frame = F.alongY(origin.x, origin.y, origin.z, d[0], d[1], d[2]);
    addMemberConnector(parent, id + "Parent", F.compose(F.inverse(constructionPose(parent)), frame));
    addMemberConnector(child, id + "Child", F.compose(F.inverse(constructionPose(child)), frame));
    addMateOnAxis(
      id,
      type,
      parent,
      id + "Parent",
      child,
      id + "Child",
      {x: 0, y: 1, z: 0},
      initial,
      {lower: lower, upper: upper, velocity: null, effort: null, acceleration: 500, overtravel: 5}
    );
  }
  /**
   * One axis's drive from `HobberDrives`: the mount on its fixed pad, servo and coupling, the screw turning in them, the BF12
   * and its ring on the floating pad, and the nut on a nut block bolted to `blockHost`. The seat holes are cut into the pads.
   */
  public function drive(hd:HobberDriveAxis, fixedPad:String, floatingPad:String, blockHost:String, initial:Float):Void {
    group("Drive");
    var id = hd.id, dr = hd.drive, d = hd.axis.direction, s = hd.axis.start;
    var at = [for (name => pose in HobberDrives.poses(hd)) name => HobberFrame.sourcePose(pose)];
    var motor = dr.motor, screw = new HzMotionDrawingScrew(hd.length, dr.endProfile), nut = new BallNut();
    fixedPart(HobberDrives.member(id,"mount"), dr.fixed, at.get("mount"), fixedPad);
    fixedPart(HobberDrives.member(id,"motor"), motor, at.get("motor"), HobberDrives.member(id,"mount"));
    addComponent(HobberDrives.member(id,"screw"), screw, at.get("screw"));
    var f = F.alongY(-s[1], s[0], HobberFrame.PART_DATUM_Z + s[2], -d[1], d[0], d[2]);
    // The screw turns in the mount's bearings: the mount is the joint's parent (the motor is bolted to it).
    addMemberConnector(HobberDrives.member(id,"mount"), id + "turnParent", F.compose(F.inverse(constructionPose(HobberDrives.member(id,"mount"))), f));
    addMemberConnector(HobberDrives.member(id,"screw"), id + "turnChild", F.compose(F.inverse(constructionPose(HobberDrives.member(id,"screw"))), f));
    var block = HobberDrives.nutBlock(hd, nut), blockPose = HobberFrame.sourcePose(HobberDrives.nutBlockPose(hd, nut));
    fixedPart(HobberDrives.member(id,"nutCarrier"), block, blockPose, blockHost, F.compose(blockPose, F.translation(0, -block.centreHeight, block.thickness / 2)));
    fixedPart(HobberDrives.member(id,"nut"), nut, at.get("nut"), HobberDrives.member(id,"nutCarrier"), blockPose);
    var ratio = addTransmission(HobberDrives.member(id,"lead"), id, HobberDrives.member(id,"screw-turn"), Transmission.LeadScrew(HobberDrives.member(id,"screw"), HobberDrives.member(id,"nut")), id == "Z" ? Opposite : id == "X" ? Opposite : Same);
    addMate(HobberDrives.member(id,"screw-turn"), Continuous, HobberDrives.member(id,"mount"), id + "turnParent", HobberDrives.member(id,"screw"), id + "turnChild", ratio * initial);
    fixedPart(HobberDrives.member(id,"floating"), dr.floating, at.get("floating"), floatingPad);
    fixedPart(HobberDrives.member(id,"ring"), dr.ring, at.get("ring"), HobberDrives.member(id,"floating"));
    supportScrew(HobberDrives.member(id,"lead"), ScrewSupport.Fixed, dr.floating.support);
    fixedPart(HobberDrives.member(id,"coupling"), dr.coupling, at.get("coupling"), HobberDrives.member(id,"screw"));
    fixedPart(HobberDrives.member(id,"motorShaft"), motor.shaft(), at.get("motor"), HobberDrives.member(id,"coupling"));
    mountContacts.push({parent: fixedPad, child: HobberDrives.member(id,"mount")});
    mountContacts.push({parent: floatingPad, child: HobberDrives.member(id,"floating")});
  }
  /** Where a scale's seat bar rests on its host: the far face of the bar from the housing, normal toward the host. */
  public function scaleBarContact(a:HobberScaleAxis):AssemblyFrame {
    var face = HobberScales.housingFace(a), t = a.toSeat, bar = HobberScales.SEAT_BAR;
    var b = a.seatBar;
    if (b == null) throw "scale has no seat bar";
    // The bar's contact face is one bar thickness behind the housing's mounting face; the bar is 20 wide across the travel.
    return F.alongY(-(face[1] + t[1] * bar), face[0] + t[0] * bar, HobberFrame.PART_DATUM_Z + face[2] + t[2] * bar, -t[1], t[0], t[2]);
  }
  /** One axis's sealed scale: the housing on its seat, the head on its carrier, and the head as the axis's position encoder. */
  public function scale(a:HobberScaleAxis, seatHost:String, headHost:String):Void {
    group("Scale");
    var housing = HobberScales.member(a.id), head = HobberScales.member(a.id,true);
    fixedPart(housing, a.housing, HobberFrame.sourcePose(a.housingPose), seatHost, F.compose(HobberFrame.sourcePose(a.housingPose), a.housing.connector("mount").frame));
    fixedPart(head, a.head, HobberFrame.sourcePose(a.headPose), headHost, F.compose(HobberFrame.sourcePose(a.headPose), a.head.connector("mount").frame));
    addEncoder(a.id + "position", a.id, head);
    exposePort(a.id + ".biss", head, "biss");
  }

}
