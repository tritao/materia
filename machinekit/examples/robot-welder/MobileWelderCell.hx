import machinekit.assembly.MachineAssembly;
import machinekit.component.Solids;
import machinekit.welding.WorkClamp;
import machinekit.welding.Weldment;
import machinekit.welding.WeldingEquipment;
import machinekit.welding.WeldingEquipment.WeldingEquipmentData;
import materia.assembly.AssemblyFrames;

/** A mobile welding robot and an independently grounded workpiece on a narrow welding table. */
class MobileWelderCell extends MachineAssembly {
  public static final ORIGIN:MobileBaseCell.FloorPose = {x: -1500, y: -800, yaw: 0};
  public final robot:MobileWeldingRobot;
  public final work:WeldingWorkpiece;
  public final tableTop:Float;
  public function new() {
    super();
    robot = new MobileWeldingRobot();
    work = new WeldingWorkpiece();
    tableTop = RobotArm.TABLE_TOP + MobileBase.DECK_Z + MobileBase.DECK_THICKNESS;
    include("robot", robot, MobileBaseCell.floorFrame(ORIGIN));
    addComponent("floor", new RoomBlock(8000, 6000, 50, "painted steel", "Floor"), AssemblyFrames.translation(0, 0, -50));
    addComponent("table", new ArmTable(300, 900, tableTop), AssemblyFrames.translation(2000, 0, 0));
    include("work", work);
    var desired:materia.assembly.AssemblyRecord.AssemblyFrame = {x: 2000, y: -150, z: tableTop,
      qx: 0, qy: 0, qz: Math.sin(Math.PI / 4), qw: Math.cos(Math.PI / 4)};
    var tableFrame = solvedPoses().get("table");
    var seat = AssemblyFrames.compose(AssemblyFrames.inverse(tableFrame),
      AssemblyFrames.compose(desired, memberConnectorFrame("work/basePlate", "base")));
    addMemberConnector("table", "weldmentSeat", seat);
    addMate("work-mount", "fixed", "table", "weldmentSeat", "work/basePlate", "base");
    addComponent("clamp", new WorkClamp());
    addMemberConnector("work/basePlate", "clampSeat", Solids.axial(WeldingCell.CLAMP_X, WeldingCell.CLAMP_Y,
      WeldingWorkpiece.PLATE_THICKNESS));
    addMate("work-clamp", "fixed", "work/basePlate", "clampSeat", "clamp", "contact");
    connectPorts("work-lead", "robot", "workLead", "clamp", "lead");
    var control = robot.port("control", "robot");
    exposePort("control", control.instanceId, control.portName);
  }
  public function weldment():Weldment return work.weldment().prefixed("work/");
  public function equipment():WeldingEquipmentData return WeldingEquipment.of(this, [weldment()]);
}
