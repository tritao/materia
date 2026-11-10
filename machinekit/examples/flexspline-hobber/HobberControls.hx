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

class HobberControls {
  public static function install(machine:FlexsplineHobber):Void {
    for(axis in ["X","Z"]) machine.addMotor(axis+"drive",HobberDrives.member(axis,"screw-turn"),HobberDrives.member(axis,"motor"),axis+"drive-driver",1);
    machine.bindMotorBrake("Zdrive","Zmotor","Z.brakeRelease","Z.brakeReleased",250,30);
    machine.addMotor("Ydrive","Ymodule/screw-turn","Ymodule/motor","Ydrive-driver",1);
    for(i in 0...3) machine.connectPorts(["X","Y","Z"][i]+"-biss",["XscaleHead","Ymodule/scaleHead","ZscaleHead"][i],"biss","bissMaster","ch"+(i+1));
    machine.addMotor("table-drive",FlexsplineHobber.C,"table/"+machine.tableSpec.drivePart,"table-drive-driver",1);
    machine.connectPorts("swivel-power","supply48","power1","swivel/housing","power");
    machine.addMotor("swivel-drive",FlexsplineHobber.A,"swivel/housing","swivel/housing",1);
    machine.exposePort("swivel.bus","swivel/housing","bus");
    machine.addMotor("hob-drive",FlexsplineHobber.B_MOTOR,"spindle/motor","hob-drive-driver",1,null,
      {speedChannel:"hob.speed",directionChannel:"hob.direction",radiansPerSpeedUnit:machine.spindle.ratio*2*Math.PI/60});
    machine.addEncoder("table-feedback",FlexsplineHobber.C,"table/"+machine.tableSpec.encoderPart,"table-drive");
    machine.addEncoder("hob-angle",FlexsplineHobber.B,"spindle/encoder");
    machine.addEncoder("hob-motor",FlexsplineHobber.B_MOTOR,"spindle/motorEncoder","hob-drive");
    machine.bindMotorBrake("swivel-drive","swivel/housing","swivel.brakeRelease","swivel.brakeReleased",1000,30);
  }
}
