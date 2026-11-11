import hobber.HobberSkeleton;
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

class HobberAHead extends HobberUnit {
  public final swivel:ErobActuator;
  public function new(skeleton:HobberSkeleton) {
    super("aHead", skeleton);
    swivel = new ErobActuator(100, 20, true, "A", new ComponentValues().setNumber("brakeHoldingTorque", FlexsplineHobber.A_BRAKE_HOLDING), layout.swivelRange, FlexsplineHobber.A_ACCELERATION);
    include("swivel", swivel, heights.swivel);
    externalMate("swivel-mount", "swivel/housing", "zAxis", "saddle", F.alongY(0, layout.swivelNativeX + 6, heights.hobCentreZ, 0, 1, 0));
    host("output","swivel/output");
    exposePort("bus","swivel/housing","bus");

  }

}
