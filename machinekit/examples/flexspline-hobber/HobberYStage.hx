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

class HobberYStage extends HobberUnit {
  public final yModule:ScrewLinearModule;
  public function new() {
    super("yStage");
    yModule=new ScrewLinearModule(layout.moduleSpec(),new HobberYPart(new ComponentValues().setToken("role","base")),new HobberYPart(new ComponentValues().setToken("role","carriage")));
    yModule.takeServiceRemoval("carriage");
    include("Ymodule",yModule,heights.yModule);
    externalMate("Ymodule-mount","Ymodule/base","aHead","output",F.alongY(0, layout.swivelNativeX, heights.hobCentreZ, 0, -1, 0));
    host("carriage","Ymodule/carriage");
    exposePort("Y.biss","Ymodule/scaleHead","biss");

  }

}
