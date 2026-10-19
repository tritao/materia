package machinekit.robotics;

import materia.assembly.AssemblyFrames;

/** Optional electrical service provision for any generic CobotArm size class.
    Input ports are controller-side boundaries. Bridges declare connectivity only:
    no cable path through joints, rotary feedthrough, or unlimited wrist rotation is implied. */
class CobotToolServices {
  public static function install(arm:CobotArm,spec:ToolServiceSpec):ToolServiceModule {
    if(arm==null || spec==null) throw "Cobot tool service needs an arm and an electrical contract";
    if(arm.hasPort("toolPower")) throw "Cobot already has a tool electrical service";
    var housing:CobotToolLink=cast arm.links[5];
    var lower=new CobotToolServiceMount(housing,false),upper=new CobotToolServiceMount(housing,true);
    var socket=new ToolServiceModule(spec);
    arm.addComponent("toolServiceLower",lower);
    arm.addComponent("toolServiceUpper",upper);
    arm.addMemberConnector("link6","serviceCollar",AssemblyFrames.identity());
    for(id in ["toolServiceLower","toolServiceUpper"])
      arm.addMate(id+"-mount","fixed","link6","serviceCollar",id,"housing");
    arm.addComponent("toolServiceSocket",socket);
    arm.addMate("tool-service-socket","fixed","toolServiceUpper","socketMount","toolServiceSocket","mount");
    arm.exposePort("toolServicePowerIn","toolServiceSocket","powerIn");
    arm.exposePort("toolServiceSignalIn","toolServiceSocket","signalIn");
    arm.exposePort("toolPower","toolServiceSocket","power1");
    arm.exposePort("toolSignal","toolServiceSocket","signal1");
    arm.exposeConnector("toolServiceOutlet","toolServiceSocket","outlet1");
    return socket;
  }
}
