package machinekit.enclosure;

import machinekit.assembly.MachineAssembly;
import machinekit.structural.RectTubeMember;
import materia.assembly.AssemblyFrames;

/** Geometric guide and leaf with a real prismatic joint. Closed/open coordinate switches are exported with the assembly. */
class SlidingDoor extends MachineAssembly {
  public final stroke:Float;
  public final leaf:EnclosurePanel;
  public static inline final SPEED_MM_PER_SECOND:Float = 200;
  public static inline final SWITCH_TOLERANCE_MM:Float = 0.5;

  public function new(width:Float, height:Float, constructionId:String = "polycarbonate-6", gap:Float = 10) {
    if (!(gap > 0) || !Math.isFinite(gap)) throw "Door needs a finite positive running gap";
    super();
    leaf = new EnclosurePanel(width, height, constructionId);
    stroke = width + gap;
    var railPose = AssemblyFrames.fromRotationMatrix(-width/2, 0, height+gap+10,
      [0,0,1, 1,0,0, 0,1,0]);
    addComponent("rail", new RectTubeMember(20,20,2,width+stroke), railPose);
    addComponent("leaf", leaf);
    addMemberConnector("rail", "slide", AssemblyFrames.compose(AssemblyFrames.inverse(railPose), leaf.connector("mount").frame));
    addMateOnAxis("slide", "prismatic", "rail", "slide", "leaf", "mount", {x:1,y:0,z:0}, 0,
      {lower:0, upper:stroke, velocity:SPEED_MM_PER_SECOND, acceleration:400, effort:null});
    addSwitch("closed", "slide", {lower:0, upper:SWITCH_TOLERANCE_MM, velocity:null, effort:null});
    addSwitch("open", "slide", {lower:stroke-SWITCH_TOLERANCE_MM, upper:stroke, velocity:null, effort:null});
    addMemberConnector("rail", "mount", AssemblyFrames.inverse(railPose));
    exposeConnector("mount", "rail", "mount");
  }
}
