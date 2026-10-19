package machinekit.enclosure;

import machinekit.assembly.MachineAssembly;
import machinekit.structural.RectTubeMember;
import materia.assembly.AssemblyFrames;

/** Geometric guide and leaf with a real prismatic joint. Actuator and guard logic are EN5 work. */
class SlidingDoor extends MachineAssembly {
  public final stroke:Float;
  public final leaf:EnclosurePanel;

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
      {lower:0, upper:stroke, velocity:null, effort:null});
    addMemberConnector("rail", "mount", AssemblyFrames.inverse(railPose));
    exposeConnector("mount", "rail", "mount");
  }
}
