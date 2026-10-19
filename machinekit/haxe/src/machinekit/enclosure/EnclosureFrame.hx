package machinekit.enclosure;

import machinekit.assembly.MachineAssembly;
import machinekit.structural.ProfileMember;
import machinekit.structural.TSlotExtrusion;
import materia.assembly.AssemblyFrames;
import materia.assembly.AssemblyRecord.AssemblyFrame;

/** Twelve cut 40-series members. Dimensions describe the clear infill opening. */
class EnclosureFrame extends MachineAssembly {
  public final width:Float;
  public final depth:Float;
  public final height:Float;
  public final profileSize:Float = 40;

  public function new(width:Float, depth:Float, height:Float) {
    if (!(width > 0) || !(depth > 0) || !(height > 0) || !Math.isFinite(width + depth + height))
      throw "Enclosure frame needs finite positive clear dimensions";
    super();
    this.width = width; this.depth = depth; this.height = height;
    var s = profileSize;
    for (x in [-1, 1]) for (y in [-1, 1])
      member('post${x+1}${y+1}', height + 2*s,
        AssemblyFrames.translation(x*(width+s)/2, y*(depth+s)/2, -s));
    for (z in [0, 1]) {
      for (y in [-1, 1]) member('cross${z}${y+1}', width,
        AssemblyFrames.fromRotationMatrix(-width/2, y*(depth+s)/2, z*(height+s)-s/2, [0,0,1, 1,0,0, 0,1,0]));
      for (x in [-1, 1]) member('side${z}${x+1}', depth,
        AssemblyFrames.fromRotationMatrix(x*(width+s)/2, -depth/2, z*(height+s)-s/2, [1,0,0, 0,0,1, 0,-1,0]));
    }
  }

  function member(id:String, length:Float, pose:AssemblyFrame):Void {
    var stock = new ProfileMember(TSlotExtrusion.forSystem("TSLOT-40-SLOT8", 40, 40), length, "aluminium 6061");
    addComponent(id, stock, pose);
    if (id != "post00") {
      var root = AssemblyFrames.translation(-(width+profileSize)/2, -(depth+profileSize)/2, -profileSize);
      addMemberConnector("post00", id, AssemblyFrames.compose(AssemblyFrames.inverse(root),
        AssemblyFrames.compose(pose, stock.connector("start").frame)));
      addMate(id + "-mount", "fixed", "post00", id, id, "start");
    }
  }
}
