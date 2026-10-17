package machinekit.hobbing;

import machinekit.assembly.MachineAssembly;
import machinekit.component.*;
import materia.assembly.AssemblyFrames as F;

/** Casting and its cast-in steel children, each separately itemised. */
class MineralFoundation extends MachineAssembly {
  public final body:MineralCasting;
  public final kind:String;
  public function new(kind:String){
    super();this.kind=kind;
    var d=FoundationLayout.dimensions(kind);
    body=new MineralCasting(new ComponentValues().setToken("kind",kind).setNumber("width",d[0]).setNumber("depth",d[1]).setNumber("height",d[2]));
    addComponent("casting",body);
    for(row in FoundationLayout.inserts(kind)){
      var part=FoundationLayout.insert(row);addComponent(row.id,part,row.pose);
      if(kind=="column"&&row.id=="basePlate"){
        addMemberConnector("basePlate","castingSeat",Solids.axial(0,0,10));
        addMate("casting-insert","fixed","basePlate","castingSeat","casting","mount");
      }else{
        addMemberConnector("casting",row.id+"Seat",F.compose(row.pose,part.connector("mount").frame));
        addMate(row.id+"-cast-in","fixed","casting",row.id+"Seat",row.id,"mount");
      }
      exposeConnector(row.id,row.id,"top");
    }
    if(kind=="column")exposeConnector("mount","basePlate","mount");
  }
  public function castingByWeight():{material:String,kg:Float}return body.castingByWeight();
}
