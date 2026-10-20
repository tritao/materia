import machinekit.assembly.MachineAssembly;
import machinekit.enclosure.GeneratedEnclosure;
import machinekit.hobbing.FoundationLayout;
import machinekit.structural.BlockBracket;
import materia.assembly.AssemblyFrames as F;

/** Four dry-side mounting envelopes. Fastener sizing and insert retention are fabrication work. */
class HobberEnclosureMounts {
  public static function install(assembly:MachineAssembly,shell:GeneratedEnclosure):Void {
    var poses=assembly.solvedPoses(),l=shell.layout,root="enclosure/frame/post00",index=0;
    for(seat in FoundationLayout.enclosureSeats()) {
      var parent="machine/bed/"+seat.id,pad=poses.get(parent);
      if(pad==null)throw "Missing enclosure foundation insert "+parent;
      var face=F.compose(pad,assembly.memberConnectorFrame(parent,"top")),side=face.x<l.centreX()?-1:1;
      var outer=side<0?l.lowX-40:l.highX+40,reach=Math.abs(outer-face.x);
      if(reach<=10)throw "Frame support needs space outside the casting";
      // Local X points outwards, Y along the side, Z upwards. Top meets the lower extrusion underside.
      var top=l.input.floorZ-40-face.z,pose=F.fromRotationMatrix(face.x,face.y,face.z,[side,0,0,0,side,0,0,0,1]);
      var id="enclosureBaseMount"+index;
      var part=new BlockBracket("enclosure dry-side outrigger",[
        {min:[0.0,-40,-40],max:[10.0,40,top]},
        {min:[10.0,-40,top-10],max:[reach,40,top]}
      ],"steel");
      assembly.addComponent(id,part,pose);
      assembly.addMemberConnector(id,"casting",F.compose(F.inverse(pose),face));
      assembly.addMate("enclosure-base-pad-"+index,"fixed",parent,"top",id,"casting");
      var contact=F.translation(side<0?l.lowX-20:l.highX+20,face.y,l.input.floorZ-40);
      assembly.addMemberConnector(id,"frame",F.compose(F.inverse(pose),contact));
      assembly.addMemberConnector(root,"baseMount"+index,F.compose(F.inverse(poses.get(root)),contact));
      if(index==0)assembly.addMate("enclosure-frame-base","fixed",id,"frame",root,"baseMount"+index);
      else assembly.addConstraint("enclosure-frame-base-"+index,"fixed",id,"frame",root,"baseMount"+index,0.01);
      index++;
    }
  }
}
