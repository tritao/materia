import machinekit.assembly.MachineAssembly;
import machinekit.enclosure.GeneratedEnclosure;
import machinekit.enclosure.TrayClamp;
import machinekit.hobbing.FoundationLayout;
import machinekit.standard.SocketHeadCapScrew;
import materia.assembly.AssemblyFrames as F;

/** Separate clamp/bolt occurrences, anchored to local steel inserts rather than through the laminate. */
class HobberTrayMounts {
  public static function install(assembly:MachineAssembly,shell:GeneratedEnclosure):Void {
    var poses=assembly.solvedPoses(),casting=poses.get("machine/bed/casting"),i=0;
    for(row in FoundationLayout.trayClampSeats()) {
      var parent="machine/bed/"+row.id;
      var seat=F.compose(poses.get(parent),assembly.memberConnectorFrame(parent,"top"));
      var out=Math.abs(row.pose.y)>300?F.transformVector(casting,0,row.pose.y<0?-1:1,0):F.transformVector(casting,row.pose.x<0?-1:1,0,0);
      var pose=F.fromRotationMatrix(seat.x,seat.y,seat.z,[out.x,-out.y,0,out.y,out.x,0,0,0,1]);
      var height=row.clampHeight==null?machinekit.enclosure.TraySeat.TOP:row.clampHeight;
      var id="enclosureTrayClamp"+i,bolt="enclosureTrayBolt"+i,clamp=new TrayClamp(height),screw=SocketHeadCapScrew.metric("M6",height>11?35:30);
      assembly.addComponent(id,clamp,pose);
      assembly.addMemberConnector(id,"seat",F.compose(F.inverse(pose),seat));
      assembly.addMate("tray-clamp-seat-"+i,"fixed",parent,"top",id,"seat");
      assembly.addComponent(bolt,screw,F.compose(pose,F.translation(0,0,height+4)));
      assembly.addMate("tray-clamp-bolt-"+i,"fixed",id,"head",bolt,"head");
      i++;
    }
  }
}
