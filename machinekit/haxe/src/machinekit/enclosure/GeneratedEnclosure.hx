package machinekit.enclosure;

import machinekit.assembly.MachineAssembly;
import materia.assembly.AssemblyFrames as F;
import materia.assembly.AssemblyRecord.AssemblyFrame;

/** Shell geometry and declared fabrication interfaces. Qualified seals/retention are separate hardware work. */
class GeneratedEnclosure extends MachineAssembly {
  public final layout:EnclosureLayout;
  public final servicePanelIds:Array<String> = ["hobService","swivelService"];
  public final serviceAccess:Array<{panelId:String,bounds:machinekit.assembly.PosedParts.PartBox}>;
  public final seamMethod:String="Continuous wet-rated sealant on tray seams; gasketed removable panels and plinth interface (unqualified)";
  public function new(layout:EnclosureLayout) {
    super();this.layout=layout;
    var w=layout.width(),d=layout.depth(),h=layout.height(),cx=layout.centreX(),cy=layout.centreY(),floor=layout.input.floorZ;
    serviceAccess=[
      {panelId:"hobService",bounds:{minX:layout.lowX-400,maxX:layout.lowX+10,minY:layout.frontY+50,maxY:layout.rearY-50,minZ:floor+50,maxZ:layout.roofZ-50}},
      {panelId:"swivelService",bounds:{minX:layout.highX-10,maxX:layout.highX+400,minY:layout.frontY+50,maxY:layout.rearY-50,minZ:floor+50,maxZ:layout.roofZ-50}}
    ];
    include("frame",new EnclosureFrame(w,d,h),F.translation(cx,cy,floor));
    var root=F.translation(layout.lowX-20,layout.frontY-20,floor-40);
    function add(id:String,panel:EnclosurePanel,pose:AssemblyFrame):Void {
      addComponent(id,panel,pose);
      addMemberConnector("frame/post00",id,F.compose(F.inverse(root),F.compose(pose,panel.connector("mount").frame)));
      addMate(id+"-mount","fixed","frame/post00",id,id,"mount");
    }
    add("hobService",new EnclosurePanel(d,h,"compact-laminate-6"),F.fromRotationMatrix(layout.lowX-20,cy,floor,[0,-1,0,1,0,0,0,0,1]));
    add("swivelService",new EnclosurePanel(d,h,"compact-laminate-6"),F.fromRotationMatrix(layout.highX+20,cy,floor,[0,-1,0,1,0,0,0,0,1]));
    add("rearMistPanel",new EnclosureCutPanel(w,h,"compact-laminate-6",0,0,0,h-120,100),F.translation(cx,layout.rearY+20,floor));
    // X width and Z height become machine X/Y; the sheet thickness becomes machine Z.
    add("roof",new EnclosurePanel(w,d,"compact-laminate-6"),F.fromRotationMatrix(cx,layout.frontY,layout.roofZ+20,[1,0,0,0,0,1,0,-1,0]));
    var left=layout.doorX-layout.doorWidth/2-layout.lowX,right=layout.highX-layout.doorX-layout.doorWidth/2;
    add("frontLeft",new EnclosurePanel(left,h,"compact-laminate-6"),F.translation(layout.lowX+left/2,layout.frontY-20,floor));
    add("frontRight",new EnclosurePanel(right,h,"compact-laminate-6"),F.translation(layout.highX-right/2,layout.frontY-20,floor));
    var b=layout.input.plinth;
    add("tray",new EnclosureCutPanel(w,d,"steel-2",b.maxX-b.minX,b.maxY-b.minY,(b.minX+b.maxX)/2-cx,(b.minY+b.maxY)/2-layout.frontY),
      F.fromRotationMatrix(cx,layout.frontY,floor-3,[1,0,0,0,0,1,0,-1,0]));
    // Upstands follow the shell's infill planes; the central opening mates to the plinth's own chip volume.
    add("trayFrontLip",new EnclosurePanel(w,30,"steel-2"),F.translation(cx,layout.frontY,floor-4));
    add("trayRearLip",new EnclosurePanel(w,30,"steel-2"),F.translation(cx,layout.rearY,floor-4));
    for(side in [-1,1])add(side<0?"trayLeftLip":"trayRightLip",new EnclosurePanel(d,30,"steel-2"),
      F.fromRotationMatrix(side<0?layout.lowX:layout.highX,cy,floor-4,[0,-1,0,1,0,0,0,0,1]));
    var pose=F.translation(layout.doorX,layout.frontY-(layout.input.opposedDoors==true?80:50),floor+30);
    if(layout.input.opposedDoors==true)include("door",new OpposedSlidingDoor(layout.doorWidth,h-30),pose);
    else include("door",new SlidingDoor(layout.doorWidth,h-30),pose);
    addMemberConnector("frame/post00","door",F.compose(F.inverse(root),pose));
    addMate("door-mount","fixed","frame/post00","door","door/rail","mount");
  }
}
