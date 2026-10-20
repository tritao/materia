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
    // Separate lifted-out blanks avoid trapping one large sheet around the assembled machine.
    function tile(id:String,x0:Float,y0:Float,x1:Float,y1:Float):Void {
      add(id,new EnclosurePanel(x1-x0,y1-y0,TraySeat.CONSTRUCTION),
        F.fromRotationMatrix((x0+x1)/2,y0,floor+TraySeat.BOTTOM+TraySeat.THICKNESS/2,[1,0,0,0,0,1,0,-1,0]));
    }
    tile("trayLeft",layout.lowX+2,layout.frontY+2,b.minX,layout.rearY-2);
    tile("trayRight",b.maxX,layout.frontY+2,layout.highX-2,layout.rearY-2);
    tile("trayFront",b.minX+TraySeat.SEAM,layout.frontY+2,b.maxX-TraySeat.SEAM,b.minY);
    var bx=(b.minX+b.maxX)/2;
    tile("trayRearLeft",b.minX+TraySeat.SEAM,b.maxY,bx-60,layout.rearY-2);
    tile("trayRearRight",bx+60,b.maxY,b.maxX-TraySeat.SEAM,layout.rearY-2);
    tile("trayRearCentre",bx-58,b.maxY+25,bx+58,layout.rearY-2);
    // Independent tray ledges on the inside of the lower frame; the tray does not carry posts.
    function support(id:String,blocks:Array<machinekit.structural.BlockBracket.BracketBlock>,pose:AssemblyFrame,material:String="aluminium 6061"):Void {
      addComponent(id,new machinekit.structural.BlockBracket(id,blocks,material),pose);
      addMemberConnector(id,"mount",F.identity());
      addMemberConnector("frame/post00",id,F.compose(F.inverse(root),pose));
      addMate(id+"-mount","fixed","frame/post00",id,id,"mount");
    }
    for(side in [-1,1]) {
      // Local X runs from the extrusion's inside face towards the tray opening.
      var x=side<0?layout.lowX:layout.highX;
      support(side<0?"trayLeftSupport":"trayRightSupport",[
        {min:[0.0,0,-40],max:[6.0,d,-10]},
        {min:[0.0,0,-10],max:[25.0,d,-4]}
      ],F.fromRotationMatrix(x,side<0?layout.frontY:layout.rearY,floor,[-side,0,0,0,-side,0,0,0,1]));
    }
    for(side in [-1,1]) {
      var y=side<0?layout.frontY:layout.rearY;
      support(side<0?"trayFrontSupport":"trayRearSupport",[
        {min:[0.0,0,-40],max:[w-50,6.0,-10]},
        {min:[0.0,0,-10],max:[w-50,25.0,-4]}
      ],F.fromRotationMatrix(side<0?layout.lowX+25:layout.highX-25,y,floor,[-side,0,0,0,-side,0,0,0,1]));
    }
    function box(id:String,x0:Float,y0:Float,z0:Float,x1:Float,y1:Float,z1:Float,material:String):Void {
      support(id,[{min:[0.0,0,0],max:[x1-x0,y1-y0,z1-z0]}],F.translation(x0,y0,floor+z0),material);
    }
    // Compressed gasket envelopes; the actual grade/free thickness still needs coolant qualification.
    function seal(id:String,x0:Float,y0:Float,x1:Float,y1:Float):Void
      box(id,x0,y0,-TraySeat.DEPTH,x1,y1,TraySeat.BOTTOM,"nitrile rubber NBR");
    seal("trayOuterSealLeft",layout.lowX,layout.frontY,layout.lowX+25,layout.rearY);
    seal("trayOuterSealRight",layout.highX-25,layout.frontY,layout.highX,layout.rearY);
    seal("trayOuterSealFront",layout.lowX+25,layout.frontY,layout.highX-25,layout.frontY+25);
    seal("trayOuterSealRear",layout.lowX+25,layout.rearY-25,layout.highX-25,layout.rearY);
    seal("trayInnerSealLeft",b.minX-18,b.minY-18,b.minX-6,b.maxY+18);
    seal("trayInnerSealRight",b.maxX+6,b.minY-18,b.maxX+18,b.maxY+18);
    seal("trayInnerSealFront",b.minX-6,b.minY-18,b.maxX+6,b.minY-6);
    seal("trayInnerSealRearLeft",b.minX-6,b.maxY+6,bx-60,b.maxY+18);
    seal("trayInnerSealRearRight",bx+60,b.maxY+6,b.maxX+6,b.maxY+18);
    // Joint backing bridges the dry gap to the casting; the rebate supports the rest of each seam.
    for(side in [-1,1]) {
      var x=side<0?b.minX:b.maxX,suffix=side<0?"Left":"Right";
      box("trayFrontJoint"+suffix,x-15,layout.frontY+25,-10,x+15,b.minY-25,-4,"aluminium 6061");
      box("trayRearJoint"+suffix,x-15,b.maxY+25,-10,x+15,layout.rearY-25,-4,"aluminium 6061");
      seal("trayFrontJointSeal"+suffix,x-10,layout.frontY+25,x+10,b.minY-18);
      seal("trayRearJointSeal"+suffix,x-10,b.maxY+18,x+10,layout.rearY-25);
    }
    // Stepped service bridge clears the X motor, whose rear end reaches into the ledge area.
    // Six-millimetre feet bear on deeper pockets; the cover is removable after the rear blanks.
    support("trayRearServiceBridge",[
      {min:[-75.0,0,-10],max:[-55.0,25,-4]},
      {min:[55.0,0,-10],max:[75.0,25,-4]},
      {min:[-58.0,0,-4],max:[-55.0,23,13]},
      {min:[55.0,0,-4],max:[58.0,23,13]},
      {min:[-55.0,21,-4],max:[55.0,23,13]},
      {min:[-75.0,0,13],max:[75.0,30,17]}
    ],F.translation(bx,b.maxY,floor),"steel");
    box("trayBridgeSealLeft",bx-75,b.maxY,11,bx-60,b.maxY+25,13,"nitrile rubber NBR");
    box("trayBridgeSealRight",bx+60,b.maxY,11,bx+75,b.maxY+25,13,"nitrile rubber NBR");
    box("trayBridgeSealRear",bx-75,b.maxY+25,11,bx+75,b.maxY+30,13,"nitrile rubber NBR");
    box("trayBridgeEdgeSealLeft",bx-60,b.maxY,-2,bx-58,b.maxY+25,11,"nitrile rubber NBR");
    box("trayBridgeEdgeSealRight",bx+58,b.maxY,-2,bx+60,b.maxY+25,11,"nitrile rubber NBR");
    box("trayBridgeEdgeSealRear",bx-58,b.maxY+23,-2,bx+58,b.maxY+25,11,"nitrile rubber NBR");
    for(side in [-1,1]) {
      var x=bx+side*60,suffix=side<0?"Left":"Right";
      box("trayRearCentreJoint"+suffix,x-15,b.maxY+25,-10,x+15,layout.rearY-25,-4,"aluminium 6061");
      seal("trayRearCentreJointSeal"+suffix,x-10,b.maxY+25,x+10,layout.rearY-25);
    }
    // Upstands follow the shell's infill planes; the central opening mates to the plinth's own chip volume.
    add("trayFrontLip",new EnclosurePanel(w+2,30,"steel-2"),F.translation(cx,layout.frontY,floor+TraySeat.TOP));
    add("trayRearLip",new EnclosurePanel(w+2,30,"steel-2"),F.translation(cx,layout.rearY,floor+TraySeat.TOP));
    for(side in [-1,1])add(side<0?"trayLeftLip":"trayRightLip",new EnclosurePanel(d-2,30,"steel-2"),
      F.fromRotationMatrix(side<0?layout.lowX:layout.highX,cy,floor+TraySeat.TOP,[0,-1,0,1,0,0,0,0,1]));
    var pose=F.translation(layout.doorX,layout.frontY-(layout.input.opposedDoors==true?80:50),floor+30);
    if(layout.input.opposedDoors==true)include("door",new OpposedSlidingDoor(layout.doorWidth,h-30),pose);
    else include("door",new SlidingDoor(layout.doorWidth,h-30),pose);
    addMemberConnector("frame/post00","door",F.compose(F.inverse(root),pose));
    addMate("door-mount","fixed","frame/post00","door","door/rail","mount");
  }
}
