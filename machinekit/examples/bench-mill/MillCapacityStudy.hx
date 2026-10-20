import machinekit.milling.*;
import machinekit.motion.*;
import machinekit.assembly.*;
import machinekit.assembly.PosedParts.PartBox;
import machinekit.component.*;
import cadkit.modeling.*;
import materia.assembly.AssemblyFrames as F;
import materia.assembly.AssemblyRecord.AssemblyFrame;
import materia.project.SceneArtifact;

/** Separate capacity study. Shares the detailed cross-slide geometry with Small.
 * The shared Z axis is complete geometry with provisional drive sizing; XY drives are not fitted.
 */
class MillCapacityStudy {
 static var assertions=0;
 static function check(ok:Bool,label:String):Void { assertions++;if(!ok)throw label; }
 static function near(a:Float,b:Float,label:String):Void check(Math.abs(a-b)<1e-5,label+": "+a+" vs "+b);

 public static function build(l:MillCapacityLayout,fixtureEnvelope:Bool=true):MachineAssembly {
  var z=l.zLayout(),frame=new MillFrame(l.frame,z),xy=new MillGuidedXYStage(l.guidedXY());
  var a=MillFramePreview.assemble(new BenchMill(false,false),frame,xy);
  if(fixtureEnvelope) {
   a.addComponent("fixture-envelope",new MillPanel(200,150,l.fixtureHeight),F.translation(0,l.spindleY,l.tableTop));
   a.addMemberConnector("xyStage/table","fixtureSeat",F.translation(0,0,l.tableThickness));
   a.addMemberConnector("fixture-envelope","seat",F.identity());
   a.addMate("fixture-on-table","fixed","xyStage/table","fixtureSeat","fixture-envelope","seat");
  }
  return a;
 }

 static function scene(a:MachineAssembly):haxe.io.Bytes return MillProject.scene(a,"mill-large",[
  "400 x 300 x 380 mm layout; Shared ribbed saddle and T-slot table; XY drives are not fitted.",
  "Shared full Z and spindle assemblies, including pneumatic release. Motor/screw sizing and structural performance remain provisional."]);
 public static function preview():haxe.io.Bytes return scene(build(new MillCapacityLayout()));

 static function zMoving(id:String):Bool return StringTools.startsWith(id,"spindle/")||(StringTools.startsWith(id,"zAxis/")&&MillZAxis.isTranslating(id.substr(6)));
 static function xyMoving(id:String):Bool return id=="fixture-envelope" ||
  (StringTools.startsWith(id,"xyStage/")&&!StringTools.startsWith(id,"xyStage/railY"));
 static function xMoving(id:String):Bool return id=="fixture-envelope"||id=="xyStage/table"||StringTools.startsWith(id,"xyStage/blockX");

 static function sweep(b:PartBox,id:String,l:MillCapacityLayout):PartBox {
  var dx=xMoving(id)?l.xTravel/2+l.overtravel:0.0;
  var dy=xyMoving(id)?l.yTravel/2+l.overtravel:0.0;
  var dz=zMoving(id)?l.zTravel+l.overtravel:0.0;
  return {minX:b.minX-dx,maxX:b.maxX+dx,minY:b.minY-dy,maxY:b.maxY+dy,
   minZ:b.minZ-(zMoving(id)?l.overtravel:0),maxZ:b.maxZ+dz};
 }
 static function boxPart(b:PartBox):Part return Part.box(b.maxX-b.minX,b.maxY-b.minY,b.maxZ-b.minZ)
  .translated(new Vector((b.minX+b.maxX)/2,(b.minY+b.maxY)/2,b.minZ));

 // Extrude the catalog profile without mounting holes: a superset of the
 // running rail at every carriage station, including every drilled section.
 static function runningSection():Part {
  var section=HgReferenceData.section("HGR25R"),edges:Array<Curve> = [];
  var first=new Vector(section[0][0],section[0][1]);
  for(i in 0...section.length) {
   var row=section[i],p=new Vector(row[0],row[1]);
   var q=i==section.length-1?first:new Vector(section[i+1][0],section[i+1][1]);
   edges.push(row.length==4?Curve.line(p,q):Curve.arc(p,new Vector(row[4],row[5]),q));
  }
  var wire=Curve.wire(edges),face=Sketch.face(wire),part=face.extrude(300);
  face.close();wire.close();for(edge in edges)edge.close();return part;
 }

 static function drawing(l:MillCapacityLayout):String {
  var s=0.34,topX=270.0,topY=240.0,sideX=610.0,sideY=710.0;
  var svg=['<svg xmlns="http://www.w3.org/2000/svg" width="1200" height="840" viewBox="0 0 1200 840">',
   '<style>text{font:15px sans-serif;fill:#172b3a}.title{font-size:23px;font-weight:bold}.note{font-size:13px}.base{fill:#c5cbd0;stroke:#394653}.column{fill:#85929d;stroke:#394653}.moving{fill:#b9d7e8;stroke:#23668d}.head{fill:#a8c5b1;stroke:#396e47}.sweep{fill:none;stroke:#d47b16;stroke-dasharray:6 4}.axis{stroke:#bb3939;stroke-dasharray:3 3}</style>',
   '<rect width="1200" height="840" fill="white"/>',
   '<text class="title" x="30" y="35">Mill capacity study: 400 x 300 x 380 mm</text>',
   '<text x="30" y="61">Packaging only; drive integration, loads, stiffness and vibration remain open.</text>'];
  function label(x:Float,y:Float,t:String):Void svg.push('<text x="'+x+'" y="'+y+'">'+t+'</text>');
  function rect(x:Float,y:Float,w:Float,h:Float,c:String):Void svg.push('<rect class="'+c+'" x="'+x+'" y="'+y+'" width="'+w+'" height="'+h+'"/>');
  // Top view: front is at the bottom. Machine X runs left/right.
  function top(x:Float,y:Float,w:Float,d:Float,c:String):Void rect(topX+(x-w/2)*s,topY-(y+d/2)*s,w*s,d*s,c);
  top(0,l.frame.baseCentreY,l.frame.baseWidth,l.frame.baseDepth,"base");
  top(0,l.frame.columnCentreY,l.frame.columnWidth,l.frame.columnDepth,"column");
  top(0,l.spindleY,l.saddleWidth,l.saddleDepth,"head");
  top(0,l.spindleY,l.tableWidth+l.xTravel,l.tableDepth+l.yTravel,"sweep");
  top(0,l.spindleY,l.tableWidth,l.tableDepth,"moving");
  svg.push('<circle cx="'+topX+'" cy="'+(topY-l.spindleY*s)+'" r="4" fill="#bb3939"/>');
  label(35,78,"TOP / table at mid travel");
  label(35,570,'Base: '+l.frame.baseWidth+' x '+l.frame.baseDepth+' mm');
  label(35,595,'Table: '+l.tableWidth+' x '+l.tableDepth+' mm');
  label(35,620,'Dashed: complete table travel envelope');
  label(35,645,'Spindle reach from column face: '+l.spindleReach+' mm');
  label(35,670,'Table-edge margins: X 150 / Y 50 mm');
  // Side view: horizontal coordinate is machine Y; Z is vertical.
  function side(y:Float,z:Float,d:Float,h:Float,c:String):Void rect(sideX+(y-l.frame.front)*s,sideY-(z+h)*s,d*s,h*s,c);
  side(l.frame.front,0,l.frame.baseDepth,l.frame.baseHeight,"base");
  side(l.frame.columnFront+4,l.frame.railPlane,l.frame.columnDepth,l.frame.columnHeight,"column");
  side(l.spindleY-l.saddleDepth/2,l.saddleBottom,l.saddleDepth,l.saddleHeight,"head");
  for(sign in [-1,1]) {
   side(l.spindleY+sign*l.xRailSpacing/2-30,l.xRailPlane,60,40,"column");
   side(l.spindleY+sign*l.xRailSpacing/2-30,l.xRailPlane+40,60,l.riserHeight,"moving");
  }
  side(l.spindleY-l.tableDepth/2,l.tableBottom,l.tableDepth,l.tableThickness,"moving");
  side(l.spindleY-75,l.tableTop,150,l.fixtureHeight,"moving");
  side(-22,l.frame.railPlane+l.frame.zRailStart,22,l.frame.zRailLength,"moving");
  side(l.headRearY,l.carriageCentreZ-l.carriageHeight/2,l.carriageThickness,l.carriageHeight,"head");
  side(l.headRearY-l.headDepth,l.headBottom,l.headDepth,l.headHeight,"head");
  side(l.headRearY-l.headDepth,l.headBottom,l.headDepth,l.headHeight+l.zTravel,"sweep");
  var axisX=sideX+(l.spindleY-l.frame.front)*s;
  svg.push('<line class="axis" x1="'+axisX+'" x2="'+axisX+'" y1="'+(sideY-l.tableTop*s)+'" y2="'+(sideY-(l.gaugeZ+l.zTravel+300)*s)+'"/>');
  label(610,95,"SIDE / head at lower stop");
  label(610,750,'Table top: '+l.tableTop+' mm above casting bottom');
  label(610,775,'Gauge to table: '+l.noseMin+' to '+l.noseMax+' mm');
  label(610,800,'Column: '+l.frame.columnWidth+' x '+l.frame.columnDepth+' x '+l.frame.columnHeight+' mm');
  label(35,760,"HG25 rails: X "+l.xRailLength+" / Y "+l.frame.yRailLength+" / Z "+l.frame.zRailLength+" mm");
  label(35,785,"5 mm overtravel included in clearance checks.");
  label(35,810,"150 kg payload is a design target; not a verified capacity.");
  svg.push('</svg>');return svg.join("\n");
 }

 public static function main():Void {
  var l=new MillCapacityLayout(),a=build(l);a.validate();
  var zLayout=l.zLayout(),zAxis=new MillZAxis(zLayout),zFrame=new MillFrame(l.frame,zLayout);
  MillZAxisIntegrity.check(zAxis,zFrame,l.overtravel).throwIfErrors();
  check(true,"Shared Z assembly clears frame and stationary hardware through overtravel");
  var model=new AssemblyModel("mm");a.addTo(model,"");var state=new AssemblyState(model.definition("capacity"));
  for(x in [-l.xTravel/2,0,l.xTravel/2])for(y in [-l.yTravel/2,0,l.yTravel/2])for(z in [-l.zTravel,-l.zTravel/2,0.0]) {
   state.setJoint("xyStage/x",x);state.setJoint("xyStage/y",y);state.setJoint("zAxis/z",z);state.forwardKinematics();
   var table=state.worldPose("xyStage/table"),saddle=state.worldPose("xyStage/saddle"),head=state.worldPose("zAxis/head"),spindle=state.worldPose("spindle/spindle");
   near(table.x,-x,"table X");near(table.y,l.spindleY-y,"table Y");near(saddle.x,0,"saddle independent of X");
   near(head.z,l.headBottom+l.zTravel+z,"head Z");near(spindle.z,l.gaugeZ+l.zTravel+z,"spindle gauge Z");
   check(l.tableWidth/2-Math.abs(x)>=150,"X table edge coverage");check(l.tableDepth/2-Math.abs(y)>=50,"Y table edge coverage");
   check(spindle.z-l.tableTop>=l.noseMin,"minimum gauge-to-table distance");
   var motorPose:AssemblyFrame=state.worldPose("zAxis/motor"),nutPose:AssemblyFrame=state.worldPose("zAxis/nut");
   near(motorPose.z,zLayout.motorFace,"Z motor remains fixed");
   near(nutPose.z,zLayout.headCentre+zLayout.nut.bodyLength/2+z,"Z nut follows carriage");
   near(state.joint("zAxis/screw-turn"),zLayout.nut.rotationFor(-z),"Z screw follows signed lead");
  }
  state.setJoint("spindle/drawbar-release",0);state.setJoint("spindle/pistonRod-release",0);state.forwardKinematics();
  var clampedBar:AssemblyFrame=state.worldPose("spindle/drawbar"),retractedRod:AssemblyFrame=state.worldPose("spindle/pistonRod");
  state.setJoint("spindle/drawbar-release",4.5);state.setJoint("spindle/pistonRod-release",MillDrawbarLayout.RELEASE_STROKE);state.forwardKinematics();
  var releasedBar:AssemblyFrame=state.worldPose("spindle/drawbar"),extendedRod:AssemblyFrame=state.worldPose("spindle/pistonRod");
  near(releasedBar.z-clampedBar.z,-4.5,"Shared drawbar inspection stroke");
  near(extendedRod.z-retractedRod.z,-MillDrawbarLayout.RELEASE_STROKE,"Shared pneumatic release inspection stroke");
  state.setJoint("spindle/drawbar-release",0);state.setJoint("spindle/pistonRod-release",0);state.forwardKinematics();
  // True continuous running-section check. Bounding boxes would fill the ball tracks.
  var block=new HgBlock("HGH25CA").geometry(),rail=runningSection();
  var uniform=rail.translated(new Vector(0,0,-150));rail.close();
  var common=block.intersect(uniform);check(common.volume()<1e-5,"HG25 block clears its running section");common.close();
  var misplaced=block.translated(new Vector(0,-2,0));common=misplaced.intersect(uniform);
  check(common.volume()>1,"guide audit detects a 2 mm seating error");common.close();misplaced.close();uniform.close();block.close();

  state.setJoint("xyStage/x",0);state.setJoint("xyStage/y",0);state.setJoint("zAxis/z",-l.zTravel);state.forwardKinematics();
  var cache=new PosedParts(),boxes:Map<String,PartBox> = [],rows=a.components();
  for(id in ["frame/base/casting","frame/column/casting","zAxis/head","xyStage/saddle","xyStage/table","zAxis/carriage"]) {
   var shape=cache.local(rows.filter(row -> row.id==id)[0].component);check(shape.valid()&&shape.solidCount()==1,"Connected valid study solid: "+id);
  }
  for(row in rows)boxes.set(row.id,sweep(cache.box(row.component,state.worldPose(row.id)),row.id,l));
  var audited=0;
  function clear(first:String,second:String):Void {
   if(first=="spindle/tool"&&second=="fixture-envelope")return;
   var b1=boxes.get(first),b2=boxes.get(second);audited++;
   if(PosedParts.apart(b1,b2))return;
   var p=boxPart(b1),q=boxPart(b2),intersection=p.intersect(q),v=intersection.volume();intersection.close();p.close();q.close();
   check(v<1e-5,'Continuous swept envelopes collide: $first / $second ($v mm3)');
  }
  for(row in rows)if(xyMoving(row.id)||StringTools.startsWith(row.id,"spindle/"))for(other in rows)if(!xyMoving(other.id)&&!zMoving(other.id)) {
   if(StringTools.startsWith(row.id,"xyStage/blockY")&&StringTools.startsWith(other.id,"xyStage/railY"))continue;
   if(StringTools.startsWith(row.id,"blockZ")&&StringTools.startsWith(other.id,"railZ"))continue;
   clear(row.id,other.id);
  }
  for(row in rows)if(zMoving(row.id))for(other in rows)if(xyMoving(other.id))clear(row.id,other.id);
  // Guide support includes the overtravel and the full imported axial envelopes.
  var guide=HgReferenceData.spec("HGH25CA"),lo=HgReferenceData.low("HGH25CA"),hi=HgReferenceData.high("HGH25CA");
  check(l.xRailLength/2-l.xTravel/2-l.overtravel-l.xBlockSpan-Math.max(-lo[2],hi[2])>=guide.railEndMargin,"X guide end margins");
  check(l.spindleY-l.yTravel/2-l.overtravel-l.yBlockSpan+lo[2]-l.frame.yRailStart>=guide.railEndMargin,"Y front guide end margin");
  check(l.frame.yRailStart+l.frame.yRailLength-(l.spindleY+l.yTravel/2+l.overtravel+l.yBlockSpan+hi[2])>=guide.railEndMargin,"Y rear guide end margin");
  check(l.carriageCentreZ-l.overtravel-l.zBlockSpan+lo[2]-(l.frame.railPlane+l.frame.zRailStart)>=guide.railEndMargin,"Z lower guide end margin");
  check(l.frame.railPlane+l.frame.zRailStart+l.frame.zRailLength-(l.carriageCentreZ+l.zTravel+l.overtravel+l.zBlockSpan+hi[2])>=guide.railEndMargin,"Z upper guide end margin");
  // Straight front approaches to the head flange bolts, with the tool
  // entering the counterbore and no installed receiving bolt in the tool volume.
  var axisPoses:Map<String,AssemblyFrame> = zAxis.solvedPoses();
  var headPart=cache.posed(zAxis.head,axisPoses.get("head"));
  for(row in zAxis.components())if(StringTools.startsWith(row.id,"headBolt")) {
   var p:AssemblyFrame=axisPoses.get(row.id);
   var tool=Part.cylinderAlongY(5.5,zLayout.headBack-zAxis.head.parameterNumber("reach")-20,
    zLayout.headBack-zAxis.head.parameterNumber("rearPlate")+MillZHead.COUNTERBORE_DEPTH,p.x,p.z);
   var hit=tool.intersect(headPart);check(hit.volume()<1e-5,"Larger head bolt retains straight tool access");hit.close();tool.close();
  }
  headPart.close();
  // Motor body can lift vertically out after removal of its mounting screws.
  var motorBox=cache.box(zLayout.motor,axisPoses.get("motor"));
  var extraction:PartBox={minX:motorBox.minX,maxX:motorBox.maxX,minY:motorBox.minY,maxY:motorBox.maxY,
   minZ:motorBox.minZ,maxZ:motorBox.maxZ+150};
  var extractionPart=boxPart(extraction);
  for(row in zFrame.components()) {
   var p:AssemblyFrame=zFrame.solvedPoses().get(row.id);
   if(PosedParts.apart(extraction,cache.box(row.component,p)))continue;
   var obstacle=cache.posed(row.component,p),hit=extractionPart.intersect(obstacle);
   check(hit.volume()<1e-5,"Z motor vertical extraction clears "+row.id);hit.close();obstacle.close();
  }
  extractionPart.close();
  cache.close();
  var restored=MachineAssembly.decode(a.encode(),MachineKitComponents.standard());restored.validate();
  check(restored.components().length==a.components().length,"capacity assembly persists");
  MillSpindleIntegrity.checkPayload(a,zLayout,{xHalfTravel:l.xTravel/2,yHalfTravel:l.yTravel/2,
   member:xyMoving,table:xMoving,fixed:id -> !xyMoving(id),cuttingTarget:"fixture-envelope"}).throwIfErrors();
  MillSpindleIntegrity.checkMotorRemoval(a).throwIfErrors();
  check(true,"Shared full spindle payload and motor removal clearances");
  var previewBytes=scene(a),decoded=SceneArtifact.decode(previewBytes);
  for(id in ["carriage","head","screw","nut","motor","fixedBearing","floatingBearing","coupling","brake-shell-0"])
   check(decoded.parts.filter(part -> part.id=="zAxis/"+id).length==1,"Preview retains shared Z member: "+id);
  for(id in ["motor","belt","drawbar","pistonRod","cylinder","reactionFrame","holder"])
   check(decoded.parts.filter(part -> part.id=="spindle/"+id).length==1,"Preview retains complete spindle: "+id);
  check(decoded.parts.filter(part -> part.id=="carriage"||part.id=="head").length==0,"No duplicate placeholder Z members");
  var dir=Sys.getEnv("MATERIA_ROOT")+"/machinekit/examples/bench-mill/build/capacity";sys.FileSystem.createDirectory(dir);
  var report={status:"packaging study; drives and structural performance not qualified",units:"mm",
   travel:{x:l.xTravel,y:l.yTravel,z:l.zTravel},table:{width:l.tableWidth,depth:l.tableDepth,top:l.tableTop,payloadTargetKg:l.payloadTargetKg},
   spindle:{columnReach:l.spindleReach,gaugeToTableMin:l.noseMin,gaugeToTableMax:l.noseMax,fixtureEnvelopeHeight:l.fixtureHeight},
   base:{width:l.frame.baseWidth,depth:l.frame.baseDepth,height:l.frame.baseHeight,front:l.frame.front,rear:l.frame.rear},
   column:{width:l.frame.columnWidth,depth:l.frame.columnDepth,height:l.frame.columnHeight},
   guides:{rail:"HGR25R",block:"HGH25CA",xLength:l.xRailLength,yLength:l.frame.yRailLength,zLength:l.frame.zRailLength,
    xSpacing:l.xRailSpacing,ySpacing:l.frame.yRailSpacing,zSpacing:l.frame.zRailSpacing},
   zAxis:{motor:zLayout.motor.designation,screwLength:zLayout.screw.totalLength,screwDiameter:zLayout.screw.thread.screwDiameter,lead:zLayout.screw.thread.signedLead(),driveSizing:"reference only; not sized for larger machine"},
   checks:{assertions:assertions,continuousEnvelopePairs:audited,overtravel:l.overtravel,poses:27,service:["straight head bolt access","150 mm vertical motor-body extraction after unbolting"]},
   pending:["Saddle/table detail and XY drive integration; Z drive sizing remains provisional","Screw diameter, lead, supports and critical-speed checks","Leadshine motor sizing from moving masses and duty","Spindle drive/release qualification, ATC and enclosure packaging","Structural tool-to-workpiece compliance and modes","Finished mass, payload reactions, support stiffness and feet selection"]};
  sys.io.File.saveContent(dir+"/dimensions.svg",drawing(l));
  sys.io.File.saveContent(dir+"/checks.json",haxe.Json.stringify(report,null,"  "));
  sys.io.File.saveContent(dir+"/mill-capacity.assembly.json",a.encode());
  sys.io.File.saveBytes(dir+"/mill-capacity.mtrg",previewBytes);
  Sys.println('Mill capacity: $assertions assertions; $audited continuous envelope pairs passed');
 }
}
