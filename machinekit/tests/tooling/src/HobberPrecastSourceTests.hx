import machinekit.hobbing.FoundationLayout;
import materia.casting.PrecastStrut;
import materia.casting.CastingPatterns;
import materia.casting.CastingBounds;
import materia.casting.BoxAlgebra;

/** Source-only checks: no native shape construction or kit export. */
@:access(materia.tooling.casting.CastingMoldPlanner)
class HobberPrecastSourceTests {
 static function main():Void {
  // A socket must not use the bounding rectangle over an internal datum hole
  // or across a re-entrant branch of the actual foam contact.
  var profile=materia.casting.CastingProfiles.rectangle(0,0,120,100);
  var hole=materia.casting.CastingProfiles.rectangle(45,35,30,30);
  var receiver={x:0.0,y:0.0,z:0.0,width:120.0,depth:100.0,height:1.0};
  var patch=materia.tooling.casting.EPSAccessNetworks.contactRectangle([
   {depth:0,contour:profile},{depth:1,contour:hole}],receiver,'contact-with-datum',20);
  if(BoxAlgebra.intersection(patch,{x:45.0,y:35.0,z:0.0,width:30.0,depth:30.0,height:1.0})!=null)
   throw 'Certified EPS contact includes an internal datum hole';
  var concave=materia.tooling.panel.PanelContours.polygon([
   {x:0.0,y:0.0},{x:120.0,y:0.0},{x:120.0,y:40.0},
   {x:40.0,y:40.0},{x:40.0,y:100.0},{x:0.0,y:100.0}]);
  patch=materia.tooling.casting.EPSAccessNetworks.contactRectangle([{depth:0,contour:concave}],receiver,'contact-with-branch',20);
  if(BoxAlgebra.volume(BoxAlgebra.subtract([patch],[
   {x:0.0,y:0.0,z:0.0,width:120.0,depth:40.0,height:1.0},
   {x:0.0,y:40.0,z:0.0,width:40.0,depth:60.0,height:1.0}]))>1e-7)
   throw 'Certified EPS foot crosses a concave branch: '+haxe.Json.stringify(patch);
  var layout=machinekit.hobbing.HobberXLayout.front();
  Sys.println('Hobber source layout: '+haxe.Json.stringify(layout));
  var values=FoundationLayout.bodyValues('bed');
  var struts=PrecastStruts.decode(values.token('precastStruts'));
  var cuts=CastingPatterns.decode(values.token('features'));
  if(struts.length!=12)throw 'Two enclosed cores require three upper and three lower source struts each';
  var boxes=[for(feature in cuts)CastingBounds.feature(feature)];
  for(strut in struts){
   if(BoxAlgebra.volume(BoxAlgebra.subtract([strut.box],boxes))!=BoxAlgebra.volume([strut.box]))
    throw 'Source cavity removes precast strut '+strut.id;
   if(cuts.filter(feature->feature.id==strut.coreFeatureId||feature.derivedFrom==strut.coreFeatureId).length==0)
    throw 'Source strut lost its parent core '+strut.id;
  }
  Sys.println('Hobber source struts: '+struts.length+' retained compound records; source cavities preserve them');
  var setupDocument=HobberMoldSetup.document(new machinekit.hobbing.MineralCastBed());
  if(materia.assembly.AssemblyFrames.transformVector(setupDocument.setup.sourceToMold,0,0,1).z>-1+1e-7)
   throw 'Hobber setup must put the source mounting faces toward the bed';
  var columnSource=new machinekit.hobbing.MineralCastColumn();
  var columnDocument=HobberMoldSetup.document(columnSource,HobberMoldSetup.HobberMoldCasting.Column);
  if(columnDocument.input.id!="hobber-column" || columnDocument.setup.id!="column-rail-faces-down")
   throw 'Hobber column must have its own source identity in the shared setup';
  if(materia.assembly.AssemblyFrames.transformVector(columnDocument.setup.sourceToMold,0,1,0).z>-1+1e-7)
   throw 'Hobber column rail faces must point toward the bed';
  var columnStrategies=materia.tooling.casting.CastingStrategies.resolve(columnDocument);
  for(feature in columnDocument.input.features)
   if(feature.kind==materia.casting.CastingFeature.CastingFeatureKind.Lightening && feature.openings.filter(o->o.kind==BodyFace).length>0){
    var strategy=columnStrategies.filter(s->s.featureId==feature.id);
    if(strategy.length!=1 || strategy[0].withdrawal==null)
     throw 'A body-open EPS core must derive its access direction from its declared opening: '+feature.id;
   }
  var columnTopology=materia.tooling.ToolingCodec.copy(columnDocument);
  columnTopology.setup.sourceToMold=materia.assembly.AssemblyFrames.identity();
  var columnVoids=new Map<String,Array<materia.casting.Box3>>(),allColumnVoids:Array<materia.casting.Box3>=[];
  for(feature in columnTopology.input.features)if(feature.kind!=materia.casting.CastingFeature.CastingFeatureKind.InsertRecess){
   var boxes=materia.tooling.casting.MoldRegions.boxes(materia.tooling.casting.MoldRegions.feature(columnTopology,feature.id));
   if(boxes==null)throw 'Hobber column source hold requires exact box topology: '+feature.id;
   columnVoids.set(feature.id,boxes);allColumnVoids=allColumnVoids.concat(boxes);
  }
  var closedColumnGroups=0;
  for(group in BoxAlgebra.groups(BoxAlgebra.subtract(allColumnVoids,[]))){
   var members=columnTopology.input.features.filter(feature->{
    var boxes=columnVoids.get(feature.id);
    return boxes!=null && BoxAlgebra.volume(BoxAlgebra.subtract(boxes,group))<BoxAlgebra.volume(boxes)-1e-7;
   });
   if(members.filter(feature->feature.openings.filter(o->o.kind==BodyFace).length>0).length==0){
    closedColumnGroups++;
    Sys.println('Hobber column source hold: '+BoxAlgebra.volume(group)+' mm3 enclosed void; '+[for(feature in members)feature.id].join(', '));
   }
  }
  if(closedColumnGroups!=2 || columnTopology.input.precastStruts.length!=0)
   throw 'Revisit the recorded hobber column support hold when its source changes';
  // Source-coordinate contact and access assertions have an explicit upright
  // fixture; the production orientation and inverted contacts are checked too.
  var document=materia.tooling.ToolingCodec.copy(setupDocument);
  document.setup.sourceToMold=materia.assembly.AssemblyFrames.identity();
  var input=document.input;
  if(document.recipe.coreWireAxis!=materia.tooling.ToolingRecipe.CoreWireAxis.Z)
   throw 'Hobber EPS must be sectioned around the vertical source struts';
  var gapCore=materia.tooling.casting.MoldCoreLayers.box(
   {x:20,y:20,z:10,width:100,depth:100,height:20},'bed-gap',document.recipe,
   materia.tooling.casting.CoreSupport.CoreSupportKind.RoutedSocket);
  var bed={panel:materia.tooling.panel.PanelJoints.blank('gap-bed',160,160,document.recipe.panel,
   document.recipe.routerDiameter,document.recipe.minimumLigament),
   pose:materia.assembly.AssemblyFrames.translation(0,0,-document.recipe.panel.measuredThickness),
   withdrawal:{x:0.0,y:0.0,z:-1.0}};
  var foam=materia.tooling.casting.MoldCores.singleBox(gapCore.foamEnvelope,gapCore.id);
  materia.tooling.casting.CoreSupports.socketFoot(gapCore,
   {x:foam.x,y:foam.y,width:foam.width,depth:foam.depth},
   {x:20.0,y:20.0,width:100.0,depth:100.0},0,10,document.recipe,bed,[]);
  var foot=gapCore.support.foot;
  if(foot==null || foot.foam.z>=0 || Math.abs(foot.foam.z+foot.foam.height-10+document.recipe.adhesiveLineThickness)>1e-7)
   throw 'EPS locating foot must span the dry bed gap and stop one glue line below the foam';
  var occupied=materia.tooling.casting.MoldRegions.boxes(materia.tooling.casting.MoldCores.occupied(gapCore));
  var glue={x:foot.foam.x,y:foot.foam.y,z:foot.foam.z+foot.foam.height,
   width:foot.foam.width,depth:foot.foam.depth,height:document.recipe.adhesiveLineThickness};
  if(occupied==null || BoxAlgebra.volume(BoxAlgebra.subtract([glue],occupied))>1e-7)
   throw 'Backing must leave the declared foot-to-core glue line free of wood';
  var jobs=materia.tooling.casting.MoldCores.cuttingJobs(gapCore);
  if(jobs.length!=gapCore.layers.length+1 || jobs[jobs.length-1].cutting.block.height<10)
   throw 'The separate gap-spanning locating foot must be supplied in the wire-cut stock jobs';
  gapCore.support.assemblyByLayer=true;gapCore.support.breakoutAfterLift=true;
  var half=(foam.width-document.recipe.adhesiveLineThickness)/2;
  gapCore.layers=[for(right in [false,true])materia.tooling.casting.CoreSupports.cutting({
   x:foam.x+(right?half+document.recipe.adhesiveLineThickness:0),y:foam.y,z:foam.z,
   width:half,depth:foam.depth,height:foam.height},document.recipe.coreWireAxis)];
  var baseMembers=materia.tooling.casting.MoldCores.baseAssemblyMembers(gapCore);
  if(baseMembers.length!=2 || baseMembers.indexOf(gapCore.id)<0
   || materia.tooling.casting.MoldCores.assemblyMembers(gapCore).length!=3)
   throw 'A locating foot spanning two stock tiles must prebond both actual contacts';
  var topCore=materia.tooling.casting.MoldCoreLayers.box(
   {x:0,y:0,z:0,width:60,depth:60,height:50},'layered-top',document.recipe,
   materia.tooling.casting.CoreSupport.CoreSupportKind.Bridge);
  var topFoam=materia.tooling.casting.MoldCores.singleBox(topCore.foamEnvelope,topCore.id);
  var halfHeight=(topFoam.height-document.recipe.adhesiveLineThickness)/2;
  topCore.layers=[for(upper in [false,true])materia.tooling.casting.CoreSupports.cutting({
   x:topFoam.x,y:topFoam.y,z:topFoam.z+(upper?halfHeight+document.recipe.adhesiveLineThickness:0),
   width:topFoam.width,depth:topFoam.depth,height:halfHeight},document.recipe.coreWireAxis)];
  topCore.support.assemblyByLayer=true;
  if(materia.tooling.casting.MoldCores.assemblyAnchor(topCore)!=1
   || materia.tooling.casting.MoldCores.bridgeAssemblyMembers(topCore).join(',')!=topCore.id)
   throw 'Only the actual carrier-facing stock preassembles with a bridge';
  var allocations=materia.tooling.casting.PrecastCoreSupports.allocations(document);
  if(allocations.length!=2)throw 'Expected two complete retained EPS networks, got '+allocations.length;
  var inverted=materia.tooling.ToolingCodec.copy(document);
  inverted.setup.sourceToMold=materia.assembly.AssemblyFrames.fromRotationMatrix(0,0,800,[1,0,0,0,-1,0,0,0,-1]);
  var invertedCores=materia.tooling.casting.PrecastCoreSupports.allocations(inverted);
  for(core in invertedCores)for(contact in core.contacts){
   var expected=contact.contact==Upper ? '-lower-' : '-upper-';
   if(contact.strutId.indexOf(expected)<0)throw 'Mold orientation must select the actual upper/lower strut contacts';
   var source=input.precastStruts.filter(strut->strut.id==contact.strutId)[0];
   var plane=source.contact==Upper ? source.box.z : source.box.z+source.box.height;
   if(Math.abs(contact.centroid.z-(800-plane))>1e-7)throw 'Precast contact coordinates must follow sourceToMold';
  }
  var strutFeatures=[for(allocation in allocations)for(id in allocation.featureIds)id];
  var access=materia.tooling.casting.EPSAccessNetworks.plan(document,
   materia.tooling.casting.CastingStrategies.resolve(document),strutFeatures);
  var cable=access.filter(network->network.featureIds.indexOf('cable-route')>=0);
  if(cable.length!=1 || cable[0].kind!=materia.tooling.casting.CoreSupport.CoreSupportKind.RoutedSocket
   || cable[0].receiverFeatureId!='foot1-stem')
   throw 'The cable route must share its actual connected foot-stem opening, without a new source breakout';
  Sys.println('EPS access networks: '+access.length+' units; cable route uses '+cable[0].receiverFeatureId);
  for(allocation in allocations){
   if(allocation.contacts.length!=6)throw 'Each retained EPS network needs six source contacts';
   if(allocation.contacts.filter(contact->contact.areaMm2<=0).length!=0)throw 'Empty precast bearing contact';
   var volume=0.0,mx=0.0,my=0.0,mz=0.0;
   for(box in allocation.sourceBoxes){
    var v=box.width*box.depth*box.height;
    volume+=v;mx+=v*(box.x+box.width/2);my+=v*(box.y+box.depth/2);mz+=v*(box.z+box.height/2);
   }
   var centre={x:mx/volume,y:my/volume,z:mz/volume};
   var reactions=materia.tooling.casting.PrecastCoreSupports.reactions(allocation.contacts,centre,1000);
   var force=0.0,momentX=0.0,momentY=0.0;
   for(reaction in reactions){
    var contact=allocation.contacts.filter(contact->contact.strutId==reaction.strutId)[0];
    force+=reaction.forceN;momentX+=reaction.forceN*contact.centroid.y;momentY+=reaction.forceN*contact.centroid.x;
   }
   if(Math.abs(force-1000)>1e-7 || Math.abs(momentX-1000*centre.y)>1e-7 || Math.abs(momentY-1000*centre.x)>1e-7)
    throw 'Precast reactions do not balance core uplift and overturning moments';
  }
  Sys.println('Hobber retained EPS: two exact enclosed networks with six source contacts each');
  var drive=allocations.filter(allocation->allocation.contacts.filter(contact->contact.areaMm2==2000).length>0);
  if(drive.length!=1 || drive[0].contacts.filter(contact->contact.areaMm2==2000).length!=6)
   throw 'Drive strut contact must use the clipped 50 x 40 face, not the full 50 x 50 stock';
  var sample=drive[0].contacts;
  var outside=false;
  try materia.tooling.casting.PrecastCoreSupports.reactions(sample,{x:10000,y:10000,z:0},1000)
  catch(error:Dynamic){
   outside=Std.string(error).indexOf('outside precast support triangle')>=0;
  }
  if(!outside)throw 'An unsupported uplift centroid must be rejected';
  var sourceFingerprint=materia.tooling.ToolingCodec.inputFingerprint(input);
  var changed=materia.tooling.ToolingCodec.copyInput(input);
  changed.precastStruts[0].box.width+=1;
  if(materia.tooling.ToolingCodec.inputFingerprint(changed)==sourceFingerprint)
   throw 'Source strut dimensions must enter the source fingerprint';
  for(outside in [false,true]){
   var invalid=materia.tooling.ToolingCodec.copy(document);
   if(outside)invalid.input.precastStruts[0].box.x=10000;
   else invalid.input.precastStruts[0].box.height+=1;
   invalid.setup.sourceFingerprint=materia.tooling.ToolingCodec.inputFingerprint(invalid.input);
   var expected=outside ? 'leaves the source casting envelope' : 'crosses a source cavity or steel insert';
   var message='';
   try materia.tooling.casting.PrecastCoreSupports.allocations(invalid) catch(error:Dynamic)message=Std.string(error);
   if(message.indexOf(expected)<0 || message.indexOf(invalid.input.precastStruts[0].id)<0)
    throw 'Expected strut containment rejection '+expected+', got '+message;
  }
  var bridgeCores:Array<materia.tooling.casting.CastingMoldPlan.CastingMoldCore>=[];
  for(allocation in allocations){
   var support=materia.tooling.casting.CoreSupports.specification(
    materia.tooling.casting.CoreSupport.CoreSupportKind.PrecastStruts,document.recipe);
   support.precastContacts=allocation.contacts;
   bridgeCores.push({id:allocation.id,featureId:allocation.featureIds[0],cavity:allocation.region,
    foamEnvelope:allocation.region,layers:[],coveringThickness:document.recipe.linerThickness,support:support});
  }
  var heads=materia.tooling.casting.PrecastCoreSupports.pressureFootprints(document,bridgeCores);
  var ownership:Array<materia.tooling.casting.CastingMoldPlan.FeatureTooling>=[];
  for(index in 0...allocations.length)for(id in allocations[index].featureIds)
   ownership.push({featureId:id,kind:EPSFeature,memberIds:[bridgeCores[index].id]});
  var units=materia.tooling.casting.CastingMoldPlanner.coreUnits(input,bridgeCores,ownership);
  var names=[for(unit in units)for(id in unit.featureIds)id];
  for(feature in ownership)if(names.indexOf(feature.featureId)<0)
   throw 'Retained EPS core units lost source fragment '+feature.featureId;
  var panels:Array<materia.tooling.panel.PanelAssemblyPlan.PanelPlacement>=[];
  var hardware:Array<materia.tooling.casting.CoreSupport.SupportHardware>=[];
  var cavity=materia.tooling.casting.MoldRegions.exactBounds(materia.tooling.casting.MoldRegions.envelope(document),input.id);
  materia.tooling.casting.CoreSupports.sharedBridges(cavity,document.recipe,bridgeCores,panels,hardware,
   materia.tooling.casting.MoldPanels.RaisedWalls.EastWest,heads);
  if(panels.length!=10 || hardware.length!=8)throw 'Two retained core bridges require two rails, two beams, six bearing pads and eight fixings';
  for(core in bridgeCores){
   if(core.support.bondedPanelIds.length!=4 || core.support.panelIds.length!=6)
    throw 'Each precast bridge has three dry bearing pads bonded to its beam';
   if(materia.tooling.casting.MoldCores.removal(core)!=materia.tooling.casting.CastingMoldPlan.CoreRemoval.LeaveInCasting)
    throw 'Enclosed EPS must remain in the casting';
  }
  var channel=input.features.filter(feature->feature.id=='X-drive-channel')[0];
  var channelBoxes=materia.tooling.casting.MoldRegions.boxes(materia.tooling.casting.MoldRegions.feature(document,channel.id));
  if(channelBoxes==null)throw 'Expected exact channel boxes in the pad-clearance fixture';
  for(core in bridgeCores)for(id in materia.tooling.casting.MoldCores.spreaderIds(core)){
   var pad=panels.filter(member->member.panel.id==id)[0];
   var outline=materia.casting.CastingProfileGeometry.bounds(materia.tooling.panel.PanelContours.outside(pad.panel));
   var footprint={x:pad.pose.x+outline.x,y:pad.pose.y+outline.y,z:0.0,width:outline.width,depth:outline.height,height:800.0};
   for(box in channelBoxes)if(BoxAlgebra.intersection(footprint,box)!=null)
    throw 'Precast bearing pad crosses the source drive-channel carrier footprint: '+id;
  }
  var closeHeads=new Map<String,Array<materia.casting.Box3>>();
  for(core in bridgeCores){
   var pads=heads.get(core.id);
   if(pads==null)throw 'Missing bridge fixture heads';
   var shift=core.id==drive[0].id;
   closeHeads.set(core.id,[for(pad in pads){x:pad.x-(shift?100:0),y:pad.y-(shift?200:0),
    z:pad.z,width:pad.width,depth:pad.depth,height:pad.height}]);
  }
  var nearbyCores=[for(core in bridgeCores){id:core.id,featureId:core.featureId,cavity:core.cavity,
   foamEnvelope:core.foamEnvelope,layers:core.layers,coveringThickness:core.coveringThickness,
   support:materia.tooling.casting.CoreSupports.specification(
    materia.tooling.casting.CoreSupport.CoreSupportKind.PrecastStruts,document.recipe)}];
  for(index in 0...nearbyCores.length)nearbyCores[index].support.precastContacts=bridgeCores[index].support.precastContacts;
  var closePanels:Array<materia.tooling.panel.PanelAssemblyPlan.PanelPlacement>=[];
  var closeHardware:Array<materia.tooling.casting.CoreSupport.SupportHardware>=[];
  materia.tooling.casting.CoreSupports.sharedBridges(cavity,document.recipe,nearbyCores,closePanels,closeHardware,
   materia.tooling.casting.MoldPanels.RaisedWalls.EastWest,closeHeads);
  if(closePanels.length!=9 || closeHardware.length!=4
   || materia.tooling.casting.MoldCores.bridgeId(nearbyCores[0])!=materia.tooling.casting.MoldCores.bridgeId(nearbyCores[1]))
   throw 'Different row centres with overlapping beam bands must share one physical beam';
  Sys.println('Hobber bridges: six dry bearing pads on two shared beams; exact source contact and uplift checks pass');
  var missing:Array<String>=[];
  for(insert in input.inserts){
   var stock=materia.assembly.AssemblyFrames.compose(document.setup.sourceToMold,insert.pose);
   var face=materia.assembly.AssemblyFrames.compose(stock,materia.assembly.AssemblyFrames.translation(
    0,0,materia.casting.InsertSpec.InsertBodyCodec.height(insert.body)));
   var holes=materia.tooling.casting.InsertFixturePlanner.availableHoles(insert);
   var selected=holes.filter(hole->materia.tooling.casting.InsertFixturePlanner.select(
    hole,stock,face,document.recipe.hardware)!=null);
   if(selected.length==0)missing.push(insert.id);
  }
  missing.sort(Reflect.compare);
  Sys.println('Source inserts needing a declared fixture thread (not invented by tooling): '+missing.join(', '));
  if(Sys.args().indexOf('--networks')>=0){
   var voids:Array<materia.casting.Box3>=[],steel:Array<materia.casting.Box3>=[];
   var featureBoxes=new Map<String,Array<materia.casting.Box3>>();
   for(feature in input.features)if(feature.kind!=InsertRecess){
    var boxes=materia.tooling.casting.MoldRegions.boxes(materia.tooling.casting.MoldRegions.feature(document,feature.id));
    if(boxes==null)throw 'Non-box source void network: '+feature.id;
    featureBoxes.set(feature.id,boxes);voids=voids.concat(boxes);
   }
   for(insert in input.inserts){
    var boxes=materia.tooling.casting.MoldRegions.boxes(materia.tooling.casting.MoldRegions.insert(document,insert.id));
    if(boxes!=null)steel=steel.concat(boxes);
   }
   for(group in BoxAlgebra.groups(BoxAlgebra.subtract(voids,steel))){
    var ids:Array<String>=[],openings:Array<String>=[];
    for(feature in input.features){
     var boxes=featureBoxes.get(feature.id);
     if(boxes==null || BoxAlgebra.volume(BoxAlgebra.subtract(boxes,group))>=BoxAlgebra.volume(boxes)-1e-7)continue;
     ids.push(feature.id);
     for(opening in feature.openings)if(opening.kind==BodyFace)openings.push(feature.id+':'+opening.target);
    }
    Sys.println('Void network '+BoxAlgebra.volume(group)+' mm3: '+ids.join(', ')+'; body openings '+openings.join(', '));
   }
  }
  if(Sys.args().indexOf('--native')>=0 || Sys.args().indexOf('--native-reservations')>=0 || Sys.args().indexOf('--native-column')>=0 || Sys.args().indexOf('--native-support-records')>=0){
   materia.tooling.casting.CastingMoldPlanner.setGeometry(new machinekit.tooling.NativeCastingMoldKernel());
   var sample=materia.tooling.casting.MoldRegions.box({x:0,y:0,z:0,width:40,depth:40,height:20});
   var boxCut=materia.tooling.casting.MoldRegions.box({x:0,y:0,z:0,width:10,depth:10,height:20});
   var round:materia.casting.CastingRegion=Placed({solid:Cut(Cylinder(6,20)),
    pose:materia.assembly.AssemblyFrames.translation(100,100,0)});
   var reserved=materia.tooling.casting.CastingMoldPlanner.reservedDifference(sample,[boxCut,round]);
   var pieces=materia.tooling.casting.MoldRegions.boxes(reserved);
   if(pieces==null || Math.abs(BoxAlgebra.volume(pieces)-30000)>1e-7)
    throw 'Separated curved inserts must not force orthogonal reservations into native booleans';
   round=Placed({solid:Cut(Cylinder(6,20)),pose:materia.assembly.AssemblyFrames.translation(25,25,0)});
   reserved=materia.tooling.casting.CastingMoldPlanner.reservedDifference(sample,[boxCut,round]);
   if(Math.abs(materia.tooling.casting.MoldRegions.volume(reserved,'real-curved-reservation')-(30000-180*Math.PI))>1e-5)
    throw 'A genuinely intersecting curved insert must retain its exact subtraction';
   // A curved catalogue tail may be certified only by rigorous sweep bounds.
   // Public sweep construction remains strict for unsupported boundary surfaces.
   var socket=FoundationLayout.traySocket().geometry();
   var beamBlank=cadkit.modeling.Part.box(20,20,4);
   var beam=beamBlank.translated(new cadkit.modeling.Vector(0,0,80));
   beamBlank.close();
   try {
    var strictRejected=false;
    try {
     var swept=machinekit.tooling.SweptRemoval.sweep(socket,new cadkit.modeling.Vector(0,0,100));
     swept.close();
    } catch(error:Dynamic) strictRejected=Std.string(error).indexOf('Unsupported removal boundary surface')>=0;
    if(!strictRejected)throw 'The public sweep must not approximate a curved socket tail';
    var motion=machinekit.tooling.SweptRemoval.analyze([
     {id:'socket',part:socket,direction:new cadkit.modeling.Vector(0,0,1)},
     {id:'beam',part:beam,direction:new cadkit.modeling.Vector(1,0,0)}]);
    if(motion.blocking.length!=1 || motion.blocking[0].movingId!='socket' || motion.blocking[0].blockerId!='beam')
     throw 'Supported boundary sweeps must detect an intermediate socket/beam collision';
    var holeBlank=cadkit.modeling.Part.box(12,10,6);
    var hole=holeBlank.translated(new cadkit.modeling.Vector(0,0,79));
    holeBlank.close();
    var ring=beam.subtract(hole);
    hole.close();
    try {
     var clear=machinekit.tooling.SweptRemoval.analyze([
      {id:'socket',part:socket,direction:new cadkit.modeling.Vector(0,0,1)},
      {id:'ring',part:ring,direction:new cadkit.modeling.Vector(1,0,0)}]);
     if(clear.blocking.length!=0)throw 'Enclosing swept box must certify clearance through the ring';
     ring.close();
    } catch(error:Dynamic) {ring.close();throw error;}
    var uncertainBlank=cadkit.modeling.Part.box(0.2,0.2,1);
    var uncertain=uncertainBlank.translated(new cadkit.modeling.Vector(0,0,80));
    var unsupported=cadkit.modeling.Part.sphere(4);
    uncertainBlank.close();
    try {
     var rejected=false;
     try machinekit.tooling.SweptRemoval.analyze([
      {id:'unsupported',part:unsupported,direction:new cadkit.modeling.Vector(0,0,1)},
      {id:'uncertain',part:uncertain,direction:new cadkit.modeling.Vector(1,0,0)}])
     catch(error:Dynamic) rejected=Std.string(error).indexOf('Unsupported removal relation unsupported/uncertain')>=0;
     if(!rejected)throw 'An uncertified curved sweep must remain Unsupported with both member IDs';
     uncertain.close(); unsupported.close();
    } catch(error:Dynamic) {uncertain.close();unsupported.close();throw error;}
    socket.close(); beam.close();
   } catch(error:Dynamic) {socket.close();beam.close();throw error;}
   if(Sys.args().indexOf('--native-reservations')>=0){
    Sys.println('Reservation geometry: exact box subtraction and real curved-intersection checks pass');
    return;
   }
   if(Sys.args().indexOf('--native-column')>=0){
    var rejected=false;
    try machinekit.tooling.CastingMoldCad.plan(columnDocument,HobberMoldSetup.options(columnDocument.input))
    catch(error:Dynamic) rejected=Std.string(error).indexOf('EPS core has no declared access or cast-in supports: core-')>=0;
    if(!rejected)throw 'An enclosed column EPS void without source supports must not receive an invented fixture';
    Sys.println('Hobber column source hold verified: planner rejects unsupported enclosed EPS; no mold kit claimed');
    return;
   }
   var planned=machinekit.tooling.CastingMoldCad.plan(setupDocument,HobberMoldSetup.options(input));
   if(Sys.args().indexOf('--native-support-records')>=0){
    HobberMoldChecks.supportRecords(planned);
    var ordinary=materia.tooling.casting.MoldCoreLayers.box({x:0,y:0,z:0,width:100,depth:100,height:20},
     'ordinary-core',planned.document.recipe,materia.tooling.casting.CoreSupport.CoreSupportKind.Bridge);
    var rows=machinekit.tooling.ToolingFabricationExport.coreSupportRecords([ordinary],[]);
    var encoded:Array<Dynamic>=cast haxe.Json.parse(materia.tooling.ToolingCodec.canonical(rows));
    if(encoded[0].withdrawal.z!=1 || encoded[0].withdrawal.x!=0 || encoded[0].withdrawal.y!=0)
     throw 'An intact bridge core must retain its actual withdrawal direction';
    Sys.println('EPS shop support records: retained/breakout units omit an intact pull; ordinary bridge direction preserved');
    return;
   }
   var driveFixture=planned.fixtures.filter(fixture->fixture.insertId=='driveSeat');
   if(driveFixture.length!=1 || materia.tooling.casting.CastingMoldPlan.MoldFixtures.withdrawal(driveFixture[0]).z>-1+1e-7)
    throw 'The channel permits a straight bed withdrawal for the drive-seat datum';
   for(volume in materia.tooling.casting.CastingMoldPlan.MoldFixtures.volumes(driveFixture[0]))for(piece in volume.pieces)
    if(piece.x+piece.width>planned.cavity.x+planned.cavity.width+1e-7)
     throw 'The drive-seat datum must not cross the closed east wall';
   Sys.println('Hobber native plan: '+planned.cores.length+' cores, '+planned.formers.length+' formers, '+planned.fixtures.length+' insert fixtures');
   var review=machinekit.tooling.CastingMoldCad.review(planned);
   HobberMoldChecks.run(planned,review);
  }
 }
}
