package materia.tooling.casting;

import materia.casting.Box3;
import materia.casting.BoxAlgebra;
import materia.casting.CastingFeature;
import materia.casting.CastingRegion;
import materia.casting.CastingProfileGeometry as G;
import materia.casting.CastingSection.CastingSectionContour;
import materia.assembly.AssemblyFrames as F;
import materia.tooling.ToolingDocument;
import materia.tooling.ToolingCodec;
import materia.tooling.casting.CastingSetup.CastingFeatureStrategy;
import materia.tooling.casting.CoreSupport.CoreSupportKind;

typedef EPSAccessNetwork = {
 var id:String;
 var featureIds:Array<String>;
 var cavity:CastingRegion;
 var kind:CoreSupportKind;
 var receiverFeatureId:String;
 var receiver:Box3;
}

/** Connected EPS cuts can share a declared opening. A service branch does not
 * need a new breakout merely because its own cut is inside the body. This box
 * path never substitutes a stock envelope for curved material.
 */
class EPSAccessNetworks {
 static inline var EPS=1e-7;

 /** Existing routed-foot construction, sized from the actual receiver section. */
 public static function add(p:CastingMoldPlan,network:EPSAccessNetwork):CastingMoldPlan.CastingMoldCore {
  var core=MoldCoreLayers.network(network.cavity,network.featureIds[0],network.id,p.document.recipe,network.kind);
  if(network.kind==RoutedSocket){
   // The first certified constant-profile band defines the real foam
   // contact. A protruding insert can put the bed below the body opening;
   // the locating foot spans that dry backing gap without moving the source.
   var kernel=CastingMoldPlanner.geometry;
   if(kernel==null)throw "EPS socket requires the exact section kernel: "+core.id;
   var foam=MoldRegions.exactBounds(core.foamEnvelope,core.id);
   if(Math.abs(foam.z-network.receiver.z)>EPS)throw "EPS socket misses its declared body opening: "+core.id;
   var local=MoldRegions.moved(core.foamEnvelope,F.translation(-foam.x,-foam.y,-foam.z));
   var bands=kernel.prismaticBands(local,foam.width,foam.depth,foam.height,core.id);
   var contact=bands.filter(band->Math.abs(band.bottomMm)<=EPS);
   if(contact.length!=1)throw "EPS socket has no certified bed contact band: "+core.id;
   var rows=[for(row in contact[0].contours){depth:row.depth,
    contour:materia.tooling.panel.PanelContours.placed(row.contour,0,foam.x,foam.y)}];
   var bounds=contactRectangle(rows,network.receiver,core.id,3*p.document.recipe.routerDiameter+2*p.document.recipe.minimumLigament);
   var records:Array<CastingMoldPlan.PenetrationPlan>=[];
   CoreSupports.socketFoot(core,{x:bounds.x,y:bounds.y,width:bounds.width,depth:bounds.depth},
    {x:network.receiver.x,y:network.receiver.y,width:network.receiver.width,depth:network.receiver.depth},
    p.cavity.z,foam.z,p.document.recipe,p.panels.members[0],records);
   for(record in records)p.openings.push(CastingMoldPlan.MoldOpenings.fromRecord(record));
  }
  p.cores.push(core);
  for(id in network.featureIds)p.features.push({featureId:id,kind:EPSFeature,memberIds:[core.id]});
  return core;
 }

 /** A locating foot needs a real rectangular patch, not the bounds of an
  * entire branched section. Grid cells are certified against exact line/arc
  * contours; hole bounds conservatively exclude contact without changing foam.
  */
 public static function contactRectangle(rows:Array<CastingSectionContour>,receiver:Box3,id:String,minimumSide:Float=0):Box3 {
  var candidates:Array<Box3>=[];
  for(outer in rows)if(outer.depth%2==0){
   var bounds=G.bounds(outer.contour);
   var domain=BoxAlgebra.intersection(receiver,{x:bounds.x,y:bounds.y,z:receiver.z,
    width:bounds.width,depth:bounds.height,height:receiver.height});
   if(domain==null)continue;
   var holes=rows.filter(row->row.depth==outer.depth+1 && G.inside(outer.contour,
    row.contour[0].start.x,row.contour[0].start.y));
   var xs=[domain.x,domain.x+domain.width],ys=[domain.y,domain.y+domain.depth];
   for(row in [outer].concat(holes))for(edge in row.contour)for(point in G.extrema(edge)){
    if(point.x>domain.x && point.x<domain.x+domain.width)xs.push(point.x);
    if(point.y>domain.y && point.y<domain.y+domain.depth)ys.push(point.y);
   }
   function unique(values:Array<Float>):Array<Float>{
    values.sort(Reflect.compare);var out:Array<Float>=[];
    for(value in values)if(out.length==0 || value-out[out.length-1]>EPS)out.push(value);
    return out;
   }
   xs=unique(xs);ys=unique(ys);
   var cells:Array<Box3>=[];
   for(x in 0...xs.length-1)for(y in 0...ys.length-1){
    var width=xs[x+1]-xs[x],depth=ys[y+1]-ys[y];
    if(G.containsRect(outer.contour,xs[x],ys[y],width,depth))cells.push({
     x:xs[x],y:ys[y],z:receiver.z,width:width,depth:depth,height:receiver.height});
   }
   var exclusions=[for(hole in holes){
    var b=G.bounds(hole.contour);
    {x:b.x,y:b.y,z:receiver.z,width:b.width,depth:b.height,height:receiver.height};
   }];
   candidates=candidates.concat(BoxAlgebra.coalesce(BoxAlgebra.subtract(cells,exclusions)));
  }
  candidates=candidates.filter(box->box.width>minimumSide+EPS && box.depth>minimumSide+EPS);
  candidates.sort((a,b)->a.width*a.depth!=b.width*b.depth
   ? Reflect.compare(b.width*b.depth,a.width*a.depth)
   : a.x!=b.x ? Reflect.compare(a.x,b.x) : Reflect.compare(a.y,b.y));
  if(candidates.length==0)throw "EPS socket has no certified rectangular contact: "+id;
  return candidates[0];
 }

 public static function plan(d:ToolingDocument,strategies:Array<CastingFeatureStrategy>,
  excluded:Array<String>,?reserved:Array<Box3>):Array<EPSAccessNetwork> {
  var ids=[for(strategy in strategies)if(strategy.strategy==SacrificialEPS
   && excluded.indexOf(strategy.featureId)<0)strategy.featureId];
  var selected=d.input.features.filter(feature->ids.indexOf(feature.id)>=0);
  var seeds=selected.filter(feature->feature.openings.filter(opening->opening.kind==BodyFace).length==0);
  if(seeds.length==0)return [];
  var byFeature=new Map<String,Array<Box3>>(),voids:Array<Box3>=[];
  for(feature in selected){
   var boxes=MoldRegions.boxes(MoldRegions.feature(d,feature.id));
   if(boxes==null)throw "Unsupported non-box EPS access network: "+feature.id;
   byFeature.set(feature.id,boxes);voids=voids.concat(boxes);
  }
  var steel:Array<Box3>=[],round:Array<{id:String,region:CastingRegion}>=[];
  for(insert in d.input.inserts){
   var region=MoldRegions.insert(d,insert.id),boxes=MoldRegions.boxes(region);
   if(boxes==null)round.push({id:insert.id,region:region});
   else steel=steel.concat(boxes);
  }
  if(reserved!=null)steel=steel.concat(reserved);
  var result:Array<EPSAccessNetwork>=[];
  var bedZ=MoldRegions.exactBounds(MoldRegions.envelope(d),d.input.id).z;
  for(group in BoxAlgebra.groups(BoxAlgebra.subtract(voids,steel))){
   var members=selected.filter(feature->{
    var boxes=byFeature.get(feature.id);
    return boxes!=null && BoxAlgebra.volume(BoxAlgebra.subtract(boxes,group))<BoxAlgebra.volume(boxes)-EPS;
   });
   if(members.filter(feature->seeds.indexOf(feature)>=0).length==0)continue;
   var portals:Array<{featureId:String,kind:CoreSupportKind,box:Box3,area:Float}>=[];
   for(feature in members)for(opening in feature.openings)if(opening.kind==BodyFace){
    var source=materia.casting.CastingCavities.bodyFaceNormal(d.input.envelope,opening.target);
    var normal=F.transformVector(d.setup.sourceToMold,source.x,source.y,source.z);
    if(Math.abs(normal.z)<1-EPS)continue;
    var boxes=byFeature.get(feature.id);
    if(boxes==null)throw "Missing EPS portal geometry: "+feature.id;
    for(box in boxes){
     var bed=normal.z<0 && Math.abs(box.z-bedZ)<EPS;
     var fill=normal.z>0 && Math.abs(box.z+box.height-d.setup.fillHeightMm)<EPS;
     if(bed || fill)portals.push({featureId:feature.id,kind:fill ? Bridge : RoutedSocket,box:box,area:box.width*box.depth});
    }
   }
   if(portals.length==0)continue;
   var region=CastingRegion.Union([for(box in group)MoldRegions.box(box)]);
   for(insert in round){
    var bounds=materia.casting.CastingRegionBounds.box(insert.region);
    if(bounds!=null && group.filter(box->BoxAlgebra.intersection(box,bounds)!=null).length==0)continue;
    if(MoldRegions.overlaps(region,insert.region,EPS,insert.id))
     throw "Unsupported round insert in box EPS access topology: "+insert.id;
   }
   for(feature in members){
    var boxes=byFeature.get(feature.id);
    if(boxes==null || BoxAlgebra.volume(BoxAlgebra.subtract(boxes,steel.concat(group)))>EPS)
     throw "EPS access network splits source feature: "+feature.id;
   }
   // Prefer the largest real opening, avoiding a small stem as the load path
   // when the same unit has a full pour-face opening. IDs stabilize equal areas.
   portals.sort((a,b)->a.area==b.area ? Reflect.compare(a.featureId,b.featureId) : Reflect.compare(b.area,a.area));
   var portal=portals[0],featureIds=[for(feature in members)feature.id];
   featureIds.sort(Reflect.compare);
   var limits=[for(box in group)BoxAlgebra.limits(box)];
   limits.sort((a,b)->Reflect.compare(ToolingCodec.canonical(a),ToolingCodec.canonical(b)));
   result.push({id:"access-core-"+haxe.crypto.Sha256.encode(ToolingCodec.canonical(limits)).substr(0,20),
    featureIds:featureIds,cavity:region,kind:portal.kind,receiverFeatureId:portal.featureId,receiver:portal.box});
  }
  result.sort((a,b)->Reflect.compare(a.id,b.id));
  return result;
 }
}
