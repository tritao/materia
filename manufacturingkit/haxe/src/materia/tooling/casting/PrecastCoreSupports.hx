package materia.tooling.casting;

import materia.assembly.AssemblyFrames as F;
import materia.casting.Box3;
import materia.casting.BoxAlgebra;
import materia.casting.CastingFeature;
import materia.casting.CastingRegion;
import materia.casting.PrecastStrut;
import materia.tooling.ToolingDocument;

typedef PrecastCoreAllocation = {
 var id:String;
 var featureIds:Array<String>;
 var sourceBoxes:Array<Box3>;
 var region:CastingRegion;
 var contacts:Array<materia.tooling.casting.CoreSupport.PrecastCoreContact>;
}

/** Source compound struts retain complete enclosed void networks. Box topology
 * uses the shared exact algebra; a non-box topology is never bounded into a box.
 */
class PrecastCoreSupports {
 static inline var EPS = 1e-7;

 /** Rectangular dry spreader boards cover the declared upper strut heads.
  * Their bounds define manufactured boards, never an approximation of foam.
  */
 public static function pressureFootprints(d:ToolingDocument,cores:Array<CastingMoldPlan.CastingMoldCore>):Map<String,Array<Box3>> {
  var result=new Map<String,Array<Box3>>();
  for(core in cores)if(core.support.kind==CoreSupport.CoreSupportKind.PrecastStruts){
   var heads:Array<Box3>=[];
   for(contact in core.support.precastContacts)if(contact.contact==Upper){
    var sources=d.input.precastStruts.filter(strut->strut.id==contact.strutId);
    if(sources.length!=1)throw "Unknown upper precast strut: "+contact.strutId;
    var source=sources[0].box;
    var transformed=MoldRegions.boxes(MoldRegions.moved(MoldRegions.box(source),d.setup.sourceToMold));
    if(transformed==null || transformed.length!=1)throw "Precast bridge needs axis-aligned source struts: "+contact.strutId;
    var head=transformed[0];
    if(Math.abs(head.z+head.height-d.setup.fillHeightMm)>EPS)
     throw "Upper precast strut does not meet the open fill plane: "+contact.strutId;
    heads.push(head);
   }
   if(heads.length!=3)throw "Precast bridge needs three upper strut heads: "+core.id;
   result.set(core.id,[for(head in heads){x:head.x,y:head.y,z:d.setup.fillHeightMm,
    width:head.width,depth:head.depth,height:d.recipe.panel.measuredThickness}]);
  }
  return result;
 }

 /** Three source contacts resolve force and both overturning moments. The
  * centroid is the displaced-volume centroid, never a bounding-box centre.
  * A negative reaction means the declared support triangle cannot hold it.
  */
 public static function reactions(contacts:Array<materia.tooling.casting.CoreSupport.PrecastCoreContact>,
  centroid:{x:Float,y:Float,z:Float},forceN:Float):Array<{strutId:String,forceN:Float,stressMPa:Float}> {
  var upper=contacts.filter(contact->contact.contact==Upper);
  if(upper.length!=3)throw "Precast core requires three upper bearing contacts";
  if(!Math.isFinite(forceN) || forceN<0)throw "Invalid precast core uplift";
  var a=upper[0].centroid,b=upper[1].centroid,c=upper[2].centroid;
  var determinant=(b.y-c.y)*(a.x-c.x)+(c.x-b.x)*(a.y-c.y);
  if(Math.abs(determinant)<=EPS)throw "Collinear upper precast contacts: "+[for(contact in upper)contact.strutId].join(", ");
  var first=((b.y-c.y)*(centroid.x-c.x)+(c.x-b.x)*(centroid.y-c.y))/determinant;
  var second=((c.y-a.y)*(centroid.x-c.x)+(a.x-c.x)*(centroid.y-c.y))/determinant;
  var shares=[first,second,1-first-second],result=[];
  for(index in 0...3){
   var contact=upper[index],share=shares[index];
   if(share < -EPS)throw "Core uplift lies outside precast support triangle: "+contact.strutId;
   if(!Math.isFinite(contact.areaMm2) || contact.areaMm2<=0)throw "Empty precast bearing contact: "+contact.strutId;
   var load=Math.max(0,share)*forceN;
   result.push({strutId:contact.strutId,forceN:load,stressMPa:load/contact.areaMm2});
  }
  return result;
 }

 public static function contactPatch(boxes:Array<Box3>,strut:PrecastStrut)
  :Null<materia.tooling.casting.CoreSupport.PrecastCoreContact> {
  var area=0.0, mx=0.0, my=0.0, s=strut.box;
  var plane=strut.contact==Upper ? s.z : s.z+s.height;
  for(box in boxes){
   var face=strut.contact==Upper ? box.z+box.height : box.z;
   if(Math.abs(face-plane)>EPS)continue;
   var x=Math.max(box.x,s.x),y=Math.max(box.y,s.y);
   var width=Math.min(box.x+box.width,s.x+s.width)-x;
   var depth=Math.min(box.y+box.depth,s.y+s.depth)-y;
   if(width<=0 || depth<=0)continue;
   var patch=width*depth;
   area+=patch;mx+=patch*(x+width/2);my+=patch*(y+depth/2);
  }
  return area==0 ? null : {strutId:strut.id,contact:strut.contact,areaMm2:area,
   centroid:{x:mx/area,y:my/area,z:plane}};
 }

 public static function allocations(d:ToolingDocument):Array<PrecastCoreAllocation> {
  if(d.input.precastStruts.length==0)return [];
  var inverse=F.inverse(d.setup.sourceToMold),byFeature=new Map<String,Array<Box3>>(),voids:Array<Box3>=[];
  for(feature in d.input.features)if(feature.kind!=InsertRecess){
   var boxes=MoldRegions.boxes(MoldRegions.moved(MoldRegions.feature(d,feature.id),inverse));
   if(boxes==null)throw "Unsupported non-box retained-core topology: "+feature.id;
   byFeature.set(feature.id,boxes);voids=voids.concat(boxes);
  }
  var insertBoxes:Array<Box3>=[],nonBoxInserts:Array<{id:String,region:CastingRegion}>=[];
  for(insert in d.input.inserts){
   var region=MoldRegions.moved(MoldRegions.insert(d,insert.id),inverse),boxes=MoldRegions.boxes(region);
   if(boxes==null)nonBoxInserts.push({id:insert.id,region:region});
   else insertBoxes=insertBoxes.concat(boxes);
  }
  var sourceEnvelope=MoldRegions.moved(MoldRegions.envelope(d),inverse);
  for(strut in d.input.precastStruts){
   var region=MoldRegions.box(strut.box);
   if(MoldRegions.volume(CastingRegion.Difference(region,[sourceEnvelope]),strut.id)>EPS)
    throw "Precast strut leaves the source casting envelope: "+strut.id;
   if(BoxAlgebra.volume(BoxAlgebra.subtract([strut.box],voids.concat(insertBoxes)))
     < strut.box.width*strut.box.depth*strut.box.height-EPS)
    throw "Precast strut crosses a source cavity or steel insert: "+strut.id;
   for(insert in nonBoxInserts){
    var bounds=materia.casting.CastingRegionBounds.box(insert.region);
    if(bounds!=null && BoxAlgebra.intersection(strut.box,bounds)==null)continue;
    if(MoldRegions.overlaps(region,insert.region,EPS,strut.id))
     throw "Precast strut intersects source insert "+insert.id+": "+strut.id;
   }
  }
  var result:Array<PrecastCoreAllocation>=[],used=new Map<String,Bool>();
  for(group in BoxAlgebra.groups(BoxAlgebra.subtract(voids,insertBoxes))){
   var contacts:Array<materia.tooling.casting.CoreSupport.PrecastCoreContact>=[];
   for(strut in d.input.precastStruts){
    var patch=contactPatch(group,strut);
    if(patch!=null)contacts.push(patch);
   }
   if(contacts.length==0)continue;
   if(contacts.filter(contact->contact.contact==Upper).length<3 || contacts.filter(contact->contact.contact==Lower).length<3)
    throw "Retained core needs three upper and three lower source contacts: "+contacts[0].strutId;
   var region=CastingRegion.Union([for(box in group)MoldRegions.box(box)]),ids:Array<String>=[];
   for(feature in d.input.features)if(byFeature.exists(feature.id)){
    var boxes=byFeature.get(feature.id);
    if(boxes==null)throw "Missing retained-core source boxes: "+feature.id;
    if(BoxAlgebra.volume(BoxAlgebra.subtract(boxes,group))>=BoxAlgebra.volume(boxes)-EPS)continue;
    if(feature.openings.filter(opening->opening.kind==BodyFace).length>0)
     throw "Retained source struts touch a face-open void network: "+feature.id;
    var remaining=BoxAlgebra.subtract(boxes,insertBoxes.concat(group));
    if(BoxAlgebra.volume(remaining)>EPS)
     throw "Unsupported retained-core boundary splits source feature: "+feature.id;
    ids.push(feature.id);
   }
   ids.sort(Reflect.compare);
   if(ids.length==0)throw "Retained core has no source feature";
   for(contact in contacts){
    var strut=d.input.precastStruts.filter(strut->strut.id==contact.strutId)[0];
    if(d.input.features.filter(feature->ids.indexOf(feature.id)>=0
      && (feature.id==strut.coreFeatureId || feature.derivedFrom==strut.coreFeatureId)).length==0)
     throw "Precast strut contacts a different source core: "+strut.id;
    if(used.exists(strut.id))throw "Precast strut belongs to multiple retained cores: "+strut.id;
    used.set(strut.id,true);
   }
   for(insert in nonBoxInserts){
    // Bounds prove separation only. An overlapping broad phase still requires
    // the exact solid intersection; it never replaces round insert geometry.
    var bounds=materia.casting.CastingRegionBounds.box(insert.region);
    if(bounds!=null && group.filter(box->BoxAlgebra.intersection(box,bounds)!=null).length==0)continue;
    if(MoldRegions.overlaps(region,insert.region,EPS,insert.id))
     throw "Unsupported non-box insert in retained-core topology: "+insert.id;
   }
   var records=[for(box in group)BoxAlgebra.limits(box)];
   records.sort((a,b)->Reflect.compare(materia.tooling.ToolingCodec.canonical(a),materia.tooling.ToolingCodec.canonical(b)));
   var id="retained-core-"+haxe.crypto.Sha256.encode(materia.tooling.ToolingCodec.canonical(records)).substr(0,20);
   var vertical=F.transformVector(d.setup.sourceToMold,0,0,1);
   if(Math.abs(vertical.z)<1-EPS)
    throw "Unsupported non-vertical precast strut orientation: "+contacts[0].strutId;
   var moldContacts=[for(contact in contacts){
    strutId:contact.strutId,
    contact:vertical.z>0 ? contact.contact : contact.contact==Upper ? Lower : Upper,
    areaMm2:contact.areaMm2,
    centroid:F.transformPoint(d.setup.sourceToMold,contact.centroid.x,contact.centroid.y,contact.centroid.z)
   }];
   result.push({id:id,featureIds:ids,sourceBoxes:group,
    region:MoldRegions.moved(region,d.setup.sourceToMold),contacts:moldContacts});
  }
  for(strut in d.input.precastStruts)if(!used.exists(strut.id))throw "Precast strut does not contact an enclosed core: "+strut.id;
  result.sort((a,b)->Reflect.compare(a.id,b.id));
  return result;
 }
}
