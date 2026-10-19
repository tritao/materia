package machinekit.motion;

import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.component.*;
import machinekit.component.ComponentRecipeSupport as P;
import machinekit.motion.VallderSupportData.ScrewSupportReference;
import machinekit.assembly.RbkMotorMount;
import materia.assembly.AssemblyFrames as F;

/** Pinned complete support, including bearings/hardware. Raw shaft axis is +Z at X=Y=0.
 * BK/BF base Y=-h. FK/FF mounting flange Z=0..H. Named datums never imply preload or load qualification.
 * Existing BF12 recipe keeps its older bearing-front frame and persistence identity.
 */
class VallderScrewSupport extends MachineComponent {
 static var recipe:Null<ComponentType>;
 public final spec:ScrewSupportReference;
 public function new(model:String="BK12") {
  if(VallderSupportData.models().indexOf(model)<0)throw "Support needs interface review: "+model;
  var s=VallderSupportData.get(model);
  super("VALLDER-"+model+"-CAD-"+s.sha256.substr(0,12),"Vallder "+model+" with included bearing/hardware","unspecified");
  spec=s;lockManufacturingMaterial("unspecified");
  addConnector("shaft",Axis,Solids.axial(0,0,0));
  for(datum in Vallder15SupportSeats.datums(model,s.sha256))addConnector(datum.name,Axis,Solids.axial(0,0,datum.z));
  if(s.base) {
   addConnector("base",Mount,F.translation(0,-s.shaftHeight,0));
  } else {
   addConnector("flangeBack",Mount,Solids.axial(0,0,0));
   addConnector("flangeFront",Mount,Solids.axial(0,0,s.height));
  }
  var index=1;
  for(p in mountingPositions()) addConnector("mount"+index++,Mount,s.base ? F.alongY(p.x,-s.shaftHeight,p.z,0,1,0) : Solids.axial(p.x,p.y,s.height));
 }
 public function mountingPositions():Array<Vector> {
  var s=spec;
  if(s.base)return [for(x in [-s.pitch/2,s.pitch/2])for(z in (s.longitudinalPitch==0 ? [0.0] : [-s.longitudinalPitch/2,s.longitudinalPitch/2]))new Vector(x,0,z)];
  var offset=s.pitch/Math.sqrt(8);
  return [for(x in [-offset,offset])for(y in [-offset,offset])new Vector(x,y,0)];
 }
 override public function hasGeometry():Bool return true;
 override public function geometry(detail:ComponentDetail=Preview):Part {
  var component=new SupplierCadComponent(RbkMotorMount.assetDirectory()+"/cad/assets/"+spec.sha256+".step",spec.sha256,designation,description);
  try {var result=component.geometry(detail);component.close();return result;}catch(e:Dynamic){component.close();throw e;}
 }
 public static function referenceRecipeType():ComponentType {
  if(recipe==null)recipe=new ComponentType("machinekit.motion.vallder-screw-support",[P.choice("model",VallderSupportData.models(),"BK12")],v -> new VallderScrewSupport(v.token("model")),true,false,null,false);
  return recipe;
 }
 override public function componentType():Null<ComponentType> return referenceRecipeType();
 override public function values():ComponentValues return new ComponentValues().setToken("model",spec.model);
}
