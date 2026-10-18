/** Manufacturer right-angle connector candidate, kept distinct from a qualified wrist mate. */
class WristRightAngleCandidate {
  public static function main():Void {
    var root=ReferenceAssetLocation.DIRECTORY;
    var file=root+"/cad/right-angle-candidate/889D-E5AC-2.stp";
    if(ReferenceWristInterfaces.digest(sys.io.File.getBytes(file))!="30ab9dd25f3605885a66fd69ff466d67a67fce81725dd07c7e1832100ef08ea1") throw "Changed right-angle supplier STEP";
    var directory=root+"/wrist-right-angle-candidate";
    var shape=WristPlugCandidate.inspectFile(file,directory,"Rockwell 889D-E5AC-* right-angle supplier reference");
    if(shape.subshapeCount(CadKit.ShapeKind.Solid)!=1 || shape.subshapeCount(CadKit.ShapeKind.Face)!=488) throw "Changed right-angle topology";
    // Keep the complete connector and 5.08 mm of the original cable; remove the
    // supplier illustration's long cable, label sleeve and stripped wire ends.
    var box=cadkit.modeling.Part.box(100,50,80);
    var clipping=box.shape.translate(new cadkit.modeling.Vector(25,5.226,-40).native());
    var cropped=shape.common(clipping);clipping.close();box.close();
    var normalized=WristPlugCandidate.pose(cropped,{x:0,y:0,z:0,qx:0,qy:-Math.sqrt(0.5),qz:0,qw:Math.sqrt(0.5)});cropped.close();
    var check=new cadkit.modeling.Part(normalized.cloneShape());
    if(!check.valid() || check.solidCount()!=1) throw "Derived right-angle connector must remain one valid solid";
    check.close();
    var mesh=normalized.tessellate(0.1);
    sys.io.File.saveBytes(directory+"/layout-plug.vertices",mesh.vertices);sys.io.File.saveBytes(directory+"/layout-plug.indices",mesh.indices);
    normalized.exportStep(directory+"/connector-with-short-stub.step");
    WristPlugCandidate.layout(root,directory,normalized,{id:"rockwell-889d-e5ac-right-angle",description:"Rockwell right-angle M12 candidate",
      externalBodyStartZMm:16.002,preferredClockDeg:180,offsetsMm:[-8.0,0.0,8.0],clocksDeg:[0.0,90.0,180.0,270.0],separationBoundMm:5,inspectMatingZone:false});
    normalized.close();shape.close();
    Sys.println("Imported supplier right-angle STEP for reference-frame inspection");
  }
}
