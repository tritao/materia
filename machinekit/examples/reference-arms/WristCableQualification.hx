import cadkit.Shape;

/** Exact supplier-surface audit. Does not manufacture a cable fit from a socket observation. */
class WristCableQualification {
  static function vector(v:CadKit.Vec3):Array<Float> return [v.get_x(),v.get_y(),v.get_z()];
  public static function main():Void {
    var root=ReferenceAssetLocation.DIRECTORY;
    var robot=new ReferenceRobot(root+"/fairino5_v6"),profile=robot.wristInterfaces[0];
    var first=ReferenceWristInterfaces.requireObservedGeometry(profile,"rs485");
    var path=root+"/"+first.sourceStep,bytes=sys.io.File.getBytes(path);
    if(ReferenceWristInterfaces.digest(bytes)!=first.sourceStepSha256) throw "Changed STEP; socket face IDs need review";
    var source=Shape.importStep(path);
    if(source.subshapeCount(CadKit.ShapeKind.Solid)!=37) throw "Changed supplier solid inventory";
    var solid=source.subshape(CadKit.ShapeKind.Solid,first.solidIndex);
    if(solid.subshapeCount(CadKit.ShapeKind.Face)!=566) throw "Changed terminal face inventory";
    var observations:Array<Dynamic> = [];
    for(id in ["rs485","io"]) {
      var binding=ReferenceWristInterfaces.requireObservedGeometry(profile,id);
      if(binding.solidIndex!=first.solidIndex || binding.sourceStepSha256!=first.sourceStepSha256) throw "Mixed socket source revisions";
      var port=ReferenceWristInterfaces.port(profile,id);
      var face=solid.subshape(CadKit.ShapeKind.Face,binding.insertFaceIndex);
      if(face.surfaceKind()!=CadKit.SurfaceKind.Plane) throw "Socket insert face is no longer planar";
      var centre=vector(face.center()),normal=vector(face.faceNormal()),expected=binding.insertFrameStepMm;
      if(Math.abs(centre[0]-expected.x)>0.00001 || Math.abs(centre[1]-expected.y)>0.00001 || Math.abs(centre[2]-expected.z)>0.00001)
        throw "Socket insert observation moved";
      var radii:Array<Float> = [];
      // Each contact wall is split into two half-cylinder faces in this hash-pinned STEP.
      for(i in 0...port.contactCount) {
        var contact=solid.subshape(CadKit.ShapeKind.Face,binding.contactFaces[i]);
        if(contact.surfaceKind()!=CadKit.SurfaceKind.Cylinder) throw "Contact wall is no longer cylindrical";
        radii.push(contact.faceAxis().get_radius());contact.close();
      }
      var boundaryRadii:Array<Float> = [],nonCircular=0;
      for(i in 0...face.subshapeCount(CadKit.ShapeKind.Edge)) {
        var edge=face.subshape(CadKit.ShapeKind.Edge,i);
        if(edge.curveKind()==CadKit.CurveKind.Circle) boundaryRadii.push(edge.edgeAxis().get_radius());
        else nonCircular++;
        edge.close();
      }
      var expectedArea=Math.PI*4*4;
      for(radius in radii) expectedArea-=Math.PI*radius*radius;
      var area=face.faceArea(),error=Math.abs(area-expectedArea),outerArcs=0;
      for(radius in boundaryRadii) if(Math.abs(radius-4)<1e-6) outerArcs++;
      if(nonCircular!=0 || outerArcs!=2 || error>0.00001) throw "Insert boundary changed; re-inspect possible key geometry";
      observations.push({port:id,insertFace:binding.insertFaceIndex,centreStepMm:centre,outwardNormalStep:normal,
        contactCount:port.contactCount,contactRadiiMm:radii,areaMm2:area,circularDiscMinusContactsAreaMm2:expectedArea,
        areaResidualMm2:error,outerRadiusMm:4,outerSemicircularEdges:outerArcs,nonCircularEdges:nonCircular,
        conclusion:"Insert face is a circular disc minus contact holes. No key notch is represented on this face; cable seating and key clocking cannot be qualified from it."});
      face.close();
    }
    solid.close();source.close();
    var gaps=ReferenceWristInterfaces.cableGeometryGaps(profile,"dh-fairino-robot-cable-14");
    if(gaps.length==0) throw "Catalogue selection code unexpectedly qualified as cable geometry";
    var rejected=false;
    try ReferenceWristInterfaces.requireCableGeometry(profile,"dh-fairino-robot-cable-14") catch(_:Dynamic) rejected=true;
    if(!rejected) throw "Unqualified supplier cable must not be automatically placed";
    var directory=root+"/wrist-cable-qualification";sys.FileSystem.createDirectory(directory);
    sys.io.File.saveContent(directory+"/report.json",haxe.Json.stringify({
      status:"Awaiting dimensioned mating cable evidence",sourceStepSha256:first.sourceStepSha256,
      sourceUrdfSha256:robot.manifest.urdfSha256,sourceSolid:first.solidIndex,observations:observations,
      candidate:profile.cables[0],missingEvidence:gaps,cableGeometryEmitted:false,clearanceStatus:"Not evaluated: no qualified plug envelope, seating transform or cable specification",
      motionStatus:"No cable route or moving harness has been certified",automaticCablePlacementRejected:rejected},null,"  "));
    Sys.println("FR5 exact insert-face audit passed: circular boundaries/contact holes; no insert key geometry. Cable placement rejected with "+gaps.length+" missing evidence items; no clearance claim emitted.");
  }
}
