package machinekit.robotics;

import cadkit.modeling.Part;
import cadkit.modeling.Vector;

/** Geometry for OffsetGripperFinger; coordinates are the shared closed-jaw interface frame. */
class OffsetFingerGeometry {
  public static final CENTRE_X=OffsetFingerSpec.CENTRE_X;
  public static final CENTRE_Z=OffsetFingerSpec.CENTRE_Z;
  public static final MOUNT_Y=OffsetFingerSpec.MOUNT_Y;
  static function box(x0:Float,x1:Float,y0:Float,y1:Float,z0:Float,z1:Float):Part {
    var raw=Part.box(x1-x0,y1-y0,z1-z0,Min,Min,Min);
    var result=raw.translated(new Vector(x0,y0,z0));raw.close();return result;
  }
  public static function blank():Part return box(CENTRE_X-30,CENTRE_X+30,30,90,CENTRE_Z-10,CENTRE_Z+10);
  public static function contact(left:Bool):Part {
    var x=left ? CENTRE_X-19 : CENTRE_X+13;
    return box(x,x+6,16,50,
      left ? CENTRE_Z-20.2 : CENTRE_Z-10,
      left ? CENTRE_Z+10 : CENTRE_Z+20.2);
  }
  public static function make(left:Bool,variant:String="cnc-aluminium"):Part {
    var sls=variant=="sls-pa12";
    if(!sls && variant!="cnc-aluminium") throw "Unknown finger manufacturing variant";
    var halfHeight=sls ? 9.0 : 7.0, extra=sls ? 4.0 : 2.0;
    var x0=left ? 183.003081202068 : 186.003081202068;
    var mountZ=CENTRE_Z+(left ? -13.2 : 13.2);
    var pad=sls ? box(left ? CENTRE_X-21 : CENTRE_X+13,left ? CENTRE_X-13 : CENTRE_X+21,
      16,50,left ? CENTRE_Z-22.2 : CENTRE_Z-10,left ? CENTRE_Z+10 : CENTRE_Z+22.2) : contact(left);
    var base=box(x0-extra,x0+20+extra,MOUNT_Y,20,mountZ-halfHeight,mountZ+halfHeight);
    var bridge=box(left ? CENTRE_X-(sls ? 21 : 19) : x0-extra,left ? x0+20+extra : CENTRE_X+(sls ? 21 : 19),
      12,20,mountZ-halfHeight,mountZ+halfHeight);
    var result=Part.fuseAll([base,bridge,pad]);base.close();bridge.close();pad.close();
    // Explicit R2 root in the XY section; avoids relying on a fillet across union seams.
    var rootX=left ? CENTRE_X-13 : CENTRE_X+13;
    var square=box(left ? rootX : rootX-(sls ? 3 : 2),left ? rootX+(sls ? 3 : 2) : rootX,20,20+(sls ? 3 : 2),mountZ-halfHeight,mountZ+halfHeight);
    var cutter=Part.cylinderSpan(sls ? 3 : 2,mountZ-halfHeight-1,mountZ+halfHeight+1,left ? rootX+(sls ? 3 : 2) : rootX-(sls ? 3 : 2),20+(sls ? 3 : 2));
    var root=square.subtract(cutter);square.close();cutter.close();
    var rounded=Part.fuseAll([result,root]);root.close();result.close();result=rounded;
    for(x in [x0+5,x0+15]) {
      // Existing M4 screws: bearing face is 4.5 mm ahead of the mounting plane.
      var through=Part.cylinderAlongY(sls ? 3.1 : 2.25,MOUNT_Y-1,25,x,mountZ);
      var counterbore=Part.cylinderAlongY(sls ? 5.1 : 3.75,MOUNT_Y+(sls ? 3.5 : 4.5),25,x,mountZ);
      var cut=result.subtractAll([through,counterbore]);result.close();through.close();counterbore.close();result=cut;
    }
    var before=result.shape.subshapeCount(CadKit.ShapeKind.Face);
    var refined=result.refined();result.close();result=refined;
    Sys.println('Refined ${left ? "left" : "right"} finger: $before to ${result.shape.subshapeCount(CadKit.ShapeKind.Face)} faces');
    if(!result.valid() || result.solidCount()!=1) {result.close();throw "Custom finger is not one valid solid";}
    return result;
  }
}
