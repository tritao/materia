import cadkit.Shape;
import cadkit.modeling.Part;
import cadkit.modeling.Vector;

/** Prototype offset fingers in the supplier frame, with jaws at their closed stop.
 * Dimensions in mm. Mount centres follow the pinned supplier fasteners, not a guessed mirror. */
class DhCustomFingers {
  public static final MIN_GAP=machinekit.robotics.OffsetFingerSpec.MIN_GAP;
  public static final MAX_GAP=machinekit.robotics.OffsetFingerSpec.MAX_GAP;
  public static final GRIP_GAP=machinekit.robotics.OffsetFingerSpec.GRIP_GAP;
  public static final CENTRE_X=machinekit.robotics.OffsetFingerSpec.CENTRE_X;
  public static final CENTRE_Z=machinekit.robotics.OffsetFingerSpec.CENTRE_Z;
  static final MOUNT_Y=machinekit.robotics.OffsetFingerSpec.MOUNT_Y;

  public static function contact(left:Bool):Part return machinekit.robotics.OffsetFingerGeometry.contact(left);
  public static function make(left:Bool,variant:String="cnc-aluminium"):Part
    return new machinekit.robotics.OffsetGripperFinger(left,variant).geometry();
  public static function blank():Part {
    return machinekit.robotics.OffsetFingerGeometry.blank();
  }
  /** Audit another orientation of the same stock without changing the finger interface. */
  public static function checkBlankOrientation(stock:Shape,expectedPatch:Float):Void {
    for(left in [true,false]) {
      var part=make(left);
      for(gap in [60.0,65.0,70.0,76.0]) {
        var finger=part.shape.translate(new Vector((left ? -1 : 1)*(gap-MIN_GAP)/2,0,0).native());
        if(volumeOverlap(finger,stock)>0.001) throw "Reoriented stock penetrates finger";
        if(Math.abs(finger.distance(stock)-(gap-60)/2)>0.001) throw "Reoriented stock clearance changed";
        if(gap==60) {
          var area=bearingArea(finger,stock,new Vector(CENTRE_X+(left ? -30 : 30),0,0),Vector.X(),left ? -1 : 1);
          if(Math.abs(area-expectedPatch)>0.1) throw 'Reoriented stock contact $area != $expectedPatch mm2';
          Sys.println('Reoriented stock contact: $area mm2; no penetration');
        }
        finger.close();
      }
      part.close();
    }
  }
  static function volumeOverlap(a:Shape,b:Shape):Float {
    var overlap=a.common(b);
    var value=overlap.subshapeCount(CadKit.ShapeKind.Solid)==0 ? 0 : overlap.volume();
    overlap.close();return value;
  }
  // Solid/solid booleans can omit tangential faces. Compare interior sections
  // projected onto the same plane, after independently checking zero separation.
  static function bearingArea(a:Shape,b:Shape,origin:Vector,axis:Vector,insideA:Float):Float {
    var delta=axis.scale(insideA*0.001);
    var sa=a.section(origin.add(delta).native(),axis.native());
    var sb=b.section(origin.subtract(delta).native(),axis.native());
    var aligned=sb.translate(delta.scale(2).native());
    var patch=sa.common(aligned);
    var area=patch.subshapeCount(CadKit.ShapeKind.Face)>0 ? patch.area() : 0;
    sa.close();sb.close();aligned.close();patch.close();return area;
  }
  /** Verify supplier carriage contact, fastener clearance and stock approach. */
  public static function check(left:Shape,right:Shape,carriages:Array<Shape>,fasteners:Array<Shape>,variant:String="cnc-aluminium"):Void {
    var sleeves:Array<Part> = [];
    for(side in [true,false]) for(part in new machinekit.robotics.OffsetGripperFinger(side,variant).compressionSleeves()) sleeves.push(part);
    for(i in 0...2) {
      var finger=i==0 ? left : right;
      if(finger.distance(carriages[i])>0.001) throw "Custom finger is disconnected from its supplier carriage";
      if(volumeOverlap(finger,carriages[i])>0.001) throw "Custom finger penetrates supplier carriage";
      var mountArea=bearingArea(finger,carriages[i],new Vector(0,MOUNT_Y,0),Vector.Y(),1);
      Sys.println('Finger $i carriage bearing area $mountArea mm2');
      if(mountArea<50) throw "Finger mounting contact is only an edge or too small";
      for(j in 0...2) {
        var screw=fasteners[i*2+j];
        if(volumeOverlap(finger,screw)>0.001) throw "Custom finger obstructs retained supplier screw";
        var seat=variant=="sls-pa12" ? sleeves[i*2+(i==0 ? 1-j : j)].shape : finger;
        if(seat.distance(screw)>0.001) throw "Supplier screw head does not seat on counterbore floor";
        if(variant=="sls-pa12") {
          if(volumeOverlap(seat,finger)>0.001 || volumeOverlap(seat,carriages[i])>0.001)
            throw "Compression sleeve penetrates polymer or carriage";
          if(seat.distance(finger)>0.001 || seat.distance(carriages[i])>0.001)
            throw "Compression sleeve load path is disconnected";
        }
        var area=bearingArea(seat,screw,new Vector(0,MOUNT_Y+4.5,0),Vector.Y(),-1);
        Sys.println('Finger $i screw $j bearing contact area $area mm2');
        if(area<20) throw "Insufficient screw-head bearing contact";
        var bounds=screw.bounds(),lo=bounds.get_min(),hi=bounds.get_max();
        var x=(lo.get_x()+hi.get_x())/2,z=(lo.get_z()+hi.get_z())/2;
        // 6 mm diameter driver envelope, from the head front to open tool space.
        var access=Part.cylinderAlongY(3,hi.get_y()+0.1,60,x,z);
        if(volumeOverlap(finger,access.shape)>0.001) {access.close();throw "Driver cannot reach screw";}
        for(travel in [0.0,12.5,17.0,25.0]) {
          var opposite=(i==0 ? right : left).translate(new Vector((i==0 ? 2 : -2)*travel,0,0).native());
          var overlap=volumeOverlap(opposite,access.shape);opposite.close();
          if(overlap>0.001) {access.close();throw "Opposite finger blocks driver access";}
        }
        access.close();
      }
    }
    for(sleeve in sleeves) sleeve.close();
    var stock=blank();
    try {
      for(gap in [60.0,65.0,70.0,76.0]) {
        var travel=(gap-MIN_GAP)/2;
        var l=left.translate(new Vector(-travel,0,0).native());
        var r=right.translate(new Vector(travel,0,0).native());
        try {
          for(finger in [l,r]) {
            if(volumeOverlap(finger,stock.shape)>0.001) throw 'Finger intrudes into blank at $gap mm gap';
            if(gap==60) {
              var leftSide=finger==l;
              var area=bearingArea(finger,stock.shape,
                new Vector(CENTRE_X+(leftSide ? -30 : 30),0,0),Vector.X(),leftSide ? -1 : 1);
              Sys.println('Blank contact patch $area mm2 (zero penetration)');
              if(Math.abs(area-400)>0.1) throw "Wrong blank contact patch";
            }
            if(Math.abs(finger.distance(stock.shape)-(gap-60)/2)>0.001)
              throw 'Wrong blank clearance at $gap mm gap';
          }
        } catch(error:Dynamic) {l.close();r.close();throw error;}
        l.close();r.close();
      }
      var mass=stock.shape.volume()*2700e-9;
      var requiredPerJaw=3*mass*(9.81+10)/(2*0.1);
      // Planning assumptions, not a manufacturer's finger-force rating.
      var commanded=80.0;
      var lever=40-MOUNT_Y;
      if(commanded*lever/1000>7 || commanded*13.2/1000>7 || requiredPerJaw>commanded)
        throw "Finger load assumptions exceed the preliminary design budget";
      Sys.println('Custom fingers: 26–76 mm gap; 60 mm grip at 17 mm/jaw; blank $mass kg; required $requiredPerJaw N/jaw (mu=0.1, a=10 m/s2, factor=3)');
      Sys.println('Planning only: 80 N/jaw gives ${commanded*lever/1000} Nm at the mount; manufacturer finger-force derating remains unverified');
      Sys.println("Custom finger checks passed: mount contact, screw bearing seats, driver access, and blank grip/approach at four gaps");
    } catch(error:Dynamic) {stock.close();throw error;}
    stock.close();
  }
}
