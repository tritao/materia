import materia.assembly.AssemblyFrames as F;
import materia.assembly.AssemblyRecord.AssemblyFrame;

/** Floor-standing rack; device local XY mounting faces lie on the vertical backplate. */
class HobberElectricalLayout {
  public static function panelPose():AssemblyFrame
    return F.translation(-650,100,-FlexsplineHobber.MACHINE_FROM_WORK.z);

  public static function devicePose(offsetX:Float,heightFromFloor:Float):AssemblyFrame {
    var panel=panelPose();
    // Device local +Z points out of the plate, towards work -Y; local +Y points up.
    return F.fromRotationMatrix(panel.x+offsetX,panel.y-10,panel.z+heightFromFloor,
      [1,0,0, 0,0,-1, 0,1,0]);
  }
}
