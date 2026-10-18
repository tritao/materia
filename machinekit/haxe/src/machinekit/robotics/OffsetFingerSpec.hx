package machinekit.robotics;

/** Shared interface contract, independent of manufacturing route. Coordinates mm. */
class OffsetFingerSpec {
  public static final INTERFACE_ID="offset-finger-2xm4-10-pitch-60-blank-v1";
  public static final CENTRE_X=194.503081202068;
  public static final CENTRE_Z=307.089453582398;
  public static final MOUNT_Y=-2.27830199531;
  public static final MIN_GAP=26.0;
  public static final MAX_GAP=76.0;
  public static final GRIP_GAP=60.0;
  public final left:Bool;
  public final mountX:Float;
  public final mountZ:Float;
  public function new(left:Bool) {
    this.left=left;mountX=left ? 193.003081202068 : 196.003081202068;
    mountZ=CENTRE_Z+(left ? -13.2 : 13.2);
  }
  public function holes():Array<Float> return [mountX-5,mountX+5];
}
