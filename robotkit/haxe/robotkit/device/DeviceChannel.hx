package robotkit.device;

/** One ordered hardware channel bound to a stable semantic joint ID. */
class DeviceChannel {
  public final index:Int;
  public final jointId:String;

  public function new(index:Int, jointId:String) {
    this.index = index;
    this.jointId = jointId;
  }
}
