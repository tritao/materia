package machinekit.control.erob;

/** SI-unit boundary; a future EtherCAT backend owns ESI/PDO scaling and CiA402 transitions. */
interface ERobDriveInterface {
  function configure(joints:Int):Void;
  function command(timeSeconds:Float, position:Array<Float>, velocity:Array<Float>, torqueNm:Array<Float>):Void;
  function feedback():{timeSeconds:Float,position:Array<Float>,velocity:Array<Float>,torqueNm:Array<Float>,enabled:Bool};
  function stop():Void;
  function close():Void;
}
