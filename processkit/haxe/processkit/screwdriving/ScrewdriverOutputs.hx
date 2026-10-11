package processkit.screwdriving;
/** Device outputs, implemented by channels or a real tightening controller. */
interface ScrewdriverOutputs {
  function selectProgram(slot:Int):Void;
  function start(on:Bool):Void;
  function reverse(on:Bool):Void;
  function vacuum(on:Bool):Void;
}
