package processkit;

import haxe.Int64;
import robotkit.tool.ChannelToolAdapter;
import robotkit.execution.FiredProcessEvent;
import robotkit.execution.ProcessEventValue;

/** Simulation process device driven by ChannelToolAdapter records. */
class ChannelProcessDevice implements ProcessDevice {
  public final adapter:ChannelToolAdapter;
  public final channel:String;
  final clock:Void -> Int64;
  var prepared:Bool = false;
  var currentFault:Null<String> = null;

  public function new(adapter:ChannelToolAdapter, channel:String, clock:Void -> Int64) {
    if (adapter == null || channel == null || channel.length == 0 || clock == null)
      throw "Channel process device needs adapter, channel and clock";
    this.adapter = adapter;
    this.channel = channel;
    this.clock = clock;
  }

  public function prepare():Void prepared = true;
  public function ready():Bool return prepared && currentFault == null;
  public function fault():Null<String> return currentFault;
  public function setFault(message:Null<String>):Void currentFault = message;
  public function safe():Void {
    var now = clock();
    adapter.apply(new FiredProcessEvent(Int64.ofInt(0), channel,
      ProcessEventValue.Analog(0.0), now, now, 0));
  }
  public function apply(records:Array<FiredProcessEvent>):Void adapter.applyAll(records);
}
