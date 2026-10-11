import haxe.Int64;
import processkit.feeding.BlowFeedModel;
import processkit.feeding.DiscreteBlowFeed;
import processkit.feeding.ScrewFeederDevice;
import processkit.feeding.ScrewFeederIo;
import processkit.feeding.ScrewFeederIo.ScrewFeederIoSample;
import processkit.feeding.ScrewFeederIoProfile;

/** Air timing, receipt and metering share the feeder boundary without inventing physical success. */
class BlowFeedTests {
  static var checks = 0;
  static function check(value:Bool,label:String):Void {if (!value) throw 'Blow feed: $label'; checks++;}
  static function rejects(run:Void -> Void,label:String):Void {
    var failed = false; try run() catch (_:Dynamic) failed = true; check(failed,label);
  }
  static function observed(device:ScrewFeederDevice):processkit.feeding.ScrewFeederDevice.ScrewFeederReading {
    var value = device.reading(); if (value == null) throw "Missing blow-feed acquisition"; return value;
  }
  static function fault(device:ScrewFeederDevice):String {
    var value = device.faultDetail(); if (value == null) throw "Expected blow-feed diagnostic"; return value;
  }
  public static function run():Void {
    checks = 0;
    var ready = true, present = false, delivered:Array<String> = [];
    var model = new BlowFeedModel(["a","b"],2,0.1,0.4,2,0.2,"clock",() -> ready,() -> present,(id) -> {delivered.push(id); present = true;});
    model.enable(true); model.poll(0); model.poll(0.11);
    check(observed(model).ready,"buffer ready while receiving tool idle");
    ready = false; rejects(() -> model.request(),"busy tool prevents delivery"); ready = true;
    model.request(); check(model.airOn,"request opens commissioned air pulse");
    rejects(() -> model.request(),"double request is interlocked");
    model.poll(0.32); check(!model.airOn && delivered.length == 0,"air ends before delayed receipt");
    model.poll(0.52); check(observed(model).present && delivered.join(",") == "a","only actual deposit reports receipt");
    rejects(() -> model.confirmConsumed("b"),"wrong consumption identity rejected");
    rejects(() -> model.confirmConsumed("a"),"occupied receiver cannot be cleared by command");
    present = false; model.confirmConsumed("a"); model.poll(0.53); model.request();
    model.safe(); model.poll(1); check(delivered.length == 1 && !model.airOn,"stop cancels metering and air");
    model.enable(true); model.poll(1.1); check(delivered.length == 1,"restart never replays release");
    model.request(); model.poll(1.51); check(delivered.join(",") == "a,b","explicit restart request delivers next identity");
    present = false; model.confirmConsumed("b"); model.poll(1.52);
    check(observed(model).empty == true,"depleted bulk and buffer remain distinct from receipt");
    model.transport.refill(["c"]); model.poll(1.53); model.poll(1.64); check(observed(model).ready,"identified refill replenishes buffer");
    model.request(); model.setBlocked(true); model.poll(1.65);
    check(fault(model).indexOf("blocked") >= 0 && !model.airOn,"blocked tube latches and safes air");
    rejects(() -> model.recover(),"blocked equipment cannot recover");
    model.setBlocked(false); model.recover(); model.enable(true); model.poll(1.66); model.request();
    model.setNoseJam(true); model.poll(1.67);
    check(fault(model).indexOf("nosepiece") >= 0 && !model.airOn,"receiving jam has its own diagnostic");
    model.setNoseJam(false); model.recover(); model.enable(true); model.poll(1.68); model.request();
    ready = false; model.poll(1.69); check(fault(model).indexOf("readiness") >= 0 && !model.airOn,"readiness loss during flight stops delivery");
    ready = true; model.reset(); check(model.transport.bulk() == 2 && !model.airOn,"reset restores authored stock without request");
    var missing = new BlowFeedModel(["lost"],1,0.1,0.2,1,0.1,"clock",() -> true,() -> false,(_) -> {});
    missing.enable(true); missing.poll(0); missing.poll(0.11); missing.request(); missing.poll(0.32);
    check(!observed(missing).present,"missing physical screw never becomes receipt");
    missing.poll(1.12); check(fault(missing).indexOf("missing") >= 0 && !missing.airOn,"missing receipt times out and safes air");
    var io = new BlowTestIo();
    var profile = new ScrewFeederIoProfile(0,1,0,1,null,null,"clock",0.5,2);
    var hardware = new DiscreteBlowFeed(io,profile,2,0.2,2,3,() -> true);
    io.frame(1,0,[true,false,false,false]); hardware.poll(0); hardware.enable(true);
    io.frame(2,0.01,[true,false,false,false]); hardware.poll(0.01); hardware.request();
    check(io.outputs[0] && io.outputs[1] && io.outputs[2],"hardware has separate enable/meter/air outputs");
    io.frame(3,0.22,[true,false,false,false]); hardware.poll(0.22);
    check(io.outputs[1] && !io.outputs[2] && !observed(hardware).present,"air timeout does not invent or acknowledge receipt");
    io.frame(4,0.3,[false,true,false,false]); hardware.poll(0.3);
    check(observed(hardware).present && !io.outputs[1],"fresh receipt acknowledges exactly one hardware delivery");
    hardware.safe(); check(!io.outputs[0] && !io.outputs[1] && !io.outputs[2],"hardware stop safes all three outputs");
    hardware.enable(true); io.frame(5,0.31,[true,false,false,false]); hardware.poll(0.31); hardware.request();
    io.frame(6,0.32,[true,false,true,false]); hardware.poll(0.32);
    check(fault(hardware).indexOf("blocked") >= 0 && !io.outputs[2],"scripted blocked-tube input safes outputs");
    rejects(() -> hardware.enable(true),"faulted hardware requires an explicit new owner");
    var unknown = new DiscreteBlowFeed(new BlowTestIo(),profile,2,0.2,null,null,() -> true);
    check(unknown.reading() == null,"unacquired hardware has no fabricated observations");
    rejects(() -> {var invalid = new DiscreteBlowFeed(io,profile,1,0.2,null,null,() -> true);},"air output cannot alias metering");
    rejects(() -> {var invalid = new DiscreteBlowFeed(io,profile,2,0.2,0,null,() -> true);},"tube diagnostic cannot alias ready input");
    trace('Blow-feed tests passed ($checks checks)');
  }
}
private class BlowTestIo implements ScrewFeederIo {
  public final outputs:Array<Bool> = [false,false,false];
  var value:Null<ScrewFeederIoSample> = null;
  public function new() {}
  public function frame(sequence:Int,time:Float,inputs:Array<Bool>):Void value = {sequence:Int64.ofInt(sequence),acquiredAt:time,clockId:"clock",inputs:inputs};
  public function write(pin:Int,on:Bool):Void outputs[pin] = on;
  public function sample():Null<ScrewFeederIoSample> return value;
}
function main():Void BlowFeedTests.run();
