import haxe.Int64;
import processkit.feeding.*;
import processkit.feeding.ScrewFeederIo.ScrewFeederIoSample;

/** Metering and acquisition failures must be observable without simulating loose-part collisions. */
class ScrewFeederTests {
  static var checks = 0;
  static function check(value:Bool, detail:String):Void {checks++; if (!value) throw detail;}
  static function rejects(work:Void->Void, detail:String):Void {
    var failed = false; try work() catch (_:Dynamic) failed = true; check(failed,detail);
  }
  static function observed(device:ScrewFeederDevice):processkit.feeding.ScrewFeederDevice.ScrewFeederReading {
    var value = device.reading(); if (value == null) throw "Feeder has no acquisition"; return value;
  }
  public static function run():Void {
    checks = 0;
    var nest = new FeederNest();
    var model = new ScrewFeederModel(["a","b"],2,0.1,0.2,1,"test",() -> nest.present,(id) -> {nest.present = true; nest.id = id;});
    model.enable(true); model.poll(0); check(!observed(model).ready,"A bowl cannot invent an immediately oriented screw");
    model.poll(0.11); check(model.buffered() == 1 && model.bulk() == 1 && observed(model).ready,"Track transport fills a bounded buffer");
    model.request(); rejects(() -> model.request(),"One request cannot meter two screws");
    model.poll(0.2); check(model.deliveries == 0,"Delivery delay is measured");
    model.poll(0.32); check(model.deliveries == 1 && nest.id == "a" && model.delivered == "a","Delivery preserves the actual inventory identity");
    rejects(() -> model.request(),"Occupied nest interlocks requests");
    rejects(() -> model.confirmPickup("a"),"A screw still at the nest cannot count as picked up");
    nest.present = false; model.poll(0.33); check(!observed(model).ready,"Presence loss alone cannot replenish");
    rejects(() -> model.confirmPickup("b"),"Pickup must identify the delivered body");
    model.confirmPickup("a"); model.poll(0.34); model.request(); model.safe(); model.poll(0.8);
    check(model.deliveries == 1 && !nest.present,"Stop cancels metering without release");
    model.enable(true); model.poll(0.81); check(observed(model).ready,"Restart preserves buffered inventory");
    model.poll(1.1); check(model.deliveries == 1,"Restart does not replay a request");
    model.request(); model.poll(1.31); check(nest.id == "b" && observed(model).empty == true,"Last identified screw depletes supply");
    nest.present = false; model.confirmPickup("b"); model.poll(1.32);
    check(!observed(model).ready && observed(model).empty == true,"An empty bowl cannot supply a screw");
    rejects(() -> model.refill(["a"]),"Refill cannot reuse a delivered identity");
    model.refill(["c"]); model.poll(1.33); model.poll(1.44); check(model.nextIdentity() == "c","Refill restores identified supply");
    model.setJam(true); model.poll(1.45); check(model.faultDetail() != null && !observed(model).ready,"Jam stops feeding");
    rejects(() -> model.enable(true),"Latched jam cannot restart automatically");
    model.setJam(false); model.recover(); model.enable(true); model.poll(1.46); check(observed(model).ready,"Explicit cleared jam recovery preserves inventory");
    var io = new ScriptedFeederIo();
    var profile = new ScrewFeederIoProfile(0,1,0,1,null,null,"io",0.1,0.5);
    var device = new DiscreteScrewFeeder(io,profile); device.enable(true);
    io.acquire(1,0,[true,false]); device.poll(0);
    check(observed(device).ready && observed(device).empty == null && observed(device).jammed == null,"Uninstalled diagnostic sensors stay unknown");
    device.request(); check(io.outputs[1],"Common device requests one screw through commissioned I/O");
    io.acquire(2,0.02,[false,true]); device.poll(0.02); check(!io.outputs[1] && observed(device).present,"Fresh measured presence acknowledges delivery");
    rejects(() -> device.request(),"Hardware occupied nest interlocks requests");
    device.safe(); check(!io.outputs[0] && !io.outputs[1],"Safe removes both feed outputs");
    device.enable(true); check(!observed(device).ready,"Enable needs a new acquisition");
    io.acquire(3,0.03,[true,false]); device.poll(0.03); device.request(); device.safe(); device.enable(true);
    io.acquire(4,0.04,[true,false]); device.poll(0.04); check(!io.outputs[1],"Hardware restart does not replay delivery");
    device.poll(0.2); check(device.faultDetail() != null && !io.outputs[0],"Repeated acquisitions do not hide stale feedback");
    var conflict = new DiscreteScrewFeeder(io,profile);
    io.acquire(5,0.2,[true,false]); conflict.poll(0.2); io.acquire(5,0.2,[false,false]); conflict.poll(0.2);
    check(conflict.faultDetail() != null,"A repeated source sequence cannot change inputs");
    var timing = new DiscreteScrewFeeder(io,profile); timing.enable(true);
    io.acquire(6,0.3,[true,false]); timing.poll(0.3); timing.request();
    for (i in 1...8) {var at = 0.3+i*0.08; io.acquire(6+i,at,[true,false]); timing.poll(at);}
    check(timing.faultDetail() != null && !io.outputs[1],"Delivery timeout still fires under fresh I/O");
    var diagnosed = new DiscreteScrewFeeder(io,new ScrewFeederIoProfile(0,1,0,1,2,3,"io",0.1,0.5));
    diagnosed.enable(true); io.acquire(20,1,[true,false,true,false]); diagnosed.poll(1);
    check(observed(diagnosed).empty == true && !observed(diagnosed).ready,"Installed empty input is measured");
    io.acquire(21,1.01,[true,false,false,true]); diagnosed.poll(1.01);
    check(diagnosed.faultDetail() != null && observed(diagnosed).jammed == true,"Installed jam input stops feeding");
    var resetNest = new FeederNest();
    var reset = new ScrewFeederModel(["reset"],1,0.1,0.2,1,"reset",() -> resetNest.present,(id) -> resetNest.present = true);
    reset.enable(true); reset.poll(0); reset.poll(0.11); reset.request(); reset.reset(); reset.poll(0.5);
    check(reset.deliveries == 0 && !resetNest.present && reset.bulk() == 1,"Session reset restores authored inventory without replay");
    var empty = new ScrewFeederModel([],1,0.1,0.2,1,"empty",() -> false,(_) -> {});
    empty.enable(true); empty.poll(0); check(observed(empty).empty == true && !observed(empty).ready,"An initially empty bowl is represented explicitly");
    var blockedNest = new FeederNest();
    var blocked = new ScrewFeederModel(["blocked"],1,0.1,0.2,1,"blocked",() -> blockedNest.present,(_) -> {throw "must not deposit";});
    blocked.enable(true); blocked.poll(0); blocked.poll(0.11); blocked.request(); blockedNest.present = true; blocked.poll(0.32);
    check(blocked.faultDetail() != null && blocked.deliveries == 0,"New occupancy during transport prevents deposit");
    var delayed = new ScrewFeederModel(["late"],1,0.1,0.2,1,"late",() -> false,(_) -> {});
    delayed.enable(true); delayed.poll(0); delayed.poll(0.11); delayed.request(); delayed.poll(1.2);
    check(delayed.faultDetail() != null && delayed.deliveries == 0,"Expired metering never deposits a late screw");
    var source = new DiscreteScrewFeeder(io,profile);
    io.acquire(30,2,[true,false]); source.poll(2); io.acquire(29,2.01,[true,false]); source.poll(2.01);
    check(source.faultDetail() != null,"Hardware source sequence regression stops feeding");
    var malformed = new DiscreteScrewFeeder(io,profile);
    io.acquire(31,2.1,[true]); malformed.poll(2.1);
    check(malformed.faultDetail() != null,"A short acquisition cannot fabricate presence");
    rejects(() -> new ScrewFeederIoProfile(0,0,0,1,null,null,"bad",0.1,1),"Output pins cannot alias");
    rejects(() -> new ScrewFeederModel(["same","same"],1,0.1,0.2,1,"bad",() -> false,(_) -> {}),"Inventory identities cannot alias");
    var waitIo = new ScriptedFeederIo(), waitDevice = new DiscreteScrewFeeder(waitIo,profile);
    waitIo.acquire(1,0,[true,false]); waitDevice.poll(0);
    var wait = new processkit.skill.WaitForScrewFeeder(waitDevice,1);
    wait.start();
    var snapshot = new robotkit.core.RobotSnapshot("feeder-test",Int64.ofInt(1),Int64.ofInt(0),[],[],[],0,0);
    var status = wait.update(snapshot,0.01);
    check(!waitIo.outputs[1],"Shared preparation waits for enabled acquisition before requesting");
    waitIo.acquire(2,0.01,[true,false]); waitDevice.poll(0.01);
    status = wait.update(snapshot,0.01); check(waitIo.outputs[1],"Shared preparation requests through the hardware contract");
    waitIo.acquire(3,0.02,[false,true]); waitDevice.poll(0.02);
    status = wait.update(snapshot,0.01);
    check(switch status {case Succeeded:true;case _:false;},"Shared preparation succeeds only on actual hardware receipt");
    var cancel = new processkit.skill.WaitForScrewFeeder(waitDevice,1); cancel.start(); cancel.cancel();
    check(!waitIo.outputs[0] && !waitIo.outputs[1],"Cancelling preparation safely stops hardware feeding");
    trace('Screw feeder tests passed ($checks checks)');
  }
  public static function main():Void run();
}
private class FeederNest {public var present = false; public var id = ""; public function new() {}}
private class ScriptedFeederIo implements ScrewFeederIo {
  public final outputs:Array<Bool> = [false,false];
  var latest:Null<ScrewFeederIoSample> = null;
  public function new() {}
  public function write(pin:Int,on:Bool):Void outputs[pin] = on;
  public function sample():Null<ScrewFeederIoSample> return latest;
  public function acquire(sequence:Int,time:Float,inputs:Array<Bool>):Void
    latest = {sequence:Int64.ofInt(sequence),acquiredAt:time,clockId:"io",inputs:inputs};
}
