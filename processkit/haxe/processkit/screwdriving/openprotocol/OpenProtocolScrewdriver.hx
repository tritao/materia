package processkit.screwdriving.openprotocol;

import processkit.screwdriving.*;
import processkit.screwdriving.ScrewdriverAcquisitionSource.ScrewdriverAcquisition;

/** Network program acknowledgement and enable with independently measured machine I/O.
    Commissioned program codes identify the same parameter sets on both interfaces.
    MID 0061 remains a controller report; it never replaces cumulative spindle feedback. */
class OpenProtocolScrewdriver implements PolledScrewdriverDevice implements ScrewdriverOutputs
    implements ScrewdriverFeedback implements ScrewdriverDeviceControl {
  final client:OpenProtocolClient;
  final machine:DiscreteScrewdriver;
  final profile:ScrewdriverIoProfile;
  var requested = 0;
  var selecting = false;
  var enabling = false;
  var inhibited = true;
  var closed = false;
  var failure:Null<String> = null;
  var report:Null<OpenProtocolResult> = null;

  public function new(client:OpenProtocolClient, io:ScrewdriverIo, profile:ScrewdriverIoProfile) {
    if (client == null || io == null || profile == null)
      throw "Network screwdriver needs a client and commissioned measured I/O";
    for (program in profile.programs) if (program.code < 1 || program.code > 999)
      throw "Network screwdriver parameter-set codes must be in 1..999";
    this.client = client; this.profile = profile.copy();
    machine = new DiscreteScrewdriver(io,profile);
    client.safe();
  }
  public function outputs():ScrewdriverOutputs return this;
  public function feedback():ScrewdriverFeedback return this;
  public function reading():ScrewdriverReading return machine.reading();
  public function acquisition():Null<ScrewdriverAcquisition> return machine.acquisition();
  public function lastReport():Null<OpenProtocolResult> return report;
  public function faultDetail():Null<String> return failure;
  public function ready():Bool return !closed && !inhibited && failure == null && requested > 0
    && client.ready() && client.selectedProgram == profile.code(requested) && machine.ready();
  public function selectProgram(slot:Int):Void {
    if (closed || failure != null)
      throw "Network screwdriver selection requires a stopped, healthy device";
    var code = profile.code(slot);
    machine.selectProgram(slot);
    requested = slot; selecting = false; enabling = false; inhibited = false;
    if (client.initialized) client.safe();
  }
  public function start(on:Bool):Void {
    if (on && !ready()) throw "Network screwdriver start requires acknowledged program, enable and measured readiness";
    machine.start(on);
  }
  public function reverse(on:Bool):Void {
    if (on && !ready()) throw "Network screwdriver reverse requires acknowledged program, enable and measured readiness";
    machine.reverse(on);
  }
  public function vacuum(on:Bool):Void {
    if (on && (closed || failure != null)) throw "Network screwdriver cannot acquire after a controller fault";
    machine.vacuum(on);
  }
  /** Stop physical drive first; a socket close is never a stop acknowledgement. */
  public function safe():Void {
    inhibited = true; selecting = false; enabling = false;
    machine.safe(); client.safe();
  }
  public function safeAcknowledged():Bool return !closed && failure == null && inhibited
    && client.safeAcknowledged() && machine.safeAcknowledged();
  public function close():Void {
    if (closed) return;
    try safe() catch (error:Dynamic) failure = Std.string(error);
    closed = true; client.close();
  }
  public function poll(now:Float):Void {
    if (closed || failure != null) return;
    try {
      machine.poll(now); client.poll(now);
      var detail = machine.faultDetail();
      if (detail != null) throw detail;
      if (client.fault != null) throw client.fault;
      while (true) {
        var value = client.nextResult(); if (value == null) break;
        report = value;
      }
      if (!inhibited && requested > 0 && client.initialized && client.idle()) {
        if (!selecting && client.safeAcknowledged()) {
          client.selectProgram(profile.code(requested)); selecting = true;
        } else if (selecting && !enabling && client.selectedProgram == profile.code(requested)) {
          client.enable(); enabling = true;
        }
      }
    } catch (error:Dynamic) {
      failure = Std.string(error);
      try safe() catch (_:Dynamic) {}
    }
  }
}
