package processkit.feeding;

/** Commissioned pins and timing; an uninstalled diagnostic input has no pin. */
class ScrewFeederIoProfile {
  public final enablePin:Int;
  public final requestPin:Int;
  public final readyPin:Int;
  public final presentPin:Int;
  public final emptyPin:Null<Int>;
  public final jamPin:Null<Int>;
  public final clockId:String;
  public final maximumAge:Float;
  public final deliveryTimeout:Float;
  public function new(enablePin:Int, requestPin:Int, readyPin:Int, presentPin:Int,
      emptyPin:Null<Int>, jamPin:Null<Int>, clockId:String, maximumAge:Float, deliveryTimeout:Float) {
    if (clockId == null || StringTools.trim(clockId).length == 0 || !Math.isFinite(maximumAge) || maximumAge <= 0
        || !Math.isFinite(deliveryTimeout) || deliveryTimeout <= maximumAge)
      throw "Screw feeder needs a source clock, feedback age and longer delivery timeout";
    distinct([enablePin,requestPin]);
    var inputs = [readyPin,presentPin];
    if (emptyPin != null) inputs.push(emptyPin);
    if (jamPin != null) inputs.push(jamPin);
    distinct(inputs);
    this.enablePin = enablePin; this.requestPin = requestPin; this.readyPin = readyPin; this.presentPin = presentPin;
    this.emptyPin = emptyPin; this.jamPin = jamPin; this.clockId = clockId;
    this.maximumAge = maximumAge; this.deliveryTimeout = deliveryTimeout;
  }
  static function distinct(pins:Array<Int>):Void {
    for (i in 0...pins.length) {
      if (pins[i] < 0 || pins[i] > 65535) throw "Screw feeder pin is outside 0..65535";
      for (j in 0...i) if (pins[i] == pins[j]) throw "Screw feeder pins must be distinct in their direction";
    }
  }
  public function copy():ScrewFeederIoProfile return new ScrewFeederIoProfile(enablePin,requestPin,readyPin,presentPin,
    emptyPin,jamPin,clockId,maximumAge,deliveryTimeout);
}
