package processkit.screwdriving;

/** Program codes are commissioned explicitly; no slot-to-bit or reverse/start convention is guessed. */
typedef ScrewdriverIoProgram = {var slot:Int; var code:Int;}

/** Active-high, binary program pins in least-significant-bit order.
    Each controller's physical polarity conversion belongs in its I/O implementation. */
class ScrewdriverIoProfile {
  public final programBits:Array<Int>;
  public final programs:Array<ScrewdriverIoProgram>;
  public final start:Int;
  public final reverse:Int;
  public final vacuum:Int;
  public final ready:Int;
  public final ok:Int;
  public final nok:Int;
  /** Commissioned reverse may require the start output as well. */
  public final reverseNeedsStart:Bool;
  public final clockId:String;
  public final maximumAge:Float;
  public function new(programBits:Array<Int>, programs:Array<ScrewdriverIoProgram>, start:Int, reverse:Int,
      vacuum:Int, ready:Int, ok:Int, nok:Int, reverseNeedsStart:Bool, clockId:String, maximumAge:Float) {
    if (programBits == null || programBits.length < 1 || programBits.length > 16 || programs == null || programs.length == 0
        || clockId == null || StringTools.trim(clockId).length == 0 || !Math.isFinite(maximumAge) || maximumAge <= 0)
      throw "Screwdriver I/O needs commissioned program bits, slots, clock and feedback age";
    var outputs = programBits.copy(); outputs.push(start); outputs.push(reverse); outputs.push(vacuum);
    distinct(outputs,"output"); distinct([ready,ok,nok],"input");
    var copied:Array<ScrewdriverIoProgram> = [];
    for (program in programs) {
      if (program == null || program.slot <= 0 || program.code < 0 || program.code >= (1 << programBits.length))
        throw "Screwdriver I/O program code does not fit its commissioned bits";
      for (prior in copied) if (prior.slot == program.slot || prior.code == program.code)
        throw "Screwdriver I/O needs distinct slots and program codes";
      copied.push({slot:program.slot,code:program.code});
    }
    this.programBits = programBits.copy(); this.programs = copied;
    this.start = start; this.reverse = reverse; this.vacuum = vacuum;
    this.ready = ready; this.ok = ok; this.nok = nok; this.reverseNeedsStart = reverseNeedsStart;
    this.clockId = clockId; this.maximumAge = maximumAge;
  }
  static function distinct(pins:Array<Int>, kind:String):Void {
    for (i in 0...pins.length) {
      if (pins[i] < 0 || pins[i] > 65535) throw 'Screwdriver I/O $kind pin is outside 0..65535';
      for (j in 0...i) if (pins[i] == pins[j]) throw 'Screwdriver I/O $kind pins must be distinct';
    }
  }
  public function code(slot:Int):Int {
    for (program in programs) if (program.slot == slot) return program.code;
    throw 'Screwdriver I/O has no commissioned program slot $slot';
  }
  public function copy():ScrewdriverIoProfile return new ScrewdriverIoProfile(programBits,programs,start,reverse,
    vacuum,ready,ok,nok,reverseNeedsStart,clockId,maximumAge);
}
