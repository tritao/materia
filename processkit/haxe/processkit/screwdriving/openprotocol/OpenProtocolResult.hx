package processkit.screwdriving.openprotocol;

/** MID 0061 revision 1, specification 2.16.0 section 5.8.2, table 98.
    The reported angle is not an absolute spindle encoder or a run-down datum.
    NOK is deliberately unclassified; this packet cannot prove cross-thread or strip. */
class OpenProtocolResult {
  public final id:Float;
  public final program:Int;
  public final ok:Bool;
  public final torqueStatus:Int;
  public final angleStatus:Int;
  public final torqueNm:Float;
  public final angleRad:Float;
  public final fingerprint:String;
  public function new(frame:OpenProtocolFrame) {
    if (frame.mid != 61 || frame.bytes.length != 232) throw "Open Protocol requires a complete revision-1 tightening result";
    var bytes = frame.bytes;
    // Every field tag must occur at its published byte offset.
    var offsets = [20,26,30,57,84,88,93,99,105,108,111,114,122,130,138,146,153,160,167,174,195,216,219];
    for (i in 0...offsets.length) if (OpenProtocolFrame.number(bytes,offsets[i],2) != i + 1)
      throw 'Open Protocol result field ${i + 1} has the wrong tag';
    for (field in [[22,4],[28,2],[86,2],[95,4],[101,4],[116,6],[124,6],[132,6],[148,5],[155,5],[162,5]]) {
      var value = OpenProtocolFrame.number(bytes,field[0],field[1]);
    }
    if (OpenProtocolFrame.number(bytes,218,1) > 3) throw "Open Protocol result has invalid batch status";
    for (offset in [176,197]) {
      var date = bytes.getString(offset,19);
      if (date.charAt(4) != "-" || date.charAt(7) != "-" || date.charAt(10) != ":" || date.charAt(13) != ":" || date.charAt(16) != ":")
        throw "Open Protocol result timestamp has invalid separators";
      for (field in [[0,4],[5,2],[8,2],[11,2],[14,2],[17,2]]) {
        var value = OpenProtocolFrame.number(bytes,offset + field[0],field[1]);
      }
    }
    program = Std.int(OpenProtocolFrame.number(bytes,90,3));
    var status = Std.int(OpenProtocolFrame.number(bytes,107,1));
    torqueStatus = Std.int(OpenProtocolFrame.number(bytes,110,1));
    angleStatus = Std.int(OpenProtocolFrame.number(bytes,113,1));
    if (program < 1 || status > 1 || torqueStatus > 2 || angleStatus > 2)
      throw "Open Protocol result has invalid status or program";
    ok = status == 1;
    torqueNm = OpenProtocolFrame.number(bytes,140,6) / 100;
    angleRad = OpenProtocolFrame.number(bytes,169,5) * Math.PI / 180;
    id = OpenProtocolFrame.number(bytes,221,10);
    if (id > 4294967295.0) throw "Open Protocol tightening ID exceeds uint32";
    fingerprint = bytes.getString(20,bytes.length - 21);
  }
}
