package materia.casting;

import materia.casting.InsertHole;
import materia.casting.CastingInput.CastingTextInput;

/** Receiver drilling belongs to the finished casting, never the pour cavity.
 * insertId names the separately itemised steel installed after cure. */
typedef PostCureBond = {
  var insertId:String;
  var receiver:InsertHole;
  var adhesive:CastingTextInput;
}

class PostCureBonds {
  public static function records(bonds:Array<PostCureBond>):Array<Dynamic> return [for (bond in bonds) {
    insertId:bond.insertId, receiver:bond.receiver, adhesive:CastingInput.CastingInputs.textRecord(bond.adhesive)
  }
  ];

  public static function snapshot(raw:Dynamic):Array<PostCureBond> {
    if (!Std.isOfType(raw, Array)) throw "Invalid post-cure bond list";
    var rows:Array<Dynamic> = cast raw, result:Array<PostCureBond> = [], seen = new Map<String, Bool>();
    for (row in rows) {
      CastingFeatureCodec.fields(row, ["insertId", "receiver", "adhesive"]);
      var id = CastingRecordCodec.string(row, "insertId");
      CastingFeatureCodec.identifier(id);
      if (seen.exists(id)) throw "Duplicate post-cure insert: " + id;
      seen.set(id, true);
      var hole = InsertHoleCodec.snapshot([Reflect.field(row, "receiver")])[0];
      if (hole.thread != null || hole.extent != Blind
        || hole.counterbore != null || hole.relief != null) throw "Bonded receiver must be a plain blind bore: " + id;
      result.push({
        insertId: id,
        receiver: hole,
        adhesive: CastingInput.CastingInputs.text(Reflect.field(
          row,
          "adhesive"
        ))
      });
    }
    result.sort((a, b) -> Reflect.compare(a.insertId, b.insertId));
    return result;
  }
}
