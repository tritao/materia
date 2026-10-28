package materia.casting;

/** A source lifting attachment is a particular steel thread, not an insert.
 * Several attachments may share one plate; fixture and anchor holes are never
 * promoted to lifting points by their diameter, label or array order. */
typedef CastingLiftingPoint = {
  var id:String;
  var insertId:String;
  var holeId:String;
}

class CastingLiftingPoints {
  public static function records(points:Array<CastingLiftingPoint>):Array<Dynamic> return [for (point in points) {
    id:point.id, insertId:point.insertId, holeId:point.holeId
  }
  ];

  public static function snapshot(raw:Dynamic):Array<CastingLiftingPoint> {
    if (!Std.isOfType(raw, Array)) throw "Invalid source lifting-point list";
    var rows:Array<Dynamic> = cast raw, result:Array<CastingLiftingPoint> = [];
    var ids = new Map<String, Bool>(), holes = new Map<String, Bool>();
    for (row in rows) {
      CastingFeatureCodec.fields(row, ["id", "insertId", "holeId"]);
      var id = CastingRecordCodec.string(row, "id"), insert = CastingRecordCodec.string(row, "insertId");
      var hole = CastingRecordCodec.string(row, "holeId");
      for (value in [id, insert, hole]) CastingFeatureCodec.identifier(value);
      if (ids.exists(id)) throw "Duplicate source lifting point: " + id;
      var key = insert + "/" + hole;
      if (holes.exists(key)) throw "Duplicate source lifting thread: " + key;
      ids.set(id, true);
      holes.set(key, true);
      result.push({id: id, insertId: insert, holeId: hole});
    }
    result.sort((a, b) -> Reflect.compare(a.id, b.id));
    return result;
  }

  public static function validate(
    points:Array<CastingLiftingPoint>,
    inserts:Array<{id:String, role:InsertSpec.InsertRole, spec:InsertSpec}>
  ):Void {
    for (point in points) {
      var rows = inserts.filter(insert -> insert.id == point.insertId);
      if (rows.length != 1 || rows[0].role
        != InsertSpec.InsertRole.LiftingMount) throw "Lifting point needs a source lifting insert: " + point.id;
      var spec = rows[0].spec;
      var occupied = InsertAnchorage.InsertAnchorageCodec.occupiedHoleIds(spec.anchorage);
      var holes = spec.holes.filter(hole -> hole.id == point.holeId);
      if (holes.length != 1 || holes[0].thread == null
        || occupied.indexOf(point.holeId) >= 0) throw "Lifting point needs a source service thread: " + point.id;
    }
  }
}
