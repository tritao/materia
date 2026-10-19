package materia.tooling;

enum abstract ToolingLengthUnit(String) to String { var Millimetres = "mm"; }
enum abstract CoatedFace(String) to String {
    var PositiveZ = "positive-z"; var NegativeZ = "negative-z"; var Both = "both"; var Uncoated = "none";
}
enum abstract StockDirection(String) to String { var Free = "free"; var WidthAligned = "width-aligned"; }
enum abstract InsertRole(String) to String {
    var GenericMount = "generic-mount"; var GuideSeat = "guide-seat"; var BearingSeat = "bearing-seat";
    var ColumnMount = "column-mount"; var LevelingSeat = "leveling-seat"; var HatchMount = "hatch-mount";
}
enum abstract InsertFinish(String) to String { var GroundAfterCasting = "ground-after-casting"; }
enum abstract MachiningSide(String) to String { var PositiveZ = "positive-z"; var NegativeZ = "negative-z"; }
enum abstract ContactFace(String) to String { var None = "none"; var PositiveZ = "positive-z"; var NegativeZ = "negative-z"; }
