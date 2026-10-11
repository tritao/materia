package cadkit.sheetmetal;

/** Actual bend angles, independent of the authored targets; evaluation preserves the blank. */
class ManufacturedSheetState implements SheetAngles {
	final angles:SheetState;
	public function new(source:SheetAngles) {
		angles = new SheetState();
		for (id in source.ids()) angles.set(id, source.angle(id));
	}
	public function set(bend:Int, angle:Float):ManufacturedSheetState {
		var result = angles.set(bend, angle);
		return this;
	}
	public function angle(bend:Int):Float return angles.angle(bend);
	public function ids():Array<Int> return angles.ids();
}
