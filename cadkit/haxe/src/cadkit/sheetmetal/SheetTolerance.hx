package cadkit.sheetmetal;

/** Authored asymmetric dimensional acceptance, in millimetres. */
class SheetTolerance {
	public final minus:Float;
	public final plus:Float;
	public function new(minus:Float, plus:Float) {
		if (!Math.isFinite(minus) || !Math.isFinite(plus) || minus < 0 || plus < 0)
			throw new SheetDiagnosis("sheet.blank-tolerance", "Blank tolerances must be finite nonnegative millimetres");
		this.minus = minus; this.plus = plus;
	}
	public function validate(nominal:Float, name:String):Void {
		if (!Math.isFinite(nominal) || nominal <= 0)
			throw new SheetDiagnosis("sheet.blank-size", "Blank " + name + " must be finite positive millimetres");
		if (minus >= nominal)
			throw new SheetDiagnosis("sheet.blank-tolerance", "Lower tolerance consumes the nominal " + name);
	}
}
