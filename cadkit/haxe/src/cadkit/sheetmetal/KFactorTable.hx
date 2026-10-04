package cadkit.sheetmetal;

/** Published or calibrated neutral-axis positions. Values are stored in the ANSI convention. */
class KFactorTable {
	final ratios:Array<Float>;
	final factors:Array<Float>;

	public function new(ratios:Array<Float>, factors:Array<Float>, din:Bool = false) {
		if (ratios.length == 0 || ratios.length != factors.length)
			throw new SheetDiagnosis("sheet.invalid-k-table", "R/T and K columns must have the same nonzero length");
		this.ratios = ratios.copy();
		this.factors = [];
		for (i in 0...ratios.length) {
			var ratio = ratios[i];
			var k = factors[i] * (din ? 0.5 : 1);
			if (!Math.isFinite(ratio) || ratio < 0 || (i > 0 && ratio <= ratios[i - 1]))
				throw new SheetDiagnosis("sheet.invalid-k-table", "R/T must be finite, nonnegative and strictly increasing");
			if (!Math.isFinite(k) || k < 0 || k > 0.5)
				throw new SheetDiagnosis("sheet.invalid-k-table", "ANSI K must be between zero and one half");
			this.factors.push(k);
		}
	}

	public function at(ratio:Float):Float {
		if (!Math.isFinite(ratio) || ratio < 0)
			throw new SheetDiagnosis("sheet.invalid-radius-ratio", "R/T must be finite and nonnegative");
		if (ratio <= ratios[0])
			return factors[0];
		for (i in 1...ratios.length)
			if (ratio <= ratios[i]) {
				var fraction = (ratio - ratios[i - 1]) / (ratios[i] - ratios[i - 1]);
				return factors[i - 1] + fraction * (factors[i] - factors[i - 1]);
			}
		return factors[factors.length - 1];
	}

	/** Canonical ANSI rows for saved stock specifications; callers cannot mutate the table. */
	public function rows():Array<KFactorRow> {
		return [for (i in 0...ratios.length) {ratio: ratios[i], k: factors[i]}];
	}
}

/** Canonical ANSI rule row, also used by the typed sheet stock record. */
@:wire typedef KFactorRow = {
	@:id(1) var ratio:Float;
	@:id(2) var k:Float;
}
