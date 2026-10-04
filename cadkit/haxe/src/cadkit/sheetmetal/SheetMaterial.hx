package cadkit.sheetmetal;

/** Sheet stock and its explicit development rule. Strength data is optional until a process needs it. */
class SheetMaterial {
	public final id:String;
	public final thickness:Float;
	public final bendRule:Null<BendRule>;
	public final mechanical:Null<SheetMechanical>;
	public final defaultRadiusRatio:Null<Float>;
	public final minRadiusRatio:Null<Float>;

	public function new(id:String, thickness:Float, ?bendRule:BendRule, ?mechanical:SheetMechanical,
		?defaultRadiusRatio:Float, ?minRadiusRatio:Float) {
		BendDimensions.validate(thickness, 0, 0);
		if (id == null || id.length == 0)
			throw new SheetDiagnosis("sheet.invalid-material", "A material needs an identity");
		if (mechanical != null) {
			if (!Math.isFinite(mechanical.uts) || !Math.isFinite(mechanical.yield) || !Math.isFinite(mechanical.youngs)
				|| mechanical.uts <= 0 || mechanical.yield <= 0 || mechanical.youngs <= 0 || mechanical.yield > mechanical.uts)
				throw new SheetDiagnosis("sheet.invalid-mechanical-data", "Strengths and modulus must be positive MPa, with yield no greater than UTS");
			this.mechanical = {uts: mechanical.uts, yield: mechanical.yield, youngs: mechanical.youngs};
		} else
			this.mechanical = null;
		this.id = id;
		this.thickness = thickness;
		this.bendRule = bendRule;
		if ((defaultRadiusRatio != null && (!Math.isFinite(defaultRadiusRatio) || defaultRadiusRatio < 0))
			|| (minRadiusRatio != null && (!Math.isFinite(minRadiusRatio) || minRadiusRatio < 0))
			|| (defaultRadiusRatio != null && minRadiusRatio != null && defaultRadiusRatio < minRadiusRatio))
			throw new SheetDiagnosis("sheet.invalid-material-radius", "Radius ratios must be finite and nonnegative, with default no smaller than minimum");
		this.defaultRadiusRatio = defaultRadiusRatio;
		this.minRadiusRatio = minRadiusRatio;
	}
	public function defaultRadius():Float {
		if (defaultRadiusRatio == null)
			throw new SheetDiagnosis("sheet.no-default-radius", "Material has no default bend radius; choose a radius explicitly", [id]);
		return thickness * defaultRadiusRatio;
	}
}

/** Plain material properties, all in MPa. */
@:wire typedef SheetMechanical = {
	@:id(1) final uts:Float;
	@:id(2) final yield:Float;
	@:id(3) final youngs:Float;
}
