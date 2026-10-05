package cadkit.sheetmetal;

import materia.project.MaterialDef;
import materia.project.MaterialDef.MaterialMechanical;
import materia.project.MaterialLibrary;

/** Sheet stock and its explicit development rule. Strength data is optional until a process needs it. */
class SheetMaterial {
	public final id:String;
	public final thickness:Float;
	public final bendRule:Null<BendRule>;
	public final mechanical:Null<MaterialMechanical>;
	public final defaultRadiusRatio:Null<Float>;
	public final minRadiusRatio:Null<Float>;
	/** Undirected rolling axis in the root wall frame, radians modulo pi. */
	public final grainAngle:Null<Float>;

	/** Copy library defaults into authored stock; rolling direction belongs to the part's root frame. */
	public static function fromDefinition(definition:MaterialDef, thickness:Float, ?grainAngle:Float):SheetMaterial {
		MaterialLibrary.validateSheetProperties(definition);
		var defaults = definition.sheet;
		if (defaults == null)
			throw new SheetDiagnosis("sheet.not-sheet-material", "Material has no sheet development defaults", [definition.id]);
		if (defaults.grain && grainAngle == null)
			throw new SheetDiagnosis("sheet.no-grain-direction", "Choose the rolling direction in the root wall frame", [definition.id]);
		if (!defaults.grain && grainAngle != null)
			throw new SheetDiagnosis("sheet.material-has-no-grain", "Material has no rolling-direction constraint", [definition.id]);
		var properties = definition.mechanical;
		var mechanical:Null<MaterialMechanical> = properties == null ? null
			: {uts: properties.uts, yield: properties.yield, youngs: properties.youngs};
		return new SheetMaterial(definition.id, thickness,
			BendRule.KFactor(new KFactorTable([for (row in defaults.kFactorTable) row.ratio], [for (row in defaults.kFactorTable) row.k])),
			mechanical, defaults.defaultRadiusRatio, defaults.minRadiusRatio, grainAngle);
	}

	public function new(id:String, thickness:Float, ?bendRule:BendRule, ?mechanical:MaterialMechanical,
		?defaultRadiusRatio:Float, ?minRadiusRatio:Float, ?grainAngle:Float) {
		BendDimensions.validate(thickness, 0, 0);
		if (id == null || id.length == 0)
			throw new SheetDiagnosis("sheet.invalid-material", "A material needs an identity");
		if (mechanical != null) {
			if (!Math.isFinite(mechanical.uts) || !Math.isFinite(mechanical.yield) || !Math.isFinite(mechanical.youngs)
				|| mechanical.uts <= 0 || mechanical.yield <= 0 || mechanical.youngs <= mechanical.yield || mechanical.yield > mechanical.uts)
				throw new SheetDiagnosis("sheet.invalid-mechanical-data", "Strengths must be positive MPa, with yield no greater than UTS and elastic modulus above yield");
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
		if (grainAngle == null) this.grainAngle = null;
		else {
			if (!Math.isFinite(grainAngle)) throw new SheetDiagnosis("sheet.invalid-grain", "Grain angle must be finite radians", [id]);
			var angle = grainAngle % Math.PI; this.grainAngle = angle < 0 ? angle + Math.PI : angle;
		}
	}
	public function defaultRadius(?angle:Float):Float {
		if (bendRule != null) switch (bendRule) {
			case Process(setup):
				if (angle == null)
					throw new SheetDiagnosis("sheet.process-angle-required", "A tooling-derived radius needs its target bend angle", [id]);
				return setup.predict(this, angle, 1).innerRadius;
			case KFactor(_): {}
		}
		if (defaultRadiusRatio == null)
			throw new SheetDiagnosis("sheet.no-default-radius", "Material has no default bend radius; choose a radius explicitly", [id]);
		return thickness * defaultRadiusRatio;
	}
}
