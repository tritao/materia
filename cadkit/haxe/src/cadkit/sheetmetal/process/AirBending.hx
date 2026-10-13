package cadkit.sheetmetal.process;

import cadkit.sheetmetal.SheetDiagnosis;
import cadkit.sheetmetal.SheetMaterial;
import materia.project.MaterialDef.MaterialMechanical;

/** Assumed air-bending formulas. Calibration and catalogue ratings remain separate inputs. */
class AirBending implements BendProcess {
	public final calibration:CalibrationTable;
	public function new(?calibration:CalibrationTable) {
		this.calibration = calibration == null ? new CalibrationTable([]) : calibration;
	}

	public function predict(material:SheetMaterial, tooling:ProcessTooling, targetAngle:Float, length:Float, ?lot:String):BendProcessResult {
		if (!Math.isFinite(targetAngle) || Math.abs(targetAngle) <= 0 || Math.abs(targetAngle) >= Math.PI)
			throw new SheetDiagnosis("sheet.invalid-process-angle", "Air bending needs a nonzero target below 180 degrees");
		if (!Math.isFinite(length) || length <= 0)
			throw new SheetDiagnosis("sheet.invalid-bend-length", "Bend length must be positive mm");
		var mechanical = material.mechanical;
		if (mechanical == null)
			throw new SheetDiagnosis("sheet.no-mechanical-data", "Air bending needs UTS, yield and Young's modulus", [material.id]);
		var thickness = material.thickness;
		// Assumed natural final radius and 90-degree minimum flange, pending measured calibration.
		var naturalRadius = 0.16 * tooling.vWidth * mechanical.uts / 410;
		var innerRadius = Math.max(naturalRadius, tooling.punchRadius);
		var entry:Null<CalibrationEntry> = lot == null ? null : calibration.find(lot, thickness, tooling, targetAngle);
		if (entry != null) innerRadius = entry.innerRadius;
		var loadedRadius = loadRadius(innerRadius, thickness, mechanical);
		// A physical punch cannot impose a radius smaller than its tip. Its loaded radius governs.
		if (loadedRadius < tooling.punchRadius) {
			if (entry != null)
				throw new SheetDiagnosis("sheet.calibration-tool-radius", "Measured radius requires a loaded radius smaller than the punch tip", [tooling.punchId, tooling.dieId]);
			loadedRadius = tooling.punchRadius;
			innerRadius = unloadRadius(loadedRadius, thickness, mechanical);
		}
		var magnitude = Math.abs(targetAngle) * (innerRadius + thickness / 2) / (loadedRadius + thickness / 2);
		var maximum = Math.PI - Math.max(tooling.punchAngle, tooling.dieAngle);
		if (magnitude >= maximum)
			throw new SheetDiagnosis("sheet.tool-angle-limit", "The loaded bend reaches the punch or die included-angle limit", [tooling.punchId, tooling.dieId]);
		var loadedAngle = targetAngle < 0 ? -magnitude : magnitude;
		var analyticDepth = depth(thickness, loadedRadius, tooling, magnitude);
		var depthOffset = entry == null ? 0.0 : entry.depthOffset;
		var force = 650 * thickness * thickness * (length / 1000) / tooling.vWidth * mechanical.uts / 450;
		if (!Math.isFinite(force) || !Math.isFinite(analyticDepth + depthOffset))
			throw new SheetDiagnosis("sheet.process-range", "Process dimensions exceed the finite calculation range");
		return {
			innerRadius: innerRadius, loadedRadius: loadedRadius,
			bendAllowance: Math.abs(targetAngle) * (innerRadius + thickness / 2),
			targetAngle: targetAngle, loadedAngle: loadedAngle, overbend: loadedAngle - targetAngle,
			depth: analyticDepth + depthOffset, analyticDepth: analyticDepth,
			depthOffset: depthOffset, calibrated: entry != null,
			force: force,
			minimumFlange: minimumFlange(tooling.vWidth),
			radiusBend: tooling.punchRadius > naturalRadius,
			recommendedOpening: tooling.vWidth >= 6 * thickness && tooling.vWidth <= 12 * thickness
		};
	}

	/** Assumed thin-stock default V=8T; thicker stock requires an explicit tooling choice. */
	public static function defaultOpening(thickness:Float):Float {
		if (!Math.isFinite(thickness) || thickness <= 0)
			throw new SheetDiagnosis("sheet.invalid-thickness", "Thickness must be positive mm");
		if (thickness > 3)
			throw new SheetDiagnosis("sheet.no-default-tooling", "Choose tooling explicitly for stock thicker than 3 mm");
		return 8 * thickness;
	}

	/** Assumed minimum flange for 90-degree air bending, in mm. */
	public static function minimumFlange(opening:Float):Float {
		if (!Math.isFinite(opening) || opening <= 0)
			throw new SheetDiagnosis("sheet.invalid-die-opening", "Die opening must be positive mm");
		return 0.7 * opening;
	}

	/** Elastic-plastic radius relation; refuse the nonphysical branch instead of extrapolating. */
	public static function unloadRadius(loadedRadius:Float, thickness:Float, mechanical:MaterialMechanical):Float {
		if (!Math.isFinite(loadedRadius) || loadedRadius <= 0 || !Math.isFinite(thickness) || thickness <= 0
			|| !Math.isFinite(mechanical.yield) || mechanical.yield <= 0
			|| !Math.isFinite(mechanical.youngs) || mechanical.youngs <= mechanical.yield)
			throw new SheetDiagnosis("sheet.invalid-springback-input", "Springback needs positive radii, thickness and elastic modulus above yield");
		var ratio = loadedRadius * mechanical.yield / (mechanical.youngs * thickness);
		if (ratio >= 0.5)
			throw new SheetDiagnosis("sheet.springback-domain", "The bend is outside the plastic radius relation");
		// Factored form avoids cancellation close to the physical branch's double root at 1/2.
		var remaining = 1 - 2 * ratio;
		var recovery = remaining * remaining * (1 + ratio);
		var result = loadedRadius / recovery;
		if (!Math.isFinite(result))
			throw new SheetDiagnosis("sheet.springback-domain", "Released radius exceeds the finite plastic radius relation");
		return result;
	}

	/** Invert the monotone physical branch with a bracketed solve, without a guessed iteration seed. */
	public static function loadRadius(finalRadius:Float, thickness:Float, mechanical:MaterialMechanical):Float {
		if (!Math.isFinite(finalRadius) || finalRadius <= 0 || !Math.isFinite(thickness) || thickness <= 0
			|| !Math.isFinite(mechanical.yield) || mechanical.yield <= 0
			|| !Math.isFinite(mechanical.youngs) || mechanical.youngs <= mechanical.yield)
			throw new SheetDiagnosis("sheet.invalid-springback-input", "Springback needs positive radii, thickness and elastic modulus above yield");
		var low = 0.0;
		var high = Math.min(finalRadius, 0.499999999 * mechanical.youngs * thickness / mechanical.yield);
		if (unloadRadius(high, thickness, mechanical) < finalRadius)
			throw new SheetDiagnosis("sheet.springback-domain", "Released radius cannot be bracketed on the physical branch");
		for (iteration in 0...80) {
			var middle = (low + high) / 2;
			if (unloadRadius(middle, thickness, mechanical) > finalRadius) high = middle;
			else low = middle;
		}
		return (low + high) / 2;
	}

	/** Punch-bottom depth below the die top; shoulder centres are at z=-rd, x=+-V/2. */
	public static function depth(thickness:Float, loadedRadius:Float, tooling:ProcessTooling, loadedAngle:Float):Float {
		if (!Math.isFinite(thickness) || thickness <= 0 || !Math.isFinite(loadedRadius) || loadedRadius <= 0
			|| loadedRadius < tooling.punchRadius || !Math.isFinite(loadedAngle) || loadedAngle < 0 || loadedAngle >= Math.PI)
			throw new SheetDiagnosis("sheet.invalid-depth-input", "Depth needs positive thickness and radius above the tip, and an angle below 180 degrees");
		var beta = loadedAngle / 2;
		var separation = loadedRadius + thickness + tooling.shoulderRadius;
		var halfWidth = tooling.vWidth / 2;
		var height = (separation - halfWidth * Math.sin(beta)) / Math.cos(beta);
		var tangentLength = halfWidth * Math.cos(beta) - height * Math.sin(beta);
		if (tangentLength <= 0 || separation >= halfWidth)
			throw new SheetDiagnosis("sheet.tool-contact-limit", "The tooling cannot provide separated flange contact points", [tooling.punchId, tooling.dieId]);
		return tooling.shoulderRadius + loadedRadius - height;
	}
}
