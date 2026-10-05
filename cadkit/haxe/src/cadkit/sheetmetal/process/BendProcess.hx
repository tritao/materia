package cadkit.sheetmetal.process;

import cadkit.sheetmetal.SheetMaterial;

/** A process predicts one bend from stock, tooling, signed target angle and bend length in mm. */
interface BendProcess {
	function predict(material:SheetMaterial, tooling:ProcessTooling, targetAngle:Float, length:Float):BendProcessResult;
}
