package machinekit.printing;

/** Feedstock and its process-response parameters for a chosen extrusion setup.
 * Diameter is metres, minimum extrusion temperature is Celsius and melt lag is
 * seconds. Lag depends on the material/hotend combination, not just polymer name.
 * This specification does not imply a verified material or hardware compatibility.
 */
class PrintMaterialSpec {
	public final name:String;
	public final filamentDiameter:Float;
	/** Zero explicitly disables the cold-extrusion threshold. */
	public final minExtrusionTemperature:Float;
	public final meltLag:Float;

	public function new(name:String,filamentDiameter:Float,minExtrusionTemperature:Float,meltLag:Float) {
		if (name==null || StringTools.trim(name).length==0) throw "Print material needs a name";
		if (!Math.isFinite(filamentDiameter) || filamentDiameter<=0) throw "Material filament diameter must be finite and positive";
		for (value in [minExtrusionTemperature,meltLag])
			if (!Math.isFinite(value) || value<0) throw "Material temperature threshold and lag must be finite and nonnegative";
		this.name=name;this.filamentDiameter=filamentDiameter;
		this.minExtrusionTemperature=minExtrusionTemperature;this.meltLag=meltLag;
	}

	/** Retains the printer example's existing unmeasured assumptions; not a vendor material profile. */
	public static function assumedPla175():PrintMaterialSpec
		return new PrintMaterialSpec("Generic PLA / V6 setup (assumed)",0.00175,170.0,0.04);
}
