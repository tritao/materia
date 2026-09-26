package machinekit.transmission;

import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.component.ComponentDetail;
import machinekit.component.ConnectorRole;
import machinekit.component.MachineComponent;
import machinekit.component.Solids;

/** Roller chain sprocket with a simplified tooth outline (straight flanks between root and tip
 * radii, not the true ANSI B29.1 seating-curve profile), extruded along local +Z.
 * CAD frame: front face at z=0, back face at z=thickness, matching `DeepGrooveBearing`.
 * Connectors: `front`, `back` (faces) and `axis` (mid-thickness), all with +Y along +Z.
 */
class Sprocket extends MachineComponent {
	public final pitch:Float;
	public final teeth:Int;
	public final boreDiameter:Float;
	public final thickness:Float;
	public final pitchDiameter:Float;
	public final outsideDiameter:Float;
	public final rootDiameter:Float;

	public function new(pitch:Float, teeth:Int, boreDiameter:Float, thickness:Float) {
		if (!(pitch > 0)) throw "Sprocket needs a positive chain pitch";
		if (teeth < 8) throw "Sprocket needs at least 8 teeth";
		if (!(boreDiameter > 0)) throw "Sprocket needs a positive bore diameter";
		if (!(thickness > 0)) throw "Sprocket needs a positive thickness";
		var pitchDia = pitch / Math.sin(Math.PI / teeth);
		var outsideDia = pitchDia + 0.8 * pitch;
		var rootDia = pitchDia - pitch;
		if (!(rootDia > boreDiameter))
			throw "Sprocket root diameter must clear the bore; use fewer teeth or a smaller bore";
		var pitchText = formatDecimal(pitch);
		super('SPROCKET-P$pitchText-${teeth}T', 'Sprocket, $pitchText mm pitch, ${teeth} teeth', "steel");
		this.pitch = pitch;
		this.teeth = teeth;
		this.boreDiameter = boreDiameter;
		this.thickness = thickness;
		pitchDiameter = pitchDia;
		outsideDiameter = outsideDia;
		rootDiameter = rootDia;
		addConnector("front", Face, Solids.axial(0, 0, 0));
		addConnector("axis", Axis, Solids.axial(0, 0, thickness / 2));
		addConnector("back", Face, Solids.axial(0, 0, thickness));
	}

	override public function geometry(detail:ComponentDetail = Preview):Part {
		var body = Solids.prism(profile(), 0, thickness);
		return Solids.cut(body, [Solids.cylinder(boreDiameter / 2, -0.1, thickness + 0.1)]);
	}

	/** Rounds to 3 decimal places and trims trailing zeros; avoids printing a binary float's full
	 * imprecise expansion (e.g. 12.7) for designations built from non-power-of-two pitches.
	 */
	static function formatDecimal(value:Float):String {
		var scaled:Int = Math.round(value * 1000);
		var whole:Int = Std.int(scaled / 1000);
		var frac = scaled - whole * 1000;
		if (frac == 0) return Std.string(whole);
		var digits = Std.string(frac);
		while (digits.length < 3) digits = "0" + digits;
		while (digits.length > 1 && digits.charAt(digits.length - 1) == "0") digits = digits.substr(0, digits.length - 1);
		return '$whole.$digits';
	}

	function profile():Array<Vector> {
		var rf = rootDiameter / 2, ra = outsideDiameter / 2;
		var angleStep = 2 * Math.PI / teeth;
		var halfRoot = 0.35 * angleStep / 2, halfTip = 0.2 * angleStep / 2;
		var points:Array<Vector> = [];
		for (i in 0...teeth) {
			var center = i * angleStep;
			points.push(new Vector(rf * Math.cos(center - halfRoot), rf * Math.sin(center - halfRoot)));
			points.push(new Vector(ra * Math.cos(center - halfTip), ra * Math.sin(center - halfTip)));
			points.push(new Vector(ra * Math.cos(center + halfTip), ra * Math.sin(center + halfTip)));
			points.push(new Vector(rf * Math.cos(center + halfRoot), rf * Math.sin(center + halfRoot)));
		}
		return points;
	}
}
