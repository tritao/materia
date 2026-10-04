import cadkit.sheetmetal.FlatPattern;
import cadkit.sheetmetal.Sheet;
import cadkit.sheetmetal.SheetBlank;
import cadkit.sheetmetal.SheetBlankOrder;
import cadkit.sheetmetal.SheetDemoParts;
import cadkit.sheetmetal.SheetDiagnosis;
import cadkit.sheetmetal.SheetMaterial;
import cadkit.sheetmetal.SheetTolerance;
import cadkit.sheetmetal.WallOutline;
import materia.sheet.SheetProjectCodec;

/** Derived blanks enter ManufacturingKit's actual saved requirement format with authored identity and tolerances. */
class SheetBlankSmoke {
	static var checks:Int = 0;
	static function check(value:Bool, label:String):Void { checks++; if (!value) throw label; }
	static function near(a:Float, b:Float, label:String):Void { check(Math.abs(a - b) < 1e-8, label); }
	static function order(id:String, quantity:Int = 4):SheetBlankOrder {
		return {id: id, revision: 2, partId: "part", partRevision: 3, quantity: quantity,
			thicknessTolerance: new SheetTolerance(0.05, 0.1), widthTolerance: new SheetTolerance(0.1, 0.2), heightTolerance: new SheetTolerance(0.3, 0.4)};
	}
	public static function run():Int {
		checks = 0;
		var tray = SheetBlank.requirement(FlatPattern.of(SheetDemoParts.p5()), order("tray-stock")); var depth = 22.5 + Math.PI / 2 * 1.57;
		near(tray.blankWidth, 145 + 2 * depth, "hand-computed tray stock width"); near(tray.blankHeight, 95 + 2 * depth, "hand-computed tray stock height");
		near(tray.thickness, 1.5, "sheet supplies nominal stock thickness");
		check(tray.materialId == "mild-steel" && tray.lengthUnit == "mm", "sheet supplies material identity and units");
		check(tray.id == "tray-stock" && tray.partId == "part" && tray.revision == 2 && tray.partRevision == 3 && tray.quantity == 4, "caller supplies production identities and revisions");
		check(tray.allowedRotations.length == 2 && tray.allowedRotations[0] == 0 && tray.allowedRotations[1] == 90 && tray.directionConstraint == "free", "ungrained blank permits either stock axis");
		near(tray.thicknessToleranceMinus, 0.05, "authored thickness tolerance"); near(tray.widthTolerancePlus, 0.2, "authored width tolerance"); near(tray.heightToleranceMinus, 0.3, "authored height tolerance");
		var grained = new Sheet(new SheetMaterial("blank-fixture", 1.5, SheetDemoParts.materialA().bendRule, null, null, null, Math.PI / 4));
		var wall = grained.addWall(WallOutline.rectangle(20, 10)); var requirement = SheetBlank.requirement(FlatPattern.of(grained), order("grain-stock"));
		near(requirement.blankWidth, 30 / Math.sqrt(2), "arbitrary grain stock width"); near(requirement.blankHeight, 30 / Math.sqrt(2), "arbitrary grain stock height");
		check(requirement.allowedRotations.length == 1 && requirement.allowedRotations[0] == 0 && requirement.directionConstraint == "width-aligned", "exported grain axis is preserved on stock X");
		var project = SheetProjectCodec.empty(); project.requirements.push(requirement);
		project.customMaterials = [{id: "blank-fixture", name: "Assumed blank integration fixture", visual: {baseColor: [0.7, 0.7, 0.7], metallic: 0.5, roughness: 0.5}, physical: {density: 7850, spec: "assumed reference steel"}}];
		var restored = SheetProjectCodec.decode(SheetProjectCodec.encode(project)).requirements[0];
		check(restored.id == requirement.id && restored.partRevision == 3 && restored.materialId == "blank-fixture", "ManufacturingKit codec accepts derived requirements");
		near(restored.blankWidth, requirement.blankWidth, "manufacturing stock dimensions round trip");
		check(restored.directionConstraint == "width-aligned" && restored.allowedRotations.length == 1 && restored.allowedRotations[0] == 0, "manufacturing grain constraints round trip");
		var bad = order(" "); var refused = false;
		try { var invalid = SheetBlank.requirement(FlatPattern.of(grained), bad); } catch (error:SheetDiagnosis) { refused = error.code == "sheet.blank-order"; }
		check(refused, "missing production identity is diagnosed");
		var tooLoose:SheetBlankOrder = {id: "bad-tolerance", revision: 1, partId: "part", partRevision: 1, quantity: 1,
			thicknessTolerance: new SheetTolerance(1.5, 0), widthTolerance: new SheetTolerance(0, 0), heightTolerance: new SheetTolerance(0, 0)};
		refused = false;
		try { var invalid = SheetBlank.requirement(FlatPattern.of(grained), tooLoose); } catch (error:SheetDiagnosis) { refused = error.code == "sheet.blank-tolerance"; }
		check(refused, "lower tolerance cannot consume nominal stock thickness");
		refused = false;
		try { var invalid = SheetBlank.requirement(FlatPattern.of(grained), order("empty-job", 0)); } catch (error:SheetDiagnosis) { refused = error.code == "sheet.blank-order"; }
		check(refused, "empty production quantity is diagnosed");
		refused = false;
		try { var tolerance = new SheetTolerance(0, 0); tolerance.validate(Math.NaN, "width"); } catch (error:SheetDiagnosis) { refused = error.code == "sheet.blank-size"; }
		check(refused, "invalid nominal size has a distinct diagnosis from invalid tolerance");
		return checks;
	}
	static function main():Void { trace("Sheet blank smoke passed (" + run() + " assertions)"); }
}
