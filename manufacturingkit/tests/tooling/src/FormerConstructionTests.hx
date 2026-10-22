import materia.tooling.casting.FormerConstruction;
import materia.tooling.casting.MoldPanels;
import materia.tooling.ToolingRecipe;
import materia.tooling.ToolingCodec;

class FormerConstructionTests {
	public static function run(recipe:ToolingRecipe):Void {
		var assertions=0;
		function check(ok:Bool,message:String):Void {assertions++;if (!ok) throw message;}
		var stocks=recipe.former.stocks;
		var thin=FormerConstruction.solidStock(stocks,8,100,100);
		var faced=FormerConstruction.solidStock(stocks,20,100,100);
		check(thin!=null && thin.measuredThickness==17.8,"shallow former selects the declared panel-family sheet");
		check(faced!=null && faced.measuredThickness==25,"20 mm former selects the next declared sheet to face");
		check(FormerConstruction.solidStock(stocks,120,111,110)==null,"tall former selects panel construction instead of a glued stack");
		check(FormerConstruction.solidStock(stocks,8,5000,100)==null,"a solid blank must fit the declared sheet footprint");
		var reversed=stocks.copy();reversed.reverse();
		check(ToolingCodec.canonical(FormerConstruction.solidStock(reversed,20,100,100))==ToolingCodec.canonical(faced),
			"stock declaration order cannot select a different former blank");
		var shellRecipe=ToolingCodec.recipeSnapshot(recipe);
		shellRecipe.minimumLigament=2;
		var shell=MoldPanels.formerBox("former-test",111,110,120,shellRecipe);
		check(shell.members.length==5 && shell.joints.length==6,"hollow former has one cap and four repairable rabbeted walls");
		check(shell.members.filter(member -> member.panel.id.indexOf("bottom")>=0).length==0,"withdrawal side has no closing panel");
		for (joint in shell.joints)
			check(joint.closure==materia.tooling.panel.PanelJoints.PanelClosure.InternalPocketScrews,
				"former closure uses the shared removable bolt-joint record");
		Sys.println('FormerConstructionTests: $assertions assertions passed');
	}
}
