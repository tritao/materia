/** Focused folded router fixture without the rest of MachineKit smoke. */
import CncRouterPreview.CncRouterChecks;

class FoldedZRouterCheck {
	public static function main():Void {
		CncRouterChecks.runFoldedZ();
		var combined = new CncRouter(true, true);
		var geometry = combined.describe();
		if (geometry.mechanical.elasticNetworks == null || geometry.mechanical.elasticNetworks.length != 1 ||
			combined.check().hasErrors()) throw "The folded Z stage must combine with the router's X/Y carriage belts";
	}
}
