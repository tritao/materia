package nativekit.ui.widgets.docking;

import nativekit.ui.core.BuildContext;
import nativekit.ui.core.View;

typedef DockPanelBuilder = BuildContext->View;

/** Lazy view factory kept separate from the view-independent dock model. */
class DockPanelContent {
	public final panelId:String;
	public final build:DockPanelBuilder;

	public function new(panelId:String, build:DockPanelBuilder) {
		if (panelId == null || panelId.length == 0 || build == null)
			throw "Dock panel content requires a stable panel ID and builder";
		this.panelId = panelId;
		this.build = build;
	}
}
