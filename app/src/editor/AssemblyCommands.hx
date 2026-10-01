package app.editor;

import app.Main.ReferenceEditorApp;
import app.MatePickTool;
import materia.assembly.AssemblyDefinition.AssemblyMateKind;
import nativekit.ui.core.Command;

/** Assembly mate commands: each starts picking two faces for a mate of its kind in the perspective viewport. */
@:access(app.Main.ReferenceEditorApp)
class AssemblyCommands {
	public static function install(app:ReferenceEditorApp):Void {
		registerMate(app, "assembly.mate-planar", "Mate faces: Planar", AssemblyMateKind.Planar);
		registerMate(app, "assembly.mate-coaxial", "Mate faces: Coaxial", AssemblyMateKind.Coaxial);
		registerMate(app, "assembly.mate-parallel", "Mate faces: Parallel", AssemblyMateKind.Parallel);
		registerMate(app, "assembly.mate-perpendicular", "Mate faces: Perpendicular", AssemblyMateKind.Perpendicular);
	}

	static function registerMate(app:ReferenceEditorApp, id:String, label:String, kind:AssemblyMateKind):Void {
		app.commands.register(new Command(id, label, function() {
			var viewport = app.perspectiveViewport;
			if (viewport == null) return;
			viewport.beginMatePick(new MatePickTool(app.session, kind));
			app.commands.refresh();
		}, null, function() return app.canEditObjects() && app.perspectiveViewport != null && app.session.canMateFaces()));
	}
}
