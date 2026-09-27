package app.editor;

import app.MateriaProjectRunner;
import app.Main.ReferenceEditorApp;
import haxe.Json;
import haxe.io.Path as UiPath;
import nativekit.ui.core.View;
import nativekit.ui.widgets.KeyedView;
import nativekit.ui.widgets.controls.Button;
import nativekit.ui.widgets.layout.Column;
import nativekit.ui.widgets.layout.Row;
import nativekit.ui.widgets.text.Text;
import sys.FileSystem;
import sys.io.File;

/** A project-declared editor panel and selection handler, hosted through JSON files.
 * The project executable owns its state and view description; the editor owns widgets.
 */
class ProjectUiExtension {
	public final projectPath:String;
	final artifact:String;
	final executable:String;
	final command:String;
	final temporaryDirectory:String;
	final actions:Array<Dynamic> = [];
	var response:Dynamic;
	var lastSelection:String = "";

	public static function open(projectPath:String, ?initialAction:String):Null<ProjectUiExtension> {
		var absolute = FileSystem.fullPath(projectPath);
		var root:Dynamic = Json.parse(File.getContent(absolute));
		var declaration:Dynamic = Reflect.field(root, "uiExtension");
		if (declaration == null) return null;
		if (Reflect.field(declaration, "kind") != "haxeon-command" ||
			Reflect.field(declaration, "protocol") != "materia.project-ui.v1")
			throw "Unsupported project UI extension";
		var projectRoot = UiPath.directory(absolute);
		var manifest = requiredPath(projectRoot, Reflect.field(declaration, "manifest"));
		var artifact = requiredPath(projectRoot, Reflect.field(declaration, "artifact"));
		var command = requiredText(declaration, "command");
		var installation = MateriaProjectRunner.installationRoot();
		var builder = UiPath.join([installation, "haxeon", "scripts", "haxeon"]);
		var executable = UiPath.join([installation, "haxeon", ".tools", "hashlink", "hl"]);
		// Haxeon's compiler service may inherit output descriptors after the build command
		// exits, so use inherited stdio instead of waiting for captured pipe EOF.
		if (Sys.command(builder, ["build", "--project=" + manifest]) != 0)
			throw "Could not build project UI extension";
		if (!FileSystem.exists(artifact)) throw "Project UI extension artifact was not built: " + artifact;
		var result = new ProjectUiExtension(absolute, artifact, executable, command);
		try {
			if (initialAction != null && initialAction.length > 0)
				result.actions.push({kind: "action", id: initialAction});
			result.evaluate();
			return result;
		} catch (error:Dynamic) {
			result.dispose();
			throw error;
		}
	}

	function new(projectPath:String, artifact:String, executable:String, command:String) {
		this.projectPath = projectPath;
		this.artifact = artifact;
		this.executable = executable;
		this.command = command;
		var tempRoot = Sys.getEnv("TMPDIR");
		temporaryDirectory = UiPath.join([tempRoot == null ? "/tmp" : tempRoot,
			"materia-project-ui-" + Sys.getPid() + "-" + Std.random(1000000000)]);
		FileSystem.createDirectory(temporaryDirectory);
	}

	public function action(id:String):Void {
		if (id == null || id.length == 0) throw "Project UI action needs an ID";
		actions.push({kind: "action", id: id});
		try evaluate() catch (error:Dynamic) { actions.pop(); throw error; }
	}

	public function select(sceneId:String):Void {
		if (sceneId == null || !StringTools.startsWith(sceneId, "project:")) {
			lastSelection = "";
			return;
		}
		if (sceneId == lastSelection) return;
		var previousSelection = lastSelection;
		lastSelection = sceneId;
		actions.push({kind: "select", id: sceneId});
		try evaluate() catch (error:Dynamic) {
			actions.pop();
			lastSelection = previousSelection;
			throw error;
		}
	}

	public function panel(onAction:String->Void):View {
		var panel:Dynamic = Reflect.field(response, "panel");
		var rows:Array<KeyedView> = [new KeyedView("heading",
			new Text(Std.string(Reflect.field(panel, "title"))))];
		var buttons:Array<KeyedView> = [];
		var declarations:Array<Dynamic> = Reflect.field(panel, "actions");
		if (declarations != null) for (entry in declarations) {
			var id:String = Reflect.field(entry, "id");
			var button = new Button(Std.string(Reflect.field(entry, "label")), null,
				function() onAction(id), "project-ui-" + id);
			button.enabled = Reflect.field(entry, "enabled") == true;
			buttons.push(new KeyedView(id, button));
		}
		var lines:Array<Dynamic> = Reflect.field(panel, "rows");
		var actionAfter:Null<Int> = Reflect.field(panel, "actionAfter");
		if (lines != null) for (index in 0...lines.length) {
			if (buttons.length > 0 && actionAfter == index)
				rows.push(new KeyedView("actions", new Row("project-ui-actions", buttons,
					ReferenceEditorApp.actionRowStyle())));
			var line = lines[index];
			rows.push(new KeyedView("line-" + index, new Text(Std.string(Reflect.field(line, "text")))));
		}
		if (buttons.length > 0 && (lines == null || actionAfter == null ||
			actionAfter < 0 || actionAfter >= lines.length))
			rows.push(new KeyedView("actions", new Row("project-ui-actions", buttons,
				ReferenceEditorApp.actionRowStyle())));
		return new Column("project-ui-panel", rows);
	}

	public function colours():Array<Dynamic> {
		var value:Dynamic = Reflect.field(response, "colours");
		return value == null ? [] : cast value;
	}

	public function requestedSelection():Null<String>
		return Reflect.field(response, "selectScene");

	public function dispose():Void {
		for (name in ["request.json", "response.json"]) {
			var path = UiPath.join([temporaryDirectory, name]);
			if (FileSystem.exists(path)) FileSystem.deleteFile(path);
		}
		if (FileSystem.exists(temporaryDirectory)) FileSystem.deleteDirectory(temporaryDirectory);
	}

	function evaluate():Void {
		var input = UiPath.join([temporaryDirectory, "request.json"]);
		var output = UiPath.join([temporaryDirectory, "response.json"]);
		File.saveContent(input, Json.stringify({protocol: "materia.project-ui.v1", actions: actions}));
		if (FileSystem.exists(output)) FileSystem.deleteFile(output);
		MateriaProjectRunner.runCommand(executable, [artifact, command, input, output],
			"Could not run project UI extension");
		if (!FileSystem.exists(output)) throw "Project UI extension did not write a response";
		var value:Dynamic = Json.parse(File.getContent(output));
		if (Reflect.field(value, "protocol") != "materia.project-ui.v1" ||
			Reflect.field(value, "panel") == null)
			throw "Project UI extension returned an invalid response";
		response = value;
	}

	static function requiredText(value:Dynamic, field:String):String {
		var result:Dynamic = Reflect.field(value, field);
		if (!Std.isOfType(result, String) || StringTools.trim(result).length == 0)
			throw 'Project UI extension needs "$field"';
		return result;
	}

	static function requiredPath(root:String, value:Dynamic):String {
		if (!Std.isOfType(value, String) || StringTools.trim(value).length == 0)
			throw "Project UI extension needs a file path";
		var path:String = value;
		return UiPath.isAbsolute(path) ? path : UiPath.join([root, path]);
	}
}
