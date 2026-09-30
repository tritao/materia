import nativekit.ui.properties.PropertyOption;
import nativekit.ui.properties.PropertyType;
import nativekit.ui.properties.PropertyValue;
import nativekit.ui.properties.PropertyValueTools;
import nativekit.ui.settings.SettingDefinition;
import nativekit.ui.settings.SettingOptions;
import nativekit.ui.settings.SettingsRegistry;
import nativekit.ui.settings.SettingsStore;
import sys.FileSystem;
import sys.io.File;

/** Settings core: definitions, defaults, validation, change events and saving. */
class SettingsTests {
	static final DIRECTORY = "build/settings-test";

	public static function main():Int {
		try {
			definitions();
			registry();
			values();
			listeners();
			persistence();
			damagedFiles();
			Sys.println("Settings tests passed");
			return 0;
		} catch (error:Dynamic) {
			Sys.println('Settings tests failed: $error');
			return 1;
		}
	}

	static function check(condition:Bool, message:String):Void
		if (!condition)
			throw message;

	static function throws(work:Void->Void, message:String):Void {
		var thrown = false;
		try {
			work();
		} catch (_:Dynamic) {
			thrown = true;
		}
		check(thrown, message);
	}

	static function range(minimum:Float, maximum:Float):SettingOptions {
		var options = new SettingOptions();
		options.minimum = minimum;
		options.maximum = maximum;
		return options;
	}

	static function choices(keys:Array<String>):SettingOptions {
		var options = new SettingOptions();
		options.options = [for (key in keys) new PropertyOption(key, SettingDefinition.labelFor(key))];
		return options;
	}

	/** The editor's sample schema, shared by the value and file tests. */
	static function sampleRegistry():SettingsRegistry {
		var result = new SettingsRegistry();
		result.define("interface/editor/fonts/main_font_size", PropertyType.Int, PropertyValue.Int(14), range(8, 48));
		result.define("interface/editor/appearance/custom_display_scale", PropertyType.Float, PropertyValue.Float(1.0),
			range(0.5, 4.0));
		result.define("interface/editor/appearance/expand_to_title", PropertyType.Bool, PropertyValue.Bool(true));
		result.define("interface/editor/docks/dock_tab_style", PropertyType.Enum, PropertyValue.Enum("text_and_icon"),
			choices(["text_only", "icon_only", "text_and_icon"]));
		result.define("filesystem/directories/default_project_path", PropertyType.Text, PropertyValue.Text(""));
		return result;
	}

	static function definitions():Void {
		var size = new SettingDefinition("interface/editor/fonts/main_font_size", PropertyType.Int, PropertyValue.Int(14));
		check(size.label == "Main Font Size", "labels come from the last path segment");
		check(size.segments.length == 4 && size.segments[1] == "editor", "paths split into segments");
		var named = new SettingOptions();
		named.label = "TLS Certificates";
		check(new SettingDefinition("network/tls/certificates", PropertyType.Text, PropertyValue.Text(""), named).label
			== "TLS Certificates", "an explicit label wins");

		throws(() -> new SettingDefinition("Interface/Editor", PropertyType.Bool, PropertyValue.Bool(false)),
			"paths are lowercase");
		throws(() -> new SettingDefinition("interface//editor", PropertyType.Bool, PropertyValue.Bool(false)),
			"paths have no empty segments");
		throws(() -> new SettingDefinition("a/b", PropertyType.Int, PropertyValue.Bool(false)),
			"the default must have the setting's type");
		throws(() -> new SettingDefinition("a/b", PropertyType.Int, PropertyValue.Int(50), range(0, 10)),
			"the default must be in range");
		throws(() -> new SettingDefinition("a/b", PropertyType.Enum, PropertyValue.Enum("x")),
			"enum settings need choices");
		throws(() -> new SettingDefinition("a/b", PropertyType.Enum, PropertyValue.Enum("z"), choices(["x", "y"])),
			"an enum default must be one of the choices");
		throws(() -> new SettingDefinition("a/b", PropertyType.Custom("color"), PropertyValue.Custom("color", 0)),
			"custom types are not settings");

		var scale = new SettingDefinition("a/scale", PropertyType.Float, PropertyValue.Int(1));
		check(PropertyValueTools.same(scale.defaultValue, PropertyValue.Float(1.0)), "an Int default of a Float setting becomes a Float");
		check(scale.validate(PropertyValue.Float(Math.NaN)) != null, "NaN is refused");
	}

	static function registry():Void {
		var registry = new SettingsRegistry();
		var first = registry.define("interface/editor/fonts/main_font_size", PropertyType.Int, PropertyValue.Int(14));
		var again = registry.define("interface/editor/fonts/main_font_size", PropertyType.Int, PropertyValue.Int(14));
		check(first == again, "an identical redefinition returns the first definition");
		throws(() -> registry.define("interface/editor/fonts/main_font_size", PropertyType.Float, PropertyValue.Float(14)),
			"a redefinition with another type is refused");
		throws(() -> registry.define("interface/editor/fonts/main_font_size", PropertyType.Int, PropertyValue.Int(12)),
			"a redefinition with another default is refused");
		throws(() -> registry.define("interface/editor/fonts", PropertyType.Int, PropertyValue.Int(1)),
			"a setting cannot sit on another setting's group");
		registry.define("interface/editor/fonts_extra/size", PropertyType.Int, PropertyValue.Int(1));
		registry.define("interface/inspector/max_array_items", PropertyType.Int, PropertyValue.Int(10));
		check(registry.under("interface/editor").length == 2, "under() matches whole segments");
		check(registry.under("interface/editor/fonts").length == 1, "fonts does not match fonts_extra");
		check(registry.under("").length == 3 && registry.all()[2].path == "interface/inspector/max_array_items",
			"everything, in definition order");
	}

	static function values():Void {
		var store = new SettingsStore(sampleRegistry());
		check(store.getInt("interface/editor/fonts/main_font_size") == 14, "values start at their defaults");
		check(store.isDefault("interface/editor/fonts/main_font_size"), "untouched settings are at their default");

		check(store.set("interface/editor/fonts/main_font_size", PropertyValue.Int(18)) == null, "a valid value is stored");
		check(store.getInt("interface/editor/fonts/main_font_size") == 18, "the new value is read back");
		check(!store.isDefault("interface/editor/fonts/main_font_size"), "a changed setting is no longer at its default");

		check(store.set("interface/editor/fonts/main_font_size", PropertyValue.Int(100)) != null, "out of range is refused");
		check(store.set("interface/editor/fonts/main_font_size", PropertyValue.Text("big")) != null, "a wrong type is refused");
		check(store.getInt("interface/editor/fonts/main_font_size") == 18, "a refused value leaves the setting alone");

		check(store.set("interface/editor/docks/dock_tab_style", PropertyValue.Enum("sideways")) != null,
			"an unknown choice is refused");
		store.set("interface/editor/docks/dock_tab_style", PropertyValue.Enum("icon_only"));
		check(store.getString("interface/editor/docks/dock_tab_style") == "icon_only", "enum values read back as their key");

		store.set("interface/editor/appearance/custom_display_scale", PropertyValue.Int(2));
		check(store.getFloat("interface/editor/appearance/custom_display_scale") == 2.0, "an Int given to a Float setting is stored as a Float");

		store.set("interface/editor/fonts/main_font_size", PropertyValue.Int(14));
		check(store.isDefault("interface/editor/fonts/main_font_size"), "setting the default value clears the override");

		store.reset("interface/editor/docks/dock_tab_style");
		check(store.getString("interface/editor/docks/dock_tab_style") == "text_and_icon", "reset restores the default");

		store.set("interface/editor/appearance/expand_to_title", PropertyValue.Bool(false));
		store.set("filesystem/directories/default_project_path", PropertyValue.Text("/work"));
		store.resetUnder("interface");
		check(store.getBool("interface/editor/appearance/expand_to_title")
			&& store.getFloat("interface/editor/appearance/custom_display_scale") == 1.0, "resetUnder resets the group");
		check(store.getString("filesystem/directories/default_project_path") == "/work", "and nothing outside it");

		throws(() -> store.get("interface/editor/nonexistent"), "reading an undefined setting is a programming error");
		throws(() -> store.getBool("interface/editor/fonts/main_font_size"), "a typed getter checks the type");
	}

	static function listeners():Void {
		var store = new SettingsStore(sampleRegistry());
		var heard:Array<String> = [];
		var everything = 0;
		var stop = store.onChanged("interface/editor/fonts", path -> heard.push(path));
		store.onChanged("", _ -> everything++);
		var before = store.revision;

		store.set("interface/editor/fonts/main_font_size", PropertyValue.Int(20));
		store.set("interface/editor/fonts/main_font_size", PropertyValue.Int(20));
		store.set("interface/editor/appearance/expand_to_title", PropertyValue.Bool(false));
		store.set("interface/editor/fonts/main_font_size", PropertyValue.Int(99));
		check(heard.length == 1 && heard[0] == "interface/editor/fonts/main_font_size",
			"a listener hears real changes under its prefix only");
		check(everything == 2 && store.revision == before + 2, "an empty prefix hears every change; revision counts them");

		store.reset("interface/editor/appearance/custom_display_scale");
		check(everything == 2, "resetting a setting already at its default is not a change");

		stop();
		store.reset("interface/editor/fonts/main_font_size");
		check(heard.length == 1 && everything == 3, "a removed listener hears nothing more");
	}

	static function prepareFile(name:String):String {
		if (!FileSystem.exists("build"))
			FileSystem.createDirectory("build");
		if (!FileSystem.exists(DIRECTORY))
			FileSystem.createDirectory(DIRECTORY);
		var file = DIRECTORY + "/" + name;
		if (FileSystem.exists(file))
			FileSystem.deleteFile(file);
		return file;
	}

	static function persistence():Void {
		var file = prepareFile("settings.json");
		var store = new SettingsStore(sampleRegistry(), file);
		check(!FileSystem.exists(file), "nothing is written until something changes");

		store.set("interface/editor/fonts/main_font_size", PropertyValue.Int(16));
		store.set("interface/editor/appearance/custom_display_scale", PropertyValue.Float(1.25));
		store.set("interface/editor/appearance/expand_to_title", PropertyValue.Bool(false));
		store.set("interface/editor/docks/dock_tab_style", PropertyValue.Enum("text_only"));
		store.set("filesystem/directories/default_project_path", PropertyValue.Text("/home/me/projects"));
		check(store.lastError == null, 'saving works: ${store.lastError}');
		check(!FileSystem.exists(file + ".tmp"), "the temporary file is renamed into place");

		var reopened = new SettingsStore(sampleRegistry(), file);
		check(reopened.getInt("interface/editor/fonts/main_font_size") == 16, "Int survives a restart");
		check(reopened.getFloat("interface/editor/appearance/custom_display_scale") == 1.25, "Float survives a restart");
		check(!reopened.getBool("interface/editor/appearance/expand_to_title"), "Bool survives a restart");
		check(reopened.getString("interface/editor/docks/dock_tab_style") == "text_only", "Enum survives a restart");
		check(reopened.getString("filesystem/directories/default_project_path") == "/home/me/projects",
			"Text survives a restart");

		// Only changed values are saved, so a new default reaches users who never touched the setting.
		reopened.reset("interface/editor/fonts/main_font_size");
		var content = File.getContent(file);
		check(content.indexOf("main_font_size") < 0, "a value at its default is not saved");
		var newDefaults = new SettingsRegistry();
		newDefaults.define("interface/editor/fonts/main_font_size", PropertyType.Int, PropertyValue.Int(15));
		check(new SettingsStore(newDefaults, file).getInt("interface/editor/fonts/main_font_size") == 15,
			"a changed default takes effect for an untouched setting");

		// A value saved by a build that defines more settings is kept.
		File.saveContent(file, '{"version":1,"values":{"plugins/other/flag":true,"interface/editor/fonts/main_font_size":12}}');
		var partial = new SettingsRegistry();
		partial.define("interface/editor/fonts/main_font_size", PropertyType.Int, PropertyValue.Int(14));
		var older = new SettingsStore(partial, file);
		check(older.unknownPaths().join(",") == "interface/editor/fonts/main_font_size,plugins/other/flag",
			"saved values wait until their setting is defined");
		older.set("interface/editor/fonts/main_font_size", PropertyValue.Int(13));
		check(File.getContent(file).indexOf("plugins/other/flag") >= 0, "an undefined setting's saved value is written back");
		check(older.unknownPaths().join(",") == "plugins/other/flag", "a defined setting's value is adopted");
		partial.define("plugins/other/flag", PropertyType.Bool, PropertyValue.Bool(false));
		check(older.getBool("plugins/other/flag"), "a setting defined late picks up its saved value");

		// A batch saves once at the end.
		var batched = prepareFile("batched.json");
		var batch = new SettingsStore(sampleRegistry(), batched);
		batch.batch(function() {
			batch.set("interface/editor/fonts/main_font_size", PropertyValue.Int(20));
			batch.set("interface/editor/fonts/main_font_size", PropertyValue.Int(21));
			check(!FileSystem.exists(batched), "nothing is saved in the middle of a batch");
		});
		check(new SettingsStore(sampleRegistry(), batched).getInt("interface/editor/fonts/main_font_size") == 21,
			"the batch is saved when it ends");

		// Saving into a folder that does not exist yet creates it.
		var nested = DIRECTORY + "/nested/deeper/settings.json";
		var deep = new SettingsStore(sampleRegistry(), nested);
		deep.set("interface/editor/appearance/expand_to_title", PropertyValue.Bool(false));
		check(FileSystem.exists(nested), 'missing folders are created: ${deep.lastError}');
	}

	static function damagedFiles():Void {
		var file = prepareFile("damaged.json");
		File.saveContent(file, "{ this is not json");
		var store = new SettingsStore(sampleRegistry(), file);
		check(store.lastError != null, "a damaged file is reported");
		check(store.getInt("interface/editor/fonts/main_font_size") == 14, "a damaged file falls back to defaults");
		store.set("interface/editor/fonts/main_font_size", PropertyValue.Int(10));
		check(new SettingsStore(sampleRegistry(), file).getInt("interface/editor/fonts/main_font_size") == 10,
			"the next save replaces a damaged file");

		File.saveContent(file, '{"version":1,"values":{'
			+ '"interface/editor/fonts/main_font_size":"huge",'
			+ '"interface/editor/appearance/custom_display_scale":99,'
			+ '"interface/editor/docks/dock_tab_style":"sideways",'
			+ '"interface/editor/appearance/expand_to_title":1,'
			+ '"filesystem/directories/default_project_path":"/kept"}}');
		var mixed = new SettingsStore(sampleRegistry(), file);
		check(mixed.getInt("interface/editor/fonts/main_font_size") == 14, "a wrong-type value falls back to the default");
		check(mixed.getFloat("interface/editor/appearance/custom_display_scale") == 1.0, "an out-of-range value falls back");
		check(mixed.getString("interface/editor/docks/dock_tab_style") == "text_and_icon", "an unknown choice falls back");
		check(mixed.getBool("interface/editor/appearance/expand_to_title"), "a number is not a Bool");
		check(mixed.getString("filesystem/directories/default_project_path") == "/kept", "good values beside bad ones survive");

		File.saveContent(file, '[1, 2, 3]');
		var wrongShape = new SettingsStore(sampleRegistry(), file);
		check(wrongShape.lastError != null && wrongShape.isDefault("interface/editor/fonts/main_font_size"),
			"a file of the wrong shape is reported and ignored");
	}

}
