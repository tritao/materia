package tests;

import app.AppPreferences;
import app.AppSettings;
import sys.FileSystem;
import sys.io.File;

/** The saved per-user settings: defaults, persistence, rejection of unusable values and the old file. */
class AppPreferencesTests {
  static final DIRECTORY = "build/app-preferences-test";

  public static function main():Int {
    try { run(); legacyImport(); Sys.println("App preferences tests passed"); return 0; }
    catch (error:Dynamic) { Sys.println('App preferences tests failed: $error'); return 1; }
  }

  static function check(condition:Bool, message:String):Void
    if (!condition) throw message;

  /** Deletes both files so each case starts from nothing. */
  static function fresh():{settings:String, legacy:String} {
    if (!FileSystem.exists("build")) FileSystem.createDirectory("build");
    if (!FileSystem.exists(DIRECTORY)) FileSystem.createDirectory(DIRECTORY);
    var files = {settings: DIRECTORY + "/settings.json", legacy: DIRECTORY + "/preferences.json"};
    for (file in [files.settings, files.legacy]) if (FileSystem.exists(file)) FileSystem.deleteFile(file);
    return files;
  }

  static function run():Void {
    var file = fresh().settings;

    var first = new AppPreferences(file);
    check(first.antialiasing == AppPreferences.DEFAULT_ANTIALIASING && first.antialiasing == 4,
      "anti-aliasing defaults to four samples");
    check(first.showStartPage, "the Start page shows by default");
    check(first.recent.length == 0, "no recent files at first");

    first.setAntialiasing(2);
    check(first.antialiasing == 2, "the choice takes effect");
    var reopened = new AppPreferences(file);
    check(reopened.antialiasing == 2, "the choice survives a restart");
    reopened.setAntialiasing(1);
    check(new AppPreferences(file).antialiasing == 1, "turning anti-aliasing off is remembered too");

    // Unusable values leave the saved choice alone.
    reopened.setAntialiasing(0);
    reopened.setAntialiasing(-3);
    reopened.setAntialiasing(17);
    check(reopened.antialiasing == 1 && new AppPreferences(file).antialiasing == 1,
      "zero, negative and absurd counts are refused");

    // Another setting saved afterwards keeps the anti-aliasing choice.
    reopened.setShowStartPage(false);
    var both = new AppPreferences(file);
    check(both.antialiasing == 1 && !both.showStartPage, "settings are saved together");
    check(!both.store.isDefault(AppSettings.SHOW_START_PAGE), "the dialog's store sees the same values");

    // Recent files: newest first, no duplicates, at most MAX_RECENT, kept across restarts.
    for (index in 0...AppPreferences.MAX_RECENT + 2) both.addRecent('/projects/scene$index.json');
    both.addRecent("/projects/scene5.json");
    var recent = new AppPreferences(file).recent;
    check(recent.length == AppPreferences.MAX_RECENT, "the recent list is capped");
    check(recent[0] == "/projects/scene5.json" && recent[1] == "/projects/scene11.json",
      "reopening a file moves it to the front");
    check(recent.indexOf("/projects/scene0.json") < 0, "the oldest entries fall off");
    both.forgetRecent("/projects/scene5.json");
    check(new AppPreferences(file).recent[0] == "/projects/scene11.json", "a forgotten file is removed");
    check(new AppPreferences(file).antialiasing == 1, "recent files do not disturb the settings");

    // A damaged or out-of-range file falls back to the defaults.
    File.saveContent(file, '{"version":1,"values":{"${AppSettings.ANTIALIASING}":99}}');
    check(new AppPreferences(file).antialiasing == 4, "an out-of-range value is ignored");
    File.saveContent(file, '{"version":1,"values":{"${AppSettings.ANTIALIASING}":"lots"}}');
    check(new AppPreferences(file).antialiasing == 4, "a non-numeric value is ignored");
    File.saveContent(file, "not json at all");
    var damaged = new AppPreferences(file);
    check(damaged.antialiasing == 4 && damaged.showStartPage && damaged.recent.length == 0,
      "a damaged file falls back to the defaults");
  }

  /** preferences.json from earlier builds is imported once, and left alone for them. */
  static function legacyImport():Void {
    var files = fresh();
    var old = '{"showStartPage":false,"antialiasing":2,"recent":["/old/a.json",7,"/old/b.json"]}';
    File.saveContent(files.legacy, old);
    var imported = new AppPreferences(files.settings, files.legacy);
    check(!imported.showStartPage && imported.antialiasing == 2, "the old values are imported");
    check(imported.recent.join(",") == "/old/a.json,/old/b.json", "the old recent files are imported, minus junk");
    check(FileSystem.exists(files.settings), "the import is saved");
    check(File.getContent(files.legacy) == old, "the old file is left for older builds");

    // Once settings.json exists, the old file is not read again.
    imported.setAntialiasing(8);
    File.saveContent(files.legacy, '{"antialiasing":16}');
    check(new AppPreferences(files.settings, files.legacy).antialiasing == 8, "the import happens only once");

    // An old file from before a setting existed, or with unusable values, imports what it can.
    files = fresh();
    File.saveContent(files.legacy, '{"showStartPage":false,"antialiasing":99,"recent":"nope"}');
    var partial = new AppPreferences(files.settings, files.legacy);
    check(!partial.showStartPage && partial.antialiasing == 4 && partial.recent.length == 0,
      "unusable old values fall back to the defaults");

    files = fresh();
    File.saveContent(files.legacy, "not json at all");
    var damaged = new AppPreferences(files.settings, files.legacy);
    check(damaged.showStartPage && damaged.antialiasing == 4, "a damaged old file is ignored");

    files = fresh();
    var beside = AppPreferences.besideWorkspace(DIRECTORY + "/workspace.json");
    beside.setShowStartPage(false);
    check(FileSystem.exists(files.settings), "settings are kept beside the workspace file");

    fresh();
    FileSystem.deleteDirectory(DIRECTORY);
  }
}
