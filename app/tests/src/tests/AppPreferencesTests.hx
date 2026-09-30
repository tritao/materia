package tests;

import app.AppPreferences;
import sys.FileSystem;
import sys.io.File;

/** The saved per-user settings: defaults, persistence and rejection of unusable values. */
class AppPreferencesTests {
  public static function main():Int {
    try { run(); Sys.println("App preferences tests passed"); return 0; }
    catch (error:Dynamic) { Sys.println('App preferences tests failed: $error'); return 1; }
  }

  static function check(condition:Bool, message:String):Void
    if (!condition) throw message;

  static function run():Void {
    var directory = "build/app-preferences-test";
    if (!FileSystem.exists(directory)) FileSystem.createDirectory(directory);
    var file = directory + "/preferences.json";
    if (FileSystem.exists(file)) FileSystem.deleteFile(file);

    var fresh = new AppPreferences(file);
    check(fresh.antialiasing == AppPreferences.DEFAULT_ANTIALIASING && fresh.antialiasing == 4,
      "anti-aliasing defaults to four samples");
    check(fresh.showStartPage, "the Start page shows by default");

    fresh.setAntialiasing(2);
    check(fresh.antialiasing == 2, "the choice takes effect");
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

    // A preferences file from before the setting existed, or a damaged value, falls back to the default.
    File.saveContent(file, '{"showStartPage":false,"recent":[]}');
    check(new AppPreferences(file).antialiasing == 4, "an older preferences file gets the default");
    File.saveContent(file, '{"antialiasing":"lots","recent":[]}');
    check(new AppPreferences(file).antialiasing == 4, "a non-numeric value is ignored");
    File.saveContent(file, '{"antialiasing":99,"recent":[]}');
    check(new AppPreferences(file).antialiasing == 4, "an out-of-range value is ignored");
    File.saveContent(file, "not json at all");
    check(new AppPreferences(file).antialiasing == 4, "a damaged file falls back to the defaults");

    FileSystem.deleteFile(file);
    FileSystem.deleteDirectory(directory);
  }
}
