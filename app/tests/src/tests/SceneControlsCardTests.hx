package tests;

import app.AppPreferences;
import app.editor.SceneControlsCard;
import app.editor.SceneControlsCard.SceneCardModel;
import sys.FileSystem;

/** The scene card's state: collapse, corner snapping, resize, persistence and Play. */
class SceneControlsCardTests {
  static final DIRECTORY = "build/scene-card-test";

  public static function main():Int {
    try {
      collapse();
      snapping();
      resize();
      persistence();
      playing();
      Sys.println("Scene controls card tests passed");
      return 0;
    } catch (error:Dynamic) {
      Sys.println('Scene controls card tests failed: $error');
      return 1;
    }
  }

  static function check(condition:Bool, message:String):Void if (!condition) throw message;

  static function collapse():Void {
    var card = new SceneCardModel();
    check(!card.collapsed(), "the card starts expanded");
    card.toggleCollapsed();
    check(card.collapsed(), "the chevron collapses it");
    card.toggleCollapsed();
    check(!card.collapsed(), "and expands it again");
    check(SceneControlsCard.summary("Assembled", "Machine") == "Assembled · Machine", "summary joins view and configuration");
    check(SceneControlsCard.summary("Assembled", null) == "Assembled", "summary without a configuration");
    check(SceneControlsCard.summary(null, null) == "Scene", "summary falls back");
  }

  static function snapping():Void {
    var none:Array<String> = [];
    var w = 1000.0, h = 600.0;
    check(SceneCardModel.nearestCorner(900, 100, w, h, none) == SceneCardModel.TOP_RIGHT, "top right");
    check(SceneCardModel.nearestCorner(900, 500, w, h, none) == SceneCardModel.BOTTOM_RIGHT, "bottom right");
    check(SceneCardModel.nearestCorner(100, 500, w, h, none) == SceneCardModel.BOTTOM_LEFT, "bottom left");
    check(SceneCardModel.nearestCorner(100, 100, w, h, none) == SceneCardModel.TOP_LEFT, "top left when free");
    // The view-angle control holds the top-left: that drop goes to the next nearest corner.
    var blocked = [SceneCardModel.TOP_LEFT];
    check(SceneCardModel.nearestCorner(100, 100, w, h, blocked) != SceneCardModel.TOP_LEFT, "top left is excluded");
    check(SceneCardModel.nearestCorner(600, 100, w, h, blocked) == SceneCardModel.TOP_RIGHT, "nearer of the remaining, top");
    check(SceneCardModel.nearestCorner(100, 400, w, h, blocked) == SceneCardModel.BOTTOM_LEFT, "nearer of the remaining, left");

    var card = new SceneCardModel();
    card.beginDrag(500, 300, w, h, 300, 200);
    check(card.dragging, "dragging follows the pointer");
    card.moveDrag(-50, 5000, w, h, 300, 200);
    check(card.dragX == 0 && card.dragY == h - 200, "the dragged card stays inside the viewport");
    check(card.snapCorner == SceneCardModel.BOTTOM_LEFT, "the snap target is shown while dragging");
    card.moveDrag(10, 10, w, h, 300, 200);
    check(card.snapCorner != SceneCardModel.TOP_LEFT, "the snap target never offers the blocked corner");
    card.moveDrag(800, 10, w, h, 300, 200);
    card.endDrag();
    check(!card.dragging && card.corner == SceneCardModel.TOP_RIGHT, "release snaps to the corner");
    card.beginDrag(0, 0, w, h, 300, 200);
    card.cancelDrag();
    check(card.corner == SceneCardModel.TOP_RIGHT, "a cancelled drag leaves the corner alone");
  }

  static function resize():Void {
    var card = new SceneCardModel();
    card.setCorner(SceneCardModel.BOTTOM_RIGHT);
    var a = SceneCardModel.restingPosition(card.corner, 1000, 600, 300, 200);
    var b = SceneCardModel.restingPosition(card.corner, 700, 500, 300, 200);
    check(card.corner == SceneCardModel.BOTTOM_RIGHT, "the corner survives a resize");
    check(a.x == 690 && a.y == 390, "bottom right sits a margin from both edges");
    check(b.x == 390 && b.y == 290, "and follows the new edges");
    var tiny = SceneCardModel.restingPosition(SceneCardModel.BOTTOM_RIGHT, 200, 100, 300, 200);
    check(tiny.x == SceneCardModel.MARGIN && tiny.y == SceneCardModel.MARGIN, "a viewport smaller than the card keeps it on screen");
    var tl = SceneCardModel.restingPosition(SceneCardModel.TOP_LEFT, 1000, 600, 300, 200);
    check(tl.x == SceneCardModel.MARGIN && tl.y == SceneCardModel.MARGIN, "top left");
  }

  static function persistence():Void {
    if (!FileSystem.exists("build")) FileSystem.createDirectory("build");
    if (!FileSystem.exists(DIRECTORY)) FileSystem.createDirectory(DIRECTORY);
    var file = DIRECTORY + "/settings.json";
    if (FileSystem.exists(file)) FileSystem.deleteFile(file);
    var preferences = new AppPreferences(file);
    var card = new SceneControlsCard();
    card.attach(preferences.store);
    check(card.model.corner == SceneCardModel.BOTTOM_LEFT && !card.model.collapsed(), "defaults without saved state");
    card.model.setCorner(SceneCardModel.TOP_RIGHT);
    card.model.toggleCollapsed();

    var reopened = new SceneControlsCard();
    reopened.attach(new AppPreferences(file).store);
    check(reopened.model.corner == SceneCardModel.TOP_RIGHT, "the corner is restored");
    check(reopened.model.collapsed(), "the collapsed state is restored");

    // Saved state that cannot be used keeps the defaults.
    var junk = new SceneCardModel();
    junk.load({corner: "middle", collapsed: "yes"});
    check(junk.corner == SceneCardModel.BOTTOM_LEFT && !junk.collapsed(), "unknown values are ignored");
    junk.load({corner: SceneCardModel.TOP_LEFT});
    check(junk.corner != SceneCardModel.TOP_LEFT, "a saved blocked corner is ignored");
    junk.load("text");
    junk.load(null);
  }

  static function playing():Void {
    var file = DIRECTORY + "/play.json";
    if (FileSystem.exists(file)) FileSystem.deleteFile(file);
    var preferences = new AppPreferences(file);
    var card = new SceneControlsCard();
    card.attach(preferences.store);
    var model = card.model;
    model.setPlaying(true);
    check(model.collapsed(), "Play collapses an expanded card");
    model.setPlaying(false);
    check(!model.collapsed(), "pause restores the expanded card");

    model.toggleCollapsed();
    check(model.collapsed(), "user collapses");
    model.setPlaying(true);
    model.setPlaying(false);
    check(model.collapsed(), "a card the user collapsed stays collapsed after Play");

    // Opening the card during Play is temporary and is not saved as the user's choice.
    model.toggleCollapsed();
    model.setPlaying(true);
    model.toggleCollapsed();
    check(!model.collapsed(), "the card can be opened during Play");
    var saved = new AppPreferences(file).store.getState(SceneCardModel.STATE_KEY);
    check(saved.collapsed == false, "Play does not overwrite the saved choice");
    model.setPlaying(false);
    check(!model.collapsed(), "the restored state is the user's choice");
  }
}
