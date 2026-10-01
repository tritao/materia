package app.editor;

import app.Main.ReferenceEditorApp;
import app.editor.ExampleCatalog.ExampleEntry;
import Insets;
import LayoutAxis;
import LayoutAlignmentY;
import LayoutDirection;
import LayoutStyle;
import LayoutWrapMode;
import haxe.io.Path as StartPath;
import nativekit.ui.core.TextStyleOverride;
import nativekit.ui.core.View;
import nativekit.ui.icons.IconName;
import nativekit.ui.widgets.KeyedView;
import nativekit.ui.widgets.commands.CommandButton;
import nativekit.ui.widgets.controls.Button;
import nativekit.ui.widgets.controls.ButtonVariant;
import nativekit.ui.widgets.controls.Checkbox;
import nativekit.ui.widgets.controls.Spinner;
import nativekit.ui.widgets.layout.Column;
import nativekit.ui.widgets.layout.Row;
import nativekit.ui.widgets.scroll.ScrollView;
import nativekit.ui.widgets.text.Text;
import sys.FileSystem;

/** Start page dock panel: new document shortcuts, recent files, and bundled examples. */
@:access(app.Main.ReferenceEditorApp)
class StartPanel {
  static inline var CARD_WIDTH:Float = 250.0;

  public static function build(app:ReferenceEditorApp):View {
    var tokens = app.appearance.theme.tokens;
    var rows:Array<KeyedView> = [
      new KeyedView("title", new Text("Start", null, tokens.text, TextStyleOverride.text(22.0)))
    ];
    var loading = app.startLoading;
    if (loading != null)
      rows.push(new KeyedView("loading", loadingRow(app, loading)));
    else if (app.startFailure != null)
      rows.push(new KeyedView("failure", new Text(app.startFailure, null, tokens.danger,
        TextStyleOverride.text(13.0))));

    rows.push(new KeyedView("new-heading", app.sectionHeading("NEW FILE")));
    rows.push(new KeyedView("new", cardRow("start-new", [
      commandCard("empty", "start-empty-scene", "editor.new", "Empty scene", IconName.NewFile,
        ["Start with a blank scene"], app),
      commandCard("open", "start-open-file", "editor.open", "Open file...", IconName.FolderOpen,
        ["Open a saved scene or project"], app)
    ])));

    rows.push(new KeyedView("recent-heading", app.sectionHeading("RECENT FILES")));
    var recentCards:Array<KeyedView> = [];
    for (path in app.preferences.recent) {
      if (!FileSystem.exists(path)) continue;
      var target = path;
      recentCards.push(new KeyedView("recent:" + path, card(app, StartPath.withoutDirectory(path),
        IconName.NewFile, [shorten(StartPath.directory(path), 34)], null, function() {
          app.requestOpenPath(target);
        })));
    }
    rows.push(new KeyedView("recent", recentCards.length == 0
      ? new Text("Files you open or save appear here.", null, tokens.textSecondary, TextStyleOverride.text(12.0))
      : cardRow("start-recent", recentCards)));

    rows.push(new KeyedView("examples-heading", app.sectionHeading("EXAMPLES")));
    var exampleCards:Array<KeyedView> = [];
    for (entry in ExampleCatalog.available()) {
      var chosen = entry;
      exampleCards.push(new KeyedView("example:" + entry.id, card(app, entry.title,
        exampleIcon(entry), entry.description, entry.tag, function() app.requestExample(chosen))));
    }
    rows.push(new KeyedView("examples", exampleCards.length == 0
      ? new Text("No bundled examples were found next to this build.", null, tokens.textSecondary,
        TextStyleOverride.text(12.0))
      : cardRow("start-examples", exampleCards)));

    rows.push(new KeyedView("startup", new Checkbox("start-show-at-startup",
      "Show this page at startup", app.preferences.showStartPage, function(value) {
        app.preferences.setShowStartPage(value);
        app.invalidateView();
      })));

    var content = new Column("start-content", rows, contentStyle(app));
    var scrollStyle = ReferenceEditorApp.fillStyle();
    scrollStyle.background = tokens.surface;
    return new ScrollView("start-scroll", content, scrollStyle);
  }

  /** Spinner, current phase and elapsed time for a running build, with a Cancel button. */
  static function loadingRow(app:ReferenceEditorApp, entry:ExampleEntry):View {
    var tokens = app.appearance.theme.tokens;
    var job = app.startJob;
    var text = "Opening " + entry.title + "...";
    if (job != null)
      text += " " + job.control.currentPhase() + " · " + Std.int(job.elapsedSeconds()) + " s";
    var rowStyle = new LayoutStyle();
    rowStyle.width = LayoutAxis.grow();
    rowStyle.direction = LayoutDirection.LeftToRight;
    rowStyle.childAlignY = LayoutAlignmentY.Center;
    rowStyle.childGap = 10.0;
    var items:Array<KeyedView> = [
      new KeyedView("spinner", new Spinner("start-loading-spinner", "Opening " + entry.title)),
      new KeyedView("text", new Text(text, null, tokens.accent, TextStyleOverride.text(13.0)))
    ];
    if (job != null) {
      var cancel = new Button("Cancel", null, function() app.cancelExampleLoad(), "start-cancel-load");
      cancel.variant = ButtonVariant.Secondary;
      items.push(new KeyedView("cancel", cancel));
    }
    return new Row("start-loading", items, rowStyle);
  }

  static function contentStyle(app:ReferenceEditorApp):LayoutStyle {
    var style = new LayoutStyle();
    style.width = LayoutAxis.grow();
    style.height = LayoutAxis.fit();
    style.direction = LayoutDirection.TopToBottom;
    style.childGap = 12.0;
    style.padding = new Insets(24.0, 20.0, 24.0, 20.0);
    style.background = app.appearance.theme.tokens.surface;
    return style;
  }

  static function cardRow(key:String, cards:Array<KeyedView>):View {
    var style = new LayoutStyle();
    style.width = LayoutAxis.grow();
    style.direction = LayoutDirection.LeftToRight;
    style.wrapMode = LayoutWrapMode.Wrap;
    style.childGap = 10.0;
    style.rowGap = 10.0;
    return new Row(key, cards, style);
  }

  static function cardStyle(app:ReferenceEditorApp):LayoutStyle {
    var style = new LayoutStyle();
    style.width = LayoutAxis.fixed(CARD_WIDTH);
    style.direction = LayoutDirection.TopToBottom;
    style.childGap = 4.0;
    style.padding = new Insets(10.0, 10.0, 10.0, 10.0);
    style.background = app.appearance.theme.tokens.surfaceRaised;
    return style;
  }

  static function description(app:ReferenceEditorApp, key:String, lines:Array<String>, tag:Null<String>):Array<KeyedView> {
    var secondary = app.appearance.theme.tokens.textSecondary;
    var result:Array<KeyedView> = [for (index in 0...lines.length)
      new KeyedView("line:" + index, new Text(lines[index], null, secondary, TextStyleOverride.text(12.0)))];
    if (tag != null)
      result.push(new KeyedView("tag", new Text(tag, null, app.appearance.theme.tokens.accent,
        TextStyleOverride.text(11.0))));
    return result;
  }

  static function card(app:ReferenceEditorApp, title:String, icon:IconName, lines:Array<String>,
      tag:Null<String>, onClick:Void->Void):View {
    var button = new Button(shorten(title, 28), null, onClick, "card-button:" + title);
    button.leadingIcon = icon;
    button.variant = ButtonVariant.Secondary;
    button.enabled = app.startLoading == null && !app.documents.blocked();
    var rows = [new KeyedView("action", button)].concat(description(app, title, lines, tag));
    return new Column("card:" + title, rows, cardStyle(app));
  }

  static function commandCard(key:String, buttonKey:String, commandId:String, title:String, icon:IconName,
      lines:Array<String>, app:ReferenceEditorApp):KeyedView {
    var button = new CommandButton(buttonKey, commandId, app.commands);
    button.displayLabel = title;
    button.leadingIcon = icon;
    button.variant = ButtonVariant.Secondary;
    var rows = [new KeyedView("action", button)].concat(description(app, title, lines, null));
    return new KeyedView(key, new Column("card:" + key, rows, cardStyle(app)));
  }

  static function exampleIcon(entry:ExampleEntry):IconName return switch (entry.kind) {
    case Project(_): IconName.Cube;
    case Script(_) | WorkerRackToTable | WorkerGallery: IconName.Radar;
  };

  static function shorten(value:String, maximum:Int):String
    return value.length <= maximum ? value : "..." + value.substr(value.length - maximum + 3);
}
