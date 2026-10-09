package app.editor;

import app.editor.ExampleBrowser.ExampleFamily;
import haxeon.ui.widgets.controls.SearchField;

import app.Main.ReferenceEditorApp;
import app.editor.ExampleCatalog.ExampleEntry;
import haxeon.ui.Insets;
import haxeon.ui.LayoutAxis;
import haxeon.ui.LayoutAlignmentY;
import haxeon.ui.LayoutDirection;
import haxeon.ui.LayoutStyle;
import haxeon.ui.LayoutWrapMode;
import haxe.io.Path as StartPath;
import haxeon.ui.core.TextStyleOverride;
import haxeon.ui.core.View;
import haxeon.ui.icons.IconName;
import haxeon.ui.widgets.KeyedView;
import haxeon.ui.widgets.commands.CommandButton;
import haxeon.ui.widgets.controls.Button;
import haxeon.ui.widgets.controls.ButtonVariant;
import haxeon.ui.widgets.controls.Checkbox;
import haxeon.ui.widgets.controls.Spinner;
import haxeon.ui.widgets.layout.Column;
import haxeon.ui.widgets.layout.Row;
import haxeon.ui.widgets.scroll.ScrollView;
import haxeon.ui.widgets.text.Text;
import sys.FileSystem;

/** Start page dock panel: new document shortcuts, recent files, and bundled examples. */
@:access(app.Main.ReferenceEditorApp)
class StartPanel {
  static inline var CARD_WIDTH:Float = 250.0;

  public static function build(app:ReferenceEditorApp, width:Float = 1000.0):View {
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
      if (recentCards.length >= 3) break;
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
    var browser = app.startExamples;
    var available = ExampleCatalog.available();
    rows.push(new KeyedView("search", new SearchField("start-example-search", browser.query, function(value) {
      browser.search(value);
      app.invalidateView();
    }, null, "Search examples...")));
    var filters:Array<KeyedView> = [];
    for (category in ExampleBrowser.categories) {
      var chosenCategory = category;
      var button = new Button(category, null, function() {
        browser.selectCategory(chosenCategory);
        app.invalidateView();
      }, "start-category:" + category);
      button.variant = browser.category == category ? ButtonVariant.Primary : ButtonVariant.Secondary;
      button.selected = browser.category == category;
      filters.push(new KeyedView(category, button));
    }
    rows.push(new KeyedView("categories", cardRow("start-categories", filters)));
    var families = browser.filtered(available);
    var count = 0;
    for (family in families) count += family.examples.length;
    rows.push(new KeyedView("count", new Text(count + (count == 1 ? " example" : " examples"),
      null, tokens.textSecondary, TextStyleOverride.text(12.0))));
    var contentWidth = Math.max(1.0, width - 48.0);
    var cardWidth = Math.min(CARD_WIDTH, contentWidth);
    var columns = Std.int(Math.max(1.0, Math.floor((contentWidth + 10.0) / (CARD_WIDTH + 10.0))));
    var allFamilies = ExampleBrowser.families(available);
    for (category in ExampleBrowser.categories) {
      if (category == "All") continue;
      var categoryFamilies = [for (family in families) if (family.category == category) family];
      if (categoryFamilies.length == 0) continue;
      if (browser.category == "All") rows.push(new KeyedView("category-heading:" + category,
        new Text(category, null, tokens.text, TextStyleOverride.text(18.0))));
      var singles:Array<KeyedView> = [];
      var groupedFamilies:Array<ExampleFamily> = [];
      for (family in categoryFamilies) {
        var grouped = false;
        for (original in allFamilies) if (original.id == family.id) grouped = original.examples.length > 1;
        if (grouped) groupedFamilies.push(family);
        else {
          var entry = family.examples[0];
          singles.push(new KeyedView(entry.id, exampleCard(app, family, entry, cardWidth, false)));
        }
      }
      if (singles.length > 0) rows.push(new KeyedView("singles:" + category, cardGrid("singles:" + category, singles, columns)));
      for (family in groupedFamilies) {
        rows.push(new KeyedView("family-heading:" + family.id,
          new Text(family.title, null, tokens.textSecondary, TextStyleOverride.text(14.0))));
        var variants = [for (entry in family.examples) new KeyedView(entry.id, exampleCard(app, family, entry, cardWidth, true))];
        rows.push(new KeyedView("family:" + family.id, cardGrid("family:" + family.id, variants, columns)));
      }
    }
    if (count == 0) {
      rows.push(new KeyedView("empty", new Text(available.length == 0
        ? "No bundled examples were found next to this build."
        : "No examples match. Try another search or category.", null, tokens.textSecondary, TextStyleOverride.text(12.0))));
      if (available.length > 0) rows.push(new KeyedView("clear", new Button("Clear filters", null, function() {
        browser.search(""); browser.selectCategory("All"); app.invalidateView();
      }, "start-clear-filters")));
    }

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

  static function exampleBadge(entry:ExampleEntry):String {
    #if wasm
    if (entry.tag == "Editable project") return "Project example";
    #end
    return entry.tag;
  }

  /** One semantic button covers the preview, title and description. */
  static function exampleCard(app:ReferenceEditorApp, family:ExampleFamily, entry:ExampleEntry,
      width:Float, variant:Bool):View {
    var tokens = app.appearance.theme.tokens;
    var title = variant ? ExampleBrowser.variantTitle(entry) : entry.title;
    var detailsStyle = new LayoutStyle();
    detailsStyle.childGap = 4.0;
    detailsStyle.padding = new Insets(10.0, 10.0, 10.0, 10.0);
    detailsStyle.width = LayoutAxis.grow();
    var details = [new KeyedView("title", new Text(title, null, tokens.text, TextStyleOverride.text(14.0)))];
    details = details.concat(description(app, entry.id, entry.description, exampleBadge(entry)));
    var actionLabel = switch (entry.kind) {
      case Project(_): "Open project →";
      case Script(_) | WorkerRackToTable | WorkerGallery: "Open example →";
    };
    details.push(new KeyedView("open-label", new Text(actionLabel, null, tokens.accent, TextStyleOverride.text(12.0))));
    var labelStyle = new LayoutStyle();
    labelStyle.width = LayoutAxis.grow();
    labelStyle.height = LayoutAxis.fit();
    var style = new LayoutStyle();
    style.width = LayoutAxis.fixed(width);
    style.height = LayoutAxis.fit();
    style.padding = new Insets(0, 0, 0, 0);
    var button = new Button(title, style, function() app.requestExample(entry), "start-open-example:" + entry.id);
    button.variant = ButtonVariant.Secondary;
    button.accessibilityLabel = "Open " + entry.title;
    button.enabled = app.startLoading == null && !app.documents.blocked();
    button.labelView = new Column("example-content:" + entry.id, [
      new KeyedView("preview", StartExamplePreview.build(family, tokens.accent, Math.max(1.0, width - 24.0))),
      new KeyedView("details", new Column("example-details:" + entry.id, details, detailsStyle))
    ], labelStyle);
    return button;
  }

  /** Explicit rows use the dock pane width rather than the scroll content's intrinsic width. */
  static function cardGrid(key:String, cards:Array<KeyedView>, columns:Int):View {
    var rows:Array<KeyedView> = [];
    var index = 0;
    while (index < cards.length) {
      var rowStyle = new LayoutStyle();
      rowStyle.width = LayoutAxis.grow(); rowStyle.childGap = 10.0;
      rows.push(new KeyedView("row:" + index, new Row(key + ":row:" + index, cards.slice(index, index + columns), rowStyle)));
      index += columns;
    }
    var style = new LayoutStyle(); style.width = LayoutAxis.grow(); style.childGap = 10.0;
    return new Column(key, rows, style);
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
