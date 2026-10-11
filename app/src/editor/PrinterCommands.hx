package app.editor;

import app.Main.ReferenceEditorApp;
import app.PrinterMirror;
import machinekit.printing.KlipperConfig;
import haxeon.ui.core.Command;

/** Commands for a printer running Klipper behind Moonraker: connect to it, send it the project's print or a Klipper config, steer the print. */
@:access(app.Main.ReferenceEditorApp)
class PrinterCommands {
  public static function install(app:ReferenceEditorApp):Void {
    var link = app.printerLink;
    // The twin follows the machine: each state the machine reports moves the printer in the model there.
    link.onStatus = function(status) {
      if (!app.mirrorPrinter || app.session.printJob == null) return;
      try PrinterMirror.follow(app.session, status) catch (error:Dynamic) {
        app.mirrorPrinter = false;
        app.log("Printer mirror stopped: " + Std.string(error));
      }
      app.invalidateView();
    };
    function available():Bool return !app.documents.blocked();
    function hasJob():Bool return app.session.printJob != null;
    function connect():Bool {
      var endpoint = app.preferences.store.getString(AppSettings.PRINTER_ENDPOINT);
      if (StringTools.trim(endpoint).length == 0) {
        app.log("Set the printer's address under Editor Settings > Machine > Printer first");
        return false;
      }
      try link.connect(endpoint) catch (error:Dynamic) {
        app.log("Printer connection rejected: " + Std.string(error));
        return false;
      }
      app.log(link.note);
      return true;
    }
    function guarded(action:Void->Void):Void->Void
      return function() {
        try action() catch (error:Dynamic) app.log("Printer command rejected: " + Std.string(error));
        app.commands.refresh();
        app.invalidateView();
      };
    app.commands.register(new Command("printer.connect", "Printer: Connect", guarded(function() connect()), null,
      function() return available() && !link.connected()));
    app.commands.register(new Command("printer.disconnect", "Printer: Disconnect", guarded(function() {
      link.disconnect();
      app.log("Disconnected from the printer");
    }), null, function() return link.connected()));
    app.commands.register(new Command("printer.send", "Printer: Send to printer", guarded(function() {
      var job = app.session.printJob;
      if (job == null) throw "The project has no print job";
      if (!link.connected() && !connect()) return;
      link.send(app.printFileName(), job.source, true);
      app.log(link.note);
    }), null, function() return available() && hasJob()));
    app.commands.register(new Command("printer.install-config", "Printer: Install Klipper config", guarded(function() {
      var path = StringTools.trim(app.preferences.store.getString(AppSettings.PRINTER_CONFIG));
      if (path.length == 0) throw "Set the Klipper config file under Editor Settings > Machine > Printer first";
      var text = sys.io.File.getContent(path);
      var expected = KlipperConfig.parse(text);
      if (!link.connected() && !connect()) return;
      link.installConfig("printer.cfg", text, expected);
      app.log(link.note);
    }), null, function() return available()));
    app.commands.register(new Command("printer.pause", "Printer: Pause print", guarded(function() link.pause()), null,
      function() return link.connected()));
    app.commands.register(new Command("printer.resume", "Printer: Resume print", guarded(function() link.resume()), null,
      function() return link.connected()));
    app.commands.register(new Command("printer.cancel", "Printer: Cancel print", guarded(function() link.cancel()), null,
      function() return link.connected()));
    app.commands.register(new Command("printer.mirror", "Printer: Mirror the machine in the model", guarded(function() {
      app.mirrorPrinter = !app.mirrorPrinter;
      app.log(app.mirrorPrinter ? "The printer model follows the machine" : "The printer model no longer follows the machine");
    }), null, function() return available() && hasJob(), function() return app.mirrorPrinter));
  }
}
