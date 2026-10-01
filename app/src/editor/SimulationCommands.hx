package app.editor;

import app.Main.ReferenceEditorApp;
import nativekit.ui.core.Command;
import nativekit.ui.core.Shortcut;
import nativekit.ui.core.UiKey;
import nativekit.ui.core.UiModifier;

/** Transport commands for the shared simulation: play, pause, step, reset, and return to design. */
@:access(app.Main.ReferenceEditorApp)
class SimulationCommands {
  public static function install(app:ReferenceEditorApp):Void {
    var simulation = app.simulation;
    // The browser build has no physics engine until nativekit_sim_mujoco gets a web build (app/web/README.md);
    // calling into its missing imports would stop the editor.
    function available():Bool return #if wasm false #else !app.documents.blocked() #end;
    function rebuild():Bool {
      if (simulation.rebuild(app.sensors, app.scene, app.session)) {
        app.log("Shared simulation configuration applied");
        return true;
      }
      app.log("Simulation rebuild rejected: " + simulation.error);
      return false;
    }
    // Applies the pending configuration, or builds the first one, before a transport action.
    function ensureBuilt():Bool {
      if (simulation.isActive() && !simulation.pending(app.sensors, app.scene)) return true;
      return rebuild();
    }
    function play():Void {
      if (!ensureBuilt()) return;
      try {
        simulation.start();
        app.log("Simulation running");
        app.enterSimulationMode();
      } catch (error:Dynamic) {
        app.log("Run rejected: " + Std.string(error));
      }
    }
    function pause():Void {
      simulation.stop();
      app.log("Simulation paused");
    }
    function transport(action:Void->Void):Void->Void
      return function() {
        action();
        app.commands.refresh();
        app.invalidateView();
      };
    app.commands.register(new Command("sim.play", "Simulation: Play", transport(play), null,
      function() return available() && !simulation.isRunning()));
    app.commands.register(new Command("sim.pause", "Simulation: Pause", transport(pause), null,
      function() return available() && simulation.isRunning()));
    app.commands.register(new Command("sim.toggle", "Simulation: Play / Pause",
      transport(function() if (simulation.isRunning()) pause() else play()),
      new Shortcut(UiKey.F5), available, function() return simulation.isRunning()));
    app.commands.register(new Command("sim.step", "Simulation: Step", transport(function() {
      if (!ensureBuilt()) return;
      try {
        simulation.step();
        app.enterSimulationMode();
      } catch (error:Dynamic) {
        app.log("Step rejected: " + Std.string(error));
      }
    }), new Shortcut(UiKey.F10), function() return available() && !simulation.isRunning()));
    app.commands.register(new Command("sim.reset", "Simulation: Reset", transport(function() {
      app.log(simulation.reset() ? "Shared simulation reset" : "No simulation to reset");
    }), new Shortcut(UiKey.F5, UiModifier.Shift), function() return available() && simulation.isActive()));
    app.commands.register(new Command("sim.stop", "Simulation: Stop and return to Design",
      transport(function() {
        simulation.clear();
        app.log("Returned to design mode");
        app.leaveSimulationMode();
      }), null, function() return available() && simulation.isActive()));
    app.commands.register(new Command("sim.rebuild", "Simulation: Apply / Rebuild",
      transport(function() rebuild()), null, available));
  }
}
