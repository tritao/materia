package app.editor;

import app.Main.ReferenceEditorApp;
import haxeon.ui.core.Command;
import haxeon.ui.core.Shortcut;
import haxeon.ui.core.UiKey;
import haxeon.ui.core.UiModifier;

/** Transport commands for the shared simulation: play, pause, step, reset, and return to design. */
@:access(app.Main.ReferenceEditorApp)
class SimulationCommands {
  public static function install(app:ReferenceEditorApp):Void {
    var simulation = app.simulation;
    function available():Bool return !app.documents.blocked();
    // Applies the pending configuration, or builds the first one, before a transport action.
    function ensureBuilt(after:Void->Void):Void {
      if (simulation.isActive() && !simulation.pending(app.sensors, app.scene)) after();
      else app.prepareSimulation(after);
    }
    function play():Void {
      ensureBuilt(function() try {
        simulation.start();
        app.log("Simulation running");
        app.enterSimulationMode();
      } catch (error:Dynamic) {
        app.log("Run rejected: " + Std.string(error));
      });
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
      ensureBuilt(function() try {
        simulation.step();
        app.enterSimulationMode();
      } catch (error:Dynamic) {
        app.log("Step rejected: " + Std.string(error));
      });
    }), new Shortcut(UiKey.F10), function() return available() && !simulation.isRunning()));
    app.commands.register(new Command("sim.reset", "Simulation: Reset", transport(function() {
      app.log(simulation.reset() ? "Shared simulation reset" : "No simulation to reset");
    }), new Shortcut(UiKey.F5, UiModifier.Shift), function() return available() && simulation.isActive()));
    app.commands.register(new Command("sim.emergency-stop", "Simulation: Emergency Stop",
      transport(function() app.log(simulation.emergencyStop() ?
        "Emergency stop requested for every simulated robot" : "No simulated robots to stop")), null,
      function() return available() && simulation.isActive()));
    app.commands.register(new Command("sim.retry-mission", "Simulation: Reset Fault and Retry Mission",
      transport(function() app.log(simulation.retryMission() ? "Mission recovery requested" :
        "No recoverable mission fault")), null, function() {
          var mission = simulation.missionPlayer();
          return available() && mission != null && mission.failure != null;
        }));
    app.commands.register(new Command("sim.stop", "Simulation: Stop and return to Design",
      transport(function() {
        simulation.clear();
        app.log("Returned to design mode");
        app.leaveSimulationMode();
      }), null, function() return available() && simulation.isActive()));
    app.commands.register(new Command("sim.rebuild", "Simulation: Apply / Rebuild",
      transport(function() app.prepareSimulation()), null, available));
  }
}
