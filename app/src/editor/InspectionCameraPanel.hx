package app.editor;

import Observable;

import app.Main.ReferenceEditorApp;
import app.EditorPerspectiveViewport;
import haxeon.ui.core.View;
import haxeon.ui.LayoutAxis;
import haxeon.ui.LayoutStyle;
import haxeon.ui.widgets.KeyedView;
import haxeon.ui.widgets.controls.Button;
import haxeon.ui.widgets.layout.Column;
import haxeon.ui.widgets.layout.Row;
import haxeon.ui.widgets.layout.SizedBox;
import haxeon.ui.widgets.text.Text;

/** One independent presentation cursor, hosted either in a dock or a picture-in-picture layer. */
@:access(app.Main.ReferenceEditorApp) class InspectionCameraPanel implements Observable {
  public var overlay(default, null) = false;
  var viewport:Null<EditorPerspectiveViewport>;
  var generation = -1;
  var zoom = 1.0;
  var simulationRevision = -1;
  var simulationActive = false;
  public function new() {
  }
  public function showDocked():Void setOverlay(false);
  function setOverlay(value:Bool):Void {
    if (overlay == value) return;
    overlay = value;
  }
  public function advancePresentation(seconds:Float):Bool return viewport != null
    && viewport.advancePresentation(seconds);
  public function preparePresentationFrame():Void {
    if (viewport != null) viewport.preparePresentationFrame();
  }
  public function dispose():Void {
    if (viewport != null) viewport.dispose();
    viewport = null;
    simulationRevision = -1;
  }
  /** Synchronize the live renderer before checking retained UI dependencies. Builds only describe controls. */
  public function update(app:ReferenceEditorApp):Void {
    if (generation != app.session.generation) {
      dispose();
      generation = app.session.generation;
      zoom = 1;
    }
    var presentation = app.session.presentation;
    if (presentation == null || app.hostContext == null) return;
    if (viewport == null) {
      viewport = new EditorPerspectiveViewport("inspection-camera-view", app.scene, app.hostContext);
      viewport.setPlacementOptions(false, 0, false);
      viewport.setSimulationOverlays(false);
      viewport.setCosmeticThreads(false);
      viewport.onInspectionZoom = next -> {zoom = next;
      app.invalidateView();
      };
    }
    viewport.setPresentation(
      presentation,
      app.session.projectAssemblyDefinition,
      app.session.presentationCenters(),
      app.session.assemblyUnitMetres(),
      true
    );
    viewport.solvePresentationJoints = app.session.presentationJointPoses;
    viewport.presentationGeometryFingerprint = app.session.presentationGeometryFingerprint;
    var frame = app.currentFramePresentation();
    var active = app.simulation.isActive(), revision = frame == null ? 0 : frame.revision;
    if (simulationRevision != revision || simulationActive != active) {
      viewport.setSimulationState(active, frame == null ? [] : frame.environment, revision);
      simulationRevision = revision;
      simulationActive = active;
    }
    viewport.setSampleCount(app.antialiasing);
    viewport.setLightingPreset(app.lightingPreset);
  }
  public function requestPaintPatch():Bool {
    return viewport == null || viewport.updateRetainedPaint();
  }
  public function build(app:ReferenceEditorApp, inOverlay:Bool = false):View {
    var style = new LayoutStyle();
    style.width = LayoutAxis.grow();
    style.height = LayoutAxis.grow();
    style.childGap = 4;
    var rows:Array<KeyedView> = [];
    if (overlay && !inOverlay) {
      var dock = new Button("Dock camera here", null, () -> {setOverlay(false);
      app.invalidateView();
      }, "inspection-dock-here");
      return new Column(
        "inspection-overlay-notice",
        [new KeyedView("notice", new Text("Camera is shown in the main viewport.")), new KeyedView("dock", dock)],
        style
      );
    }
    if (app.session.presentation == null) return new Text("Open an assembly to inspect its views.");
    if (app.hostContext == null) return new Text("Inspection rendering requires the desktop GPU host.");
    rows.push(new KeyedView(
      "views",
      PresentationControls.build(viewport, () -> app.invalidateView(), "inspection-views")
    ));
    var tools:Array<KeyedView> = [];
    function action(id:String, label:String, fn:Void -> Void):Void tools.push(new KeyedView(
      id,
      new Button(label, null, fn, "inspection-" + id)
    ));
    action("zoom-in", "Zoom +", () -> {zoom = Math.max(0.2, viewport.presentationZoom() / 1.4);
    viewport.setPresentationZoom(zoom);
    app.invalidateView();
    });
    action("zoom-out", "Zoom -", () -> {zoom = Math.min(8, viewport.presentationZoom() * 1.4);
    viewport.setPresentationZoom(zoom);
    app.invalidateView();
    });
    action("presentation", inOverlay ? "Dock" : "Picture in picture", () -> {setOverlay(!inOverlay);
    if (overlay) {
      app.workspace.close("inspection-camera");
      app.workspace.activate("perspective");
      app.invalidateView();
    } else app.openInspectionCamera();
    });
    rows.push(new KeyedView("tools", new Row("inspection-tools", tools)));
    rows.push(new KeyedView("camera",
      new SizedBox("inspection-image", viewport, LayoutAxis.grow(), LayoutAxis.grow())));
    rows.push(new KeyedView("caption", new Text("Orbit / pan / wheel zoom · independent view")));
    return new Column("inspection-camera", rows, style);
  }
}
