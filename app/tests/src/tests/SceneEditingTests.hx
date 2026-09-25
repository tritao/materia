package tests;

import nativekit.ui.widgets.text.Text;


import app.EditorScene;
import app.EditorSceneTree;
import app.EditorSceneViewport;
import nativekit.ui.properties.PropertyBinding;
import nativekit.ui.properties.PropertyValue;
import nativekit.ui.properties.PropertyEditResult;
import nativekit.ui.core.ViewportCamera;
import nativekit.scene.SceneView;
import nativekit.scene.Transform;
import app.SceneDocumentSession;
import app.SceneCodec;
import app.SensorConfiguration;
import app.ApplicationSimulation;
import app.PerspectiveCamera;
import app.EditorPerspectiveViewport;
import app.PerspectiveSceneDrag;
import robotkit.world.RobotWorld;
import robotkit.world.McapRobotRecording;
import robotkit.world.McapRecordingReader;
import robotkit.world.ReplayRobot;
import robotkit.world.RobotRecording;
import robotkit.world.RobotCommand;
import robotkit.world.JointTarget;
import robotkit.model.Actuator;
import robotkit.model.Frame;
import robotkit.model.Joint;
import robotkit.model.JointLimits;
import robotkit.model.JointType;
import robotkit.model.Link;
import robotkit.model.RobotModel;
import robotkit.model.CollisionApproximation;
import robotkit.runtime.RobotRuntimeCompiler;
import robotkit.model.RobotDriveConfiguration;
import robotkit.model.RobotMobileConfiguration;
import robotkit.model.RobotForkConfiguration;
import sys.FileSystem;
import sys.io.File;

class SceneEditingTests {
  static function check(value:Bool, message:String):Void {
    if (!value) throw message;
  }
  static function near(actual:Float, expected:Float, message:String):Void
    check(Math.abs(actual - expected) < 0.00001, message);

  static function perspectiveCameraMath():Void {
    var camera = new PerspectiveCamera();
    var initialDistance = camera.distance;
    var initialRevision = camera.revision;
    camera.orbit(20, -10);
    check(camera.revision > initialRevision && camera.pitch > -1.49 && camera.pitch < 1.49,
      "perspective orbit changes bounded camera state");
    var targetX = camera.targetX, targetY = camera.targetY;
    camera.pan(30, -15, 600);
    check(camera.targetX != targetX || camera.targetY != targetY,
      "perspective pan moves the orbit target");
    camera.zoom(-120);
    check(camera.distance < initialDistance, "perspective wheel zoom moves closer");
    camera.frame(2.0, -3.0, 0.0, 4.0, 2.0, 0.1, 16.0 / 9.0);
    near(camera.targetX, 2.0, "perspective framing centers X");
    near(camera.targetY, -3.0, "perspective framing centers Y");
    check(camera.distance > 0.0, "perspective framing preserves a positive distance");
    var wideDistance = camera.distance;
    camera.frame(2.0, -3.0, 0.0, 4.0, 2.0, 0.1, 0.5);
    check(camera.distance > wideDistance, "perspective framing adapts to a narrow resize");
    var matrix = camera.viewProjection(16.0 / 9.0);
    for (index in 0...16) {
      var value = matrix.element(index);
      check(value == value && value - value == 0.0, "perspective matrix remains finite");
    }
    camera.reset();
    camera.fitClipRange([[-1.0, -1.0, -1.0, 1.0, 1.0, 1.0]]);
    check(camera.clipNear > camera.distance * 0.5 && camera.clipFar < camera.distance * 2.0,
      "viewport depth range follows nearby scene bounds");
    var eye = camera.eyePosition();
    camera.fitClipRange([[eye[0]-0.5, eye[1]-0.5, eye[2]-0.5,
      eye[0]+0.5, eye[1]+0.5, eye[2]+0.5]]);
    check(camera.clipNear < 0.01 && camera.clipFar > camera.clipNear,
      "camera inside visible bounds retains nearby geometry");
    camera.fitClipRange([]);
    check(camera.clipNear > 0.0 && camera.clipFar > camera.clipNear,
      "empty scene retains a valid depth range");
    var ray = camera.screenRay(800, 450, 1600, 900);
    var planeDistance = (camera.targetZ - ray.originZ) / ray.directionZ;
    near(ray.originX + ray.directionX * planeDistance, camera.targetX,
      "center camera ray reaches framed X");
    near(ray.originY + ray.directionY * planeDistance, camera.targetY,
      "center camera ray reaches framed Y");
    var scene = new EditorScene();
    camera.frame(-1.5, 0.0, 0.0, 1.6, 1.2, 0.05, 4.0 / 3.0);
    check(EditorPerspectiveViewport.pickScene(scene, camera, 800, 600, 400, 300) == "box",
      "perspective ray selects the framed object");
    scene.setVisible("box", false);
    check(EditorPerspectiveViewport.pickScene(scene, camera, 800, 600, 400, 300) == "scene",
      "perspective ray ignores hidden objects");
    scene.dispose();
    camera.reset();
    var centerRay = camera.screenRay(400, 300, 800, 600);
    var clipped = camera.projectSegment(
      centerRay.originX - centerRay.directionX,
      centerRay.originY - centerRay.directionY,
      centerRay.originZ - centerRay.directionZ,
      centerRay.originX + centerRay.directionX * 4.0,
      centerRay.originY + centerRay.directionY * 4.0,
      centerRay.originZ + centerRay.directionZ * 4.0, 800, 600);
    check(clipped != null, "perspective line clipping retains the visible segment");
    check(Math.abs(clipped[0].x - 400.0) < 1.0 &&
      Math.abs(clipped[0].y - 300.0) < 1.0 &&
      Math.abs(clipped[1].x - 400.0) < 1.0 &&
      Math.abs(clipped[1].y - 300.0) < 1.0,
      "clipped line stays within a pixel of the viewport center");
    check(camera.projectSegment(
      centerRay.originX - centerRay.directionX * 2.0,
      centerRay.originY - centerRay.directionY * 2.0,
      centerRay.originZ - centerRay.directionZ * 2.0,
      centerRay.originX - centerRay.directionX,
      centerRay.originY - centerRay.directionY,
      centerRay.originZ - centerRay.directionZ, 800, 600) == null,
      "perspective line clipping rejects segments behind the camera");
    var boxes:Array<app.SceneObjectData> = [];
    for (entry in [{id:"near", distance:4.0}, {id:"far", distance:6.0}]) boxes.push({
      id:entry.id, label:entry.id, type:"rectangle",
      x:centerRay.originX + centerRay.directionX * entry.distance,
      y:centerRay.originY + centerRay.directionY * entry.distance,
      z:centerRay.originZ + centerRay.directionZ * entry.distance,
      width:0.5, height:0.5, depth:0.5, red:0.5, green:0.5, blue:0.5,
      visible:true, collisionEnabled:true, dynamicBody:false, mass:1.0
    });
    var stacked = new EditorScene(boxes);
    check(EditorPerspectiveViewport.pickScene(stacked, camera, 800, 600, 400, 300) == "near",
      "perspective ray selects the nearest three-dimensional box");
    stacked.setVisible("near", false);
    check(EditorPerspectiveViewport.pickScene(stacked, camera, 800, 600, 400, 300) == "far",
      "perspective ray continues through a hidden front box");
    stacked.dispose();
    camera.reset();
    near(camera.targetX, 0.0, "perspective reset restores target X");
    near(camera.targetY, 0.0, "perspective reset restores target Y");
  }

  static function perspectiveDragging():Void {
    var scene = new EditorScene();
    var camera = new PerspectiveCamera();
    camera.frame(-1.5, 0.0, 0.0, 1.6, 1.2, 0.05, 4.0 / 3.0);
    var drag = PerspectiveSceneDrag.begin(scene, camera, "box", 400, 300, 800, 600, false, 0.2);
    check(drag != null, "perspective drag begins on its movement plane");
    check(drag.update(camera, 500, 300, 800, 600), "perspective drag previews movement");
    var movedX = scene.info("box").localTransform().element(12);
    check(movedX != -1.5 && drag.commit(), "perspective release commits one move");
    check(scene.document.history.undoCount == 1, "perspective drag creates one undo entry");
    scene.document.undo();
    near(scene.info("box").localTransform().element(12), -1.5,
      "undo restores the perspective drag origin");
    scene.document.redo();
    near(scene.info("box").localTransform().element(12), movedX,
      "redo restores the perspective move");
    var cancel = PerspectiveSceneDrag.begin(scene, camera, "box", 400, 300, 800, 600, true, 0.2);
    check(cancel != null && cancel.update(camera, 450, 350, 800, 600),
      "snapped perspective drag previews movement");
    check(cancel.cancel(), "perspective drag cancels");
    near(scene.info("box").localTransform().element(12), movedX,
      "cancel restores the perspective drag origin");
    check(scene.document.history.undoCount == 1, "cancel adds no undo entry");
    scene.dispose();
  }

  static function cadPlateMesh():Void {
    var scene = new EditorScene();
    check(scene.createMountingPlate(), "CAD mounting plate creation succeeds");
    var plate:app.EditorSceneObject = scene.object(scene.selectedId);
    check(plate != null && plate.kind == "cad-plate", "CAD plate retains its stable document kind");
    near(plate.width, 0.08, "CadKit millimetres convert to scene metres");
    check(scene.pick(0.0, 0.0) == "scene", "mesh picking passes through the CAD hole");
    check(scene.pick(0.03, 0.0) == plate.id, "mesh picking selects CAD material around the hole");
    var restored = new EditorScene(SceneCodec.decode(SceneCodec.encode(scene)));
    var restoredPlate:app.EditorSceneObject = restored.object(plate.id);
    check(restoredPlate != null && restoredPlate.kind == "cad-plate", "CAD object kind survives scene encoding");
    check(restored.pick(0.0, 0.0) == "scene" && restored.pick(0.03, 0.0) == plate.id,
      "reopened CAD geometry retains its through-hole mesh");
    restored.dispose(); scene.dispose();
  }

  static function simulationViewportSemantics():Void {
    var scene=new EditorScene(),viewport=new EditorSceneViewport(scene),camera=new ViewportCamera();
    scene.select("tower");var design=scene.info("tower").worldTransform(),dirty=scene.document.isDirty;
    var half=Math.PI/8,pose:{id:String,position:Array<Float>,rotation:Array<Float>}={
      id:"tower",position:[3.0,2.0,1.5],rotation:[0.0,0.0,Math.sin(half),Math.cos(half)]};
    viewport.setSimulationState(true,[pose],7);
    var screen=camera.worldToViewport(EditorSceneViewport.ORIGIN_X+300,EditorSceneViewport.ORIGIN_Y-200);
    check(viewport.pick(camera,screen.x,screen.y)=="tower",
      "XY picking follows a moved and rotated simulation body");
    check(!viewport.beginDrag(camera,screen.x,screen.y,false)&&!viewport.editingEnabled(),
      "paused simulation geometry cannot be dragged");
    viewport.frameSelected(camera);
    var framed=camera.worldToViewport(EditorSceneViewport.ORIGIN_X+300,EditorSceneViewport.ORIGIN_Y-200);
    near(framed.x,viewport.viewportWidth/2,"XY framing follows the simulation pose");
    near(framed.y,viewport.viewportHeight/2,"XY framing follows simulation Y");
    near(scene.info("tower").worldTransform().element(12),design.element(12),
      "simulation presentation preserves design X");
    check(scene.document.isDirty==dirty&&scene.document.history.undoCount==0,
      "simulation presentation creates no dirty state or undo entry");
    var reopened=new EditorScene(app.SceneCodec.decode(app.SceneCodec.encode(scene)));
    near(reopened.info("tower").worldTransform().element(12),design.element(12),
      "save and reopen preserve the design pose instead of the simulation pose");
    reopened.dispose();

    var perspective=new PerspectiveCamera();perspective.frame(3.0,2.0,1.5,1.8,1.8,0.2,4.0/3.0);
    var perspectiveView=scene.configureRenderView(new SceneView(),perspective.viewProjection(4.0/3.0),[pose]);
    var ray=perspective.screenRay(400,300,800,600);
    check(scene.pickRayWithView(perspectiveView,ray.originX,ray.originY,ray.originZ,
      ray.directionX,ray.directionY,ray.directionZ)=="tower",
      "perspective picking follows rotated simulation geometry and stable document IDs");
    scene.setVisible("tower",false);
    check(scene.pickRayWithView(perspectiveView,ray.originX,ray.originY,ray.originZ,
      ray.directionX,ray.directionY,ray.directionZ)=="scene",
      "simulation picking excludes hidden geometry");
    scene.dispose();
  }

  static function editingLifecycle():Void {
    var directory = "build/editing-lifecycle-" + Std.random(100000000);
    FileSystem.createDirectory(directory);
    var file = directory + "/scene.materia.json";
    File.saveContent(file, '{"format":"materia.scene","version":1,"objects":[]}');
    var session = new SceneDocumentSession();
    session.open(file);
    var empty = session.scene;
    var tree = new EditorSceneTree(empty);
    var viewport = new EditorSceneViewport(empty);
    var camera = new ViewportCamera();
    check(empty.selectedId == "scene" && tree.childCount("scene") == 0,
      "empty scene starts with synchronized root selection");
    check(viewport.pick(camera, EditorSceneViewport.ORIGIN_X, EditorSceneViewport.ORIGIN_Y) == "scene",
      "empty viewport has no stale geometry");

    var revision = empty.revision;
    check(empty.createRectangle(), "rectangle creation succeeds");
    var originalId = empty.selectedId;
    check(originalId == "rectangle-1", "created object receives a stable ID");
    check(tree.childCount("scene") == 1 && tree.childKeyAt("scene", 0) == originalId,
      "hierarchy observes created object");
    check(viewport.revision() > revision && viewport.pick(camera,
      EditorSceneViewport.ORIGIN_X, EditorSceneViewport.ORIGIN_Y) == originalId,
      "viewport observes created geometry");
    check(empty.properties().length == 11, "inspector exposes rendering and collision properties");

    var name = new PropertyBinding(empty.properties()[3], empty.context());
    check(name.apply(PropertyValue.Text("Hidden panel")) == PropertyEditResult.Applied,
      "rename succeeds through inspector binding");
    var renamed = empty.object(originalId);
    check(renamed != null && renamed.label == "Hidden panel", "hierarchy model observes rename");
    check(new PropertyBinding(empty.properties()[4], empty.context()).apply(PropertyValue.Float(2.4))
      == PropertyEditResult.Applied, "width edit succeeds");
    check(new PropertyBinding(empty.properties()[5], empty.context()).apply(PropertyValue.Float(0.8))
      == PropertyEditResult.Applied, "height edit succeeds");
    check(new PropertyBinding(empty.properties()[6], empty.context()).apply(PropertyValue.Text("#336699"))
      == PropertyEditResult.Applied, "colour edit succeeds");
    check(new PropertyBinding(empty.properties()[7], empty.context()).apply(PropertyValue.Float(0.6))
      == PropertyEditResult.Applied, "collision depth edit succeeds");
    check(new PropertyBinding(empty.properties()[9], empty.context()).apply(PropertyValue.Bool(true))
      == PropertyEditResult.Applied, "dynamic collision mode edit succeeds");
    check(new PropertyBinding(empty.properties()[10], empty.context()).apply(PropertyValue.Float(4.0))
      == PropertyEditResult.Applied, "collision mass edit succeeds");
    var visible = new PropertyBinding(empty.properties()[2], empty.context());
    check(visible.apply(PropertyValue.Bool(false)) == PropertyEditResult.Applied,
      "visibility edit succeeds through inspector binding");
    check(viewport.pick(camera, EditorSceneViewport.ORIGIN_X, EditorSceneViewport.ORIGIN_Y) == "scene",
      "hidden object disappears from viewport picking");

    check(empty.duplicateSelected(), "hidden object duplicates");
    var duplicateId = empty.selectedId;
    var duplicate = empty.object(duplicateId);
    check(duplicateId != originalId && duplicate != null && duplicate.label == "Hidden panel copy",
      "duplicate has a distinct stable ID and copied name");
    var duplicateState = empty.items()[1];
    check(!empty.info(duplicateId).visible() && tree.childKeyAt("scene", 0) == originalId
      && tree.childKeyAt("scene", 1) == duplicateId, "hidden state and hierarchy order duplicate together");
    check(duplicateState.width == 2.4 && duplicateState.height == 0.8 && duplicateState.depth == 0.6 &&
      duplicateState.dynamicBody && duplicateState.mass == 4.0 && duplicateState.collisionEnabled &&
      nearValue(duplicateState.red, 0.2), "rendering and collision settings duplicate together");
    check(empty.deleteSelected(), "duplicate deletes");
    check(empty.object(duplicateId) == null && empty.selectedId == originalId,
      "delete removes selection and selects its neighbour");
    empty.document.undo();
    check(empty.object(duplicateId) != null && empty.selectedId == duplicateId,
      "undo restores duplicate identity and selection");
    empty.document.redo();
    check(empty.object(duplicateId) == null && empty.selectedId == originalId,
      "redo deletes the same stable identity");

    try {
      session.save(file);
      check(!empty.document.isDirty && session.label().indexOf("*") < 0,
        "save marks the action result clean");
      empty.document.undo();
      check(empty.object(duplicateId) != null && empty.document.isDirty,
        "undo across savepoint restores object and dirty state");
      empty.document.redo();
      check(empty.object(duplicateId) == null && !empty.document.isDirty,
        "redo returns exactly to saved state");
      session.open(file);
      check(session.scene.items().length == 1 && session.scene.items()[0].id == originalId,
        "reopen preserves surviving stable identity");
      check(session.scene.items()[0].label == "Hidden panel" && !session.scene.info(originalId).visible(),
        "reopen preserves rename and hidden state");
      var reopened = session.scene.items()[0];
      check(reopened.width == 2.4 && reopened.height == 0.8 && reopened.depth == 0.6 &&
        reopened.dynamicBody && reopened.mass == 4.0 && reopened.collisionEnabled && nearValue(reopened.blue, 0.6),
        "reopen preserves dimensions, colour, and collision settings");
      check(!session.scene.document.isDirty && !session.scene.document.canUndo,
        "reopened document starts clean with fresh history");
    } catch (error:Dynamic) {
      session.dispose();
      if (FileSystem.exists(file)) FileSystem.deleteFile(file);
      if (FileSystem.exists(directory)) FileSystem.deleteDirectory(directory);
      throw error;
    }
    session.dispose();
    FileSystem.deleteFile(file);
    FileSystem.deleteDirectory(directory);
  }

  static inline function nearValue(actual:Float, expected:Float):Bool
    return Math.abs(actual - expected) < 0.00001;

  static function rectangleProperties():Void {
    var scene = new EditorScene();
    var viewport = new EditorSceneViewport(scene);
    var camera = new ViewportCamera();
    try {
      scene.select("box");
      var width = new PropertyBinding(scene.properties()[4], scene.context());
      var height = new PropertyBinding(scene.properties()[5], scene.context());
      var colour = new PropertyBinding(scene.properties()[6], scene.context());
      check(switch (width.apply(PropertyValue.Float(0.0))) {
        case PropertyEditResult.Rejected(_): true;
        default: false;
      }, "zero width is rejected");
      var notFinite = 0.0;
      notFinite = notFinite / notFinite;
      check(switch (height.apply(PropertyValue.Float(notFinite))) {
        case PropertyEditResult.Rejected(_): true;
        default: false;
      }, "non-finite height is rejected");
      check(switch (colour.apply(PropertyValue.Text("#12GG00"))) {
        case PropertyEditResult.Rejected(_): true;
        default: false;
      }, "invalid colour is rejected");
      check(width.apply(PropertyValue.Float(4.0)) == PropertyEditResult.Applied, "width updates");
      check(height.apply(PropertyValue.Float(0.5)) == PropertyEditResult.Applied, "height updates");
      check(colour.apply(PropertyValue.Text("#33CC66")) == PropertyEditResult.Applied, "colour updates");
      check(scene.pick(0.3, 0.0) == "box", "expanded geometry updates picking bounds");
      check(scene.pick(-1.5, 0.4) == "scene", "reduced geometry removes stale picking bounds");
      viewport.frameSelected(camera);
      near(camera.zoom, Math.min((viewport.viewportWidth - 96.0) / (4.0 * EditorSceneViewport.SCALE),
        (viewport.viewportHeight - 96.0) / (0.5 * EditorSceneViewport.SCALE)),
        "framing uses edited dimensions");
      var item = scene.items()[0];
      check(nearValue(item.red, 0.2) && nearValue(item.green, 0.8)
        && nearValue(item.blue, 0.4), "edited colour updates scene material state");
      scene.document.undo();
      item = scene.items()[0];
      check(nearValue(item.red, 0.22) && nearValue(item.green, 0.52)
        && nearValue(item.blue, 0.85), "colour undo restores exact source channels");
      scene.document.undo();
      item = scene.items()[0];
      check(item.height == 1.2, "height undo restores geometry");
      scene.document.undo();
      item = scene.items()[0];
      check(item.width == 1.6 && scene.pick(0.3, 0.0) == "scene",
        "width undo restores geometry and picking");
      scene.document.redo();
      scene.document.redo();
      scene.document.redo();
      item = scene.items()[0];
      check(item.width == 4.0 && item.height == 0.5 && nearValue(item.green, 0.8),
        "redo restores dimensions and colour");
      var nudgeHistory = scene.document.history.undoCount;
      check(scene.nudgeSelected(0.1, 0.0), "arrow-sized nudge applies");
      check(scene.nudgeSelected(0.0, 1.0), "Shift-sized nudge applies");
      near(scene.info("box").localTransform().element(12), -1.4, "small nudge uses 0.1 m step");
      near(scene.info("box").localTransform().element(13), 1.0, "large nudge uses 1 m step");
      check(scene.document.history.undoCount == nudgeHistory + 2, "each nudge is one undo operation");
      scene.document.undo();
      near(scene.info("box").localTransform().element(13), 0.0, "large nudge undo is independent");
      scene.document.undo();
      near(scene.info("box").localTransform().element(12), -1.5, "small nudge undo is independent");
    } catch (error:Dynamic) {
      scene.dispose();
      throw error;
    }
    scene.dispose();
  }

  static function viewportDragging():Void {
    var scene = new EditorScene();
    var viewport = new EditorSceneViewport(scene);
    var camera = new ViewportCamera(2.0, 40.0, -30.0);
    try {
      var grabWorld = camera.worldToViewport(EditorSceneViewport.ORIGIN_X - 1.2 * EditorSceneViewport.SCALE,
        EditorSceneViewport.ORIGIN_Y - 0.2 * EditorSceneViewport.SCALE);
      check(viewport.beginDrag(camera, grabWorld.x, grabWorld.y, false),
        "left drag begins on an object after camera pan and zoom");
      near(scene.info("box").localTransform().element(12), -1.5, "drag start preserves X grab offset");
      near(scene.info("box").localTransform().element(13), 0.0, "drag start preserves Y grab offset");

      var movedPointer = camera.worldToViewport(EditorSceneViewport.ORIGIN_X + 0.0 * EditorSceneViewport.SCALE,
        EditorSceneViewport.ORIGIN_Y - 0.7 * EditorSceneViewport.SCALE);
      check(viewport.updateDrag(camera, movedPointer.x, movedPointer.y), "drag preview moves object");
      near(scene.info("box").localTransform().element(12), -0.3, "preview X includes initial grab offset");
      near(scene.info("box").localTransform().element(13), 0.5, "preview Y includes initial grab offset");
      check(!scene.document.isDirty && scene.document.history.undoCount == 0,
        "drag preview does not add history entries");
      check(viewport.commitDrag(), "release commits moved preview");
      check(scene.document.isDirty && scene.document.history.undoCount == 1,
        "completed drag creates exactly one undo step");
      scene.document.undo();
      near(scene.info("box").localTransform().element(12), -1.5, "undo restores pre-drag X");
      near(scene.info("box").localTransform().element(13), 0.0, "undo restores pre-drag Y");
      scene.document.redo();
      near(scene.info("box").localTransform().element(12), -0.3, "redo restores dragged X");
      near(scene.info("box").localTransform().element(13), 0.5, "redo restores dragged Y");

      var center = camera.worldToViewport(EditorSceneViewport.ORIGIN_X - 0.3 * EditorSceneViewport.SCALE,
        EditorSceneViewport.ORIGIN_Y - 0.5 * EditorSceneViewport.SCALE);
      check(viewport.beginDrag(camera, center.x, center.y, false), "second drag begins");
      var cancelledPointer = camera.worldToViewport(EditorSceneViewport.ORIGIN_X + 1.3 * EditorSceneViewport.SCALE,
        EditorSceneViewport.ORIGIN_Y + 0.4 * EditorSceneViewport.SCALE);
      viewport.updateDrag(camera, cancelledPointer.x, cancelledPointer.y);
      check(viewport.cancelDrag(), "Escape cancels active drag");
      near(scene.info("box").localTransform().element(12), -0.3, "cancel restores original X");
      near(scene.info("box").localTransform().element(13), 0.5, "cancel restores original Y");
      check(scene.document.history.undoCount == 1, "cancel does not create an undo step");

      check(viewport.beginDrag(camera, center.x, center.y, false), "constrained drag begins");
      var horizontalPointer = camera.worldToViewport(EditorSceneViewport.ORIGIN_X + 0.4 * EditorSceneViewport.SCALE,
        EditorSceneViewport.ORIGIN_Y - 0.7 * EditorSceneViewport.SCALE);
      viewport.updateDrag(camera, horizontalPointer.x, horizontalPointer.y, true);
      near(scene.info("box").localTransform().element(12), 0.4, "horizontal constraint keeps dominant X");
      near(scene.info("box").localTransform().element(13), 0.5, "horizontal constraint locks Y");
      viewport.commitDrag();
      scene.document.undo();
      check(viewport.beginDrag(camera, center.x, center.y, false), "vertical constrained drag begins");
      var verticalPointer = camera.worldToViewport(EditorSceneViewport.ORIGIN_X - 0.1 * EditorSceneViewport.SCALE,
        EditorSceneViewport.ORIGIN_Y - 1.4 * EditorSceneViewport.SCALE);
      viewport.updateDrag(camera, verticalPointer.x, verticalPointer.y, true);
      near(scene.info("box").localTransform().element(12), -0.3, "vertical constraint locks X");
      near(scene.info("box").localTransform().element(13), 1.4, "vertical constraint keeps dominant Y");
      viewport.commitDrag();
      scene.document.undo();

      check(viewport.beginDrag(camera, center.x, center.y, true), "snapped drag begins");
      var snappedPointer = camera.worldToViewport(EditorSceneViewport.ORIGIN_X + 0.06 * EditorSceneViewport.SCALE,
        EditorSceneViewport.ORIGIN_Y - 0.94 * EditorSceneViewport.SCALE);
      viewport.updateDrag(camera, snappedPointer.x, snappedPointer.y);
      near(scene.info("box").localTransform().element(12), 0.0, "grid snapping rounds X");
      near(scene.info("box").localTransform().element(13), 1.0, "grid snapping rounds Y");
      viewport.commitDrag();
      check(scene.document.history.undoCount == 2, "snapped drag commits one undo step");
      var snappedCenter = camera.worldToViewport(EditorSceneViewport.ORIGIN_X,
        EditorSceneViewport.ORIGIN_Y - 1.0 * EditorSceneViewport.SCALE);
      check(viewport.setGridStep(0.5), "grid spacing is configurable");
      check(viewport.beginDrag(camera, snappedCenter.x, snappedCenter.y, true),
        "drag uses configured grid spacing");
      var customGridPointer = camera.worldToViewport(EditorSceneViewport.ORIGIN_X + 0.74 * EditorSceneViewport.SCALE,
        EditorSceneViewport.ORIGIN_Y - 1.26 * EditorSceneViewport.SCALE);
      viewport.updateDrag(camera, customGridPointer.x, customGridPointer.y);
      near(scene.info("box").localTransform().element(12), 0.5, "configured grid rounds X");
      near(scene.info("box").localTransform().element(13), 1.5, "configured grid rounds Y");
      viewport.cancelDrag();
    } catch (error:Dynamic) {
      scene.dispose();
      throw error;
    }
    scene.dispose();
  }

  static function sensorConfiguration():Void {
    var authored = new SensorConfiguration();
    var base = authored.model.links[0];
    var left = authored.model.addLink(new Link("Left wheel", "link/left"));
    var right = authored.model.addLink(new Link("Right wheel", "link/right"));
    var mast = authored.model.addLink(new Link("Mast", "link/mast"));
    var carriage = authored.model.addLink(new Link("Carriage", "link/carriage"));
    var forksLink = authored.model.addLink(new Link("Forks", "link/forks"));
    var configuredLidar:robotkit.model.Sensor = authored.selected();
    configuredLidar.startAngleRadians = -Math.PI * 0.5;
    configuredLidar.fieldOfViewRadians = Math.PI;
    function addJoint(id:String, name:String, type:JointType, child:Link,
        lower:Float, upper:Float):Void {
      var joint = new Joint(name, type, base, child, id);
      joint.limits = new JointLimits(lower, upper, 20.0, 100.0);
      authored.model.addJoint(joint);
    }
    addJoint("joint/left", "left wheel joint", JointType.Continuous, left, -100.0, 100.0);
    addJoint("joint/right", "right wheel joint", JointType.Continuous, right, -100.0, 100.0);
    addJoint("joint/lift", "mast lift", JointType.Prismatic, mast, 0.0, 2.0);
    addJoint("joint/tilt", "fork tilt", JointType.Revolute, carriage, -0.5, 0.5);
    addJoint("joint/spread", "fork spread", JointType.Prismatic, forksLink, 0.0, 0.8);
    authored.model.mobileBase = new RobotMobileConfiguration(
      RobotDriveConfiguration.Differential("joint/left", "joint/right", 0.1, 0.5),
      1.0, 1.5, 0.8, 1.0, 2.0, 1.0);
    authored.model.forkMechanism = new RobotForkConfiguration("joint/lift",
      1000.0, 600.0, 1.8, "joint/tilt", "joint/spread");
    var reopenedAuthored = new SensorConfiguration(
      haxe.Json.parse(haxe.Json.stringify(authored.records())));
    var reopenedMobile:Null<RobotMobileConfiguration> = reopenedAuthored.model.mobileBase;
    var reopenedForks:Null<RobotForkConfiguration> = reopenedAuthored.model.forkMechanism;
    check(reopenedMobile != null && reopenedForks != null &&
      reopenedAuthored.diagnostics().length == 0,
      "Materia robot records preserve and validate authored mechanism roles");
    var reopenedLidar:robotkit.model.Sensor = reopenedAuthored.selected();
    check(reopenedLidar.startAngleRadians == -Math.PI * 0.5 &&
      reopenedLidar.fieldOfViewRadians == Math.PI,
      "sensor records preserve LiDAR angular coverage");
    var mobileConfig:RobotMobileConfiguration = cast reopenedMobile;
    var forkConfig:RobotForkConfiguration = cast reopenedForks;
    check(switch mobileConfig.drive {
      case Differential(leftId, rightId, radius, track):
        leftId == "joint/left" && rightId == "joint/right" && radius == 0.1 && track == 0.5;
      case _: false;
    } && forkConfig.liftJointId == "joint/lift" &&
      forkConfig.tiltJointId == "joint/tilt" && forkConfig.spreadJointId == "joint/spread",
      "save and reopen retain drive geometry and named fork axes");
    authored.model.forkMechanism = null;
    authored.model.mobileBase = new RobotMobileConfiguration(
      RobotDriveConfiguration.Ackermann("joint/tilt", "joint/left", 1.2, 0.1, 0.5),
      1.0, 1.5);
    var reopenedAckermann = new SensorConfiguration(
      haxe.Json.parse(haxe.Json.stringify(authored.records())));
    var ackermannConfig:RobotMobileConfiguration = cast reopenedAckermann.model.mobileBase;
    check(switch ackermannConfig.drive {
      case Ackermann(steeringId, wheelId, wheelBase, radius, angle):
        steeringId == "joint/tilt" && wheelId == "joint/left" &&
          wheelBase == 1.2 && radius == 0.1 && angle == 0.5;
      case _: false;
    }, "save and reopen retain Ackermann steering roles and dimensions");
    reopenedAckermann.dispose();
    reopenedAuthored.dispose(); authored.dispose();

    var historySensors=new SensorConfiguration();
    historySensors.add("imu");
    historySensors.selectRobot("robot/history-b");
    var secondCount=historySensors.model.sensors.length;
    check(historySensors.document.undo()&&historySensors.robotId=="materia/robot"&&
      historySensors.model.sensors.length==2,
      "undo removes a newly added robot configuration from shared history");
    check(historySensors.document.redo()&&historySensors.robotId=="robot/history-b"&&
      historySensors.model.sensors.length==secondCount,
      "redo restores the new robot configuration and its selection");
    historySensors.selectRobot("materia/robot");
    check(historySensors.document.undo()&&historySensors.robotId=="materia/robot"&&
      historySensors.model.sensors.length==2,
      "undo after switching robots removes its creation while preserving the active robot");
    check(historySensors.document.undo()&&historySensors.model.sensors.length==1,
      "cross-robot undo removes the sensor from its owning model");
    check(historySensors.document.redo()&&historySensors.model.sensors.length==2,
      "cross-robot redo restores the sensor to its owning model");
    check(historySensors.document.redo()&&historySensors.robotId=="robot/history-b"&&
      historySensors.model.sensors.length==secondCount,
      "cross-robot redo restores the new robot configuration");
    historySensors.dispose();

    var sensors=new SensorConfiguration();
    check(sensors.model.sensors.length==1&&sensors.model.sensors[sensors.selectedIndex].kind=="lidar",
      "sensor panel starts with an editable LiDAR");
    var selectionRevision=sensors.document.revision;
    check(sensors.selectRobot("robot/selected")&&sensors.robotId=="robot/selected",
      "sensor configuration selects an explicit robot target");
    check(sensors.document.revision==selectionRevision+1,
      "creating a robot configuration creates one shared history entry");
    sensors.selectRobot("materia/robot");
    var properties=sensors.properties();
    var raysProperty=[for(property in properties) if(StringTools.endsWith(property.id,":rays")) property][0];
    var rays=new PropertyBinding(raysProperty,sensors.context());
    check(switch rays.apply(PropertyValue.Int(65)){case PropertyEditResult.Rejected(_):true;default:false;},
      "sensor UI rejects ray counts above runtime capacity");
    check(rays.apply(PropertyValue.Int(32))==PropertyEditResult.Applied,
      "sensor UI edits LiDAR resolution");
    check(sensors.model.sensors[sensors.selectedIndex].rayCount==32,"sensor model receives property edit");
    sensors.add("imu");
    check(sensors.model.sensors.length==2&&sensors.model.sensors[sensors.selectedIndex].kind=="imu",
      "sensor UI adds and selects an IMU");
    check(sensors.properties().length==14,"IMU hides LiDAR-only range and ray fields");
    check(sensors.removeSelected()&&sensors.model.sensors.length==1,
      "sensor UI removes the selected sensor");
    check(sensors.document.undo()&&sensors.model.sensors.length==2,
      "sensor add/remove operations participate in undo");
    check(sensors.diagnostics().length==0,"sensor UI produces a runtime-valid model");
    var scene=new EditorScene();
    var world=new RobotWorld();
    var simulation=new ApplicationSimulation(world);
    check(simulation.rebuild(sensors,scene),"valid sensor edits build a shared simulation: "+simulation.error);
    check(!simulation.pending(sensors,scene),"successful rebuild clears sensor and environment pending state");
    scene.select("tower");
    check(!simulation.pending(sensors,scene),"selection-only changes do not dirty simulation state");
    scene.setPosition("tower",0,2.0);
    check(simulation.pending(sensors,scene),"scene-only geometry edits require a rebuild");
    var appliedRevision=simulation.appliedRevision;
    sensors.model.sensors[0].rayCount=0;
    check(!simulation.rebuild(sensors,scene)&&simulation.appliedRevision==appliedRevision,
      "failed sensor rebuild preserves the running configuration");
    check(simulation.pending(sensors,scene),"failed replacement retains pending environment state");
    sensors.model.sensors[0].rayCount=8;
    simulation.start();
    check(simulation.rebuild(sensors,scene)&&simulation.isRunning(),
      "rebuilding a running simulation preserves realtime state");
    simulation.stop();
    var replacement=new EditorScene(scene.records());scene.dispose();scene=replacement;
    check(simulation.pending(sensors,scene),
      "replacing a document with equivalent geometry still requires a rebuild");
    check(simulation.rebuild(sensors,scene),"replacement scene rebuild succeeds");
    var observation=simulation.step().robot("materia/robot");
    check(observation!=null&&observation.sensors.length>0,
      "applied sensor configuration produces simulated measurements");
    simulation.dispose();world.close();scene.dispose();sensors.dispose();

    var lifecycleSession = new SceneDocumentSession(), lifecycleWorld = new RobotWorld();
    var lifecycleSimulation = new ApplicationSimulation(lifecycleWorld);
    var lifecycleRemote = new ReplayRobot("remote/lifecycle", new RobotRecording());
    lifecycleWorld.attach(lifecycleRemote);
    check(lifecycleSimulation.rebuild(lifecycleSession.sensors,lifecycleSession.scene),
      "lifecycle fixture builds a simulation");
    lifecycleSession.beforeReplace = lifecycleSimulation.clear;
    lifecycleSession.newDocument();
    check(lifecycleSimulation.simulatedRobotIds().length == 0 && !lifecycleSimulation.isRunning() &&
      lifecycleWorld.robot("materia/robot") == null && lifecycleWorld.robot("remote/lifecycle") == lifecycleRemote,
      "new document stops owned physics while preserving remote adapters");
    lifecycleSimulation.dispose(); lifecycleWorld.close(); lifecycleSession.dispose();
  }

  static function sensorWorkflow():Void {
    // Exercises the serialized multi-robot configuration boundary.
    var directory = "build/sensor-workflow-" + Std.random(100000000);
    FileSystem.createDirectory(directory);
    var documentPath = directory + "/robot.materia.json";
    var recordingPath = directory + "/robot.mcap";
    var session = new SceneDocumentSession();
    var rate = new PropertyBinding(session.sensors.properties()[2], session.sensors.context());
    var mountXProperty = [for (property in session.sensors.properties())
      if (StringTools.endsWith(property.id, ":position-0")) property][0];
    var mountX = new PropertyBinding(mountXProperty, session.sensors.context());
    check(rate.apply(PropertyValue.Float(20.0)) == PropertyEditResult.Applied,
      "sensor workflow edits acquisition rate");
    check(mountX.apply(PropertyValue.Float(0.25)) == PropertyEditResult.Applied,
      "sensor workflow edits mount position");
    check(session.sensors.selectRobot("materia/robot-b"),"sensor workflow adds a second robot target");
    new PropertyBinding(session.sensors.properties()[2],session.sensors.context()).apply(PropertyValue.Float(10.0));
    var arm=session.sensors.model.addLink(new Link("Arm","arm"));
    arm.mass=2.5;arm.centerOfMass=[0.1,0.2,0.3];arm.inertiaTensor=[2.0,0.0,0.0,0.0,3.0,0.0,0.0,0.0,4.0];
    arm.visualGeometry="geometry/arm-visual";arm.collisionGeometry="geometry/arm-collision";
    var joint=new Joint("Arm joint",JointType.Revolute,session.sensors.model.links[0],arm,"joint/arm");
    joint.limits.lower=-1.0;joint.limits.upper=1.0;joint.limits.velocity=2.0;joint.limits.effort=3.0;
    joint.parentFramePosition=[0.25,0.0,0.0];joint.childFramePosition=[0.0,0.1,0.0];joint.axis=[1.0,0.0,0.0];
    joint.drive=new Actuator("Arm drive",3.0,2.0);session.sensors.model.addJoint(joint);
    var armMount=session.sensors.model.addFrame(new Frame("Arm LiDAR mount",arm,"arm/lidar"));
    armMount.position=[0.4,0.0,0.0];session.sensors.model.sensors[0].frame=armMount;
    check(session.sensors.setRobotPose("materia/robot-b",[0.0,3.0,0.0],[0.0,0.0,0.0,1.0]),
      "sensor workflow stores an explicit robot pose");
    check(session.isDirty(), "sensor edits dirty the application document");
    session.save(documentPath);
    session.open(documentPath);
    check(session.sensors.configuredRobotIds().length==2,"two robot configurations survive reload");
    check(session.sensors.model.sensors[0].updateRate==10.0,
      "selected second robot retains its independent sensor rate");
    check(session.sensors.model.joints.length==1,"joint topology survives reload");
    var restoredJoint=session.sensors.model.joints[0];
    check(restoredJoint.id=="joint/arm"&&
      restoredJoint.parent.id=="base"&&restoredJoint.child.id=="arm"&&restoredJoint.limits.lower==-1.0&&
      restoredJoint.limits.upper==1.0&&restoredJoint.drive!=null&&restoredJoint.drive.maxEffort==3.0&&
      session.sensors.robotPosition("materia/robot-b")[1]==3.0&&
      session.sensors.robotPosition("materia/robot")[1]==0.0,
      "joint topology, actuator settings, and robot pose survive reload");
    check(session.sensors.model.links[1].mass==2.5&&
      session.sensors.model.links[1].centerOfMass[2]==0.3&&
      session.sensors.model.links[1].inertiaTensor[8]==4.0&&
      session.sensors.model.links[1].visualGeometry=="geometry/arm-visual"&&
      session.sensors.model.links[1].collisionGeometry=="geometry/arm-collision"&&
      restoredJoint.parentFramePosition[0]==0.25&&restoredJoint.childFramePosition[1]==0.1&&
      restoredJoint.axis[0]==1.0,
      "RobotModel v2 physical properties, geometry references, joint frames, and axis survive reload");
    var compiled=RobotRuntimeCompiler.compile(session.sensors.model);
    check(compiled.links.length==2&&compiled.links[1].mass==2.5&&
      compiled.joints[0].axis[0]==1.0&&compiled.identity.visualGeometry(1)=="geometry/arm-visual"&&
      compiled.identity.collisionGeometry(1)=="geometry/arm-collision",
      "runtime compiler lowers physical properties and retains geometry asset references");
    var emptyLegacyFrames:Array<Dynamic> = [];
    var emptyLegacySensors:Array<Dynamic> = [];
    var legacyLinks:Array<Dynamic> = [{id:"base",name:"Base"},{id:"arm",name:"Arm"}];
    var legacyJoints:Array<Dynamic> = [{id:"joint/arm",name:"Arm joint",type:"revolute",
      parentId:"base",childId:"arm",limits:{lower:-1.0,upper:1.0,velocity:2.0,effort:3.0},drive:null}];
    var legacyRecord:Dynamic={robotId:"legacy/robot",name:"Legacy robot",
      links:legacyLinks,joints:legacyJoints,
      frames:emptyLegacyFrames,sensors:emptyLegacySensors};
    var migrated=new SensorConfiguration(legacyRecord);
    check(migrated.model.schemaVersion==RobotModel.CURRENT_VERSION&&migrated.model.links[0].mass==1.0&&
      migrated.model.links[0].inertiaTensor[0]==1.0&&migrated.model.joints[0].axis[2]==1.0&&
      migrated.model.collisionApproximation==CollisionApproximation.BoundsBox,
      "version 1 robot records migrate to version 2 physical defaults");
    session.sensors.selectRobot("materia/robot");
    var restoredFrame = session.sensors.model.sensors[0].frame;
    check(restoredFrame != null && session.sensors.model.sensors[0].updateRate == 20.0 &&
      restoredFrame.position[0] == 0.25,
      "sensor settings survive document save and reload");
    session.scene.setVisible("tower",false);
    var hiddenObstacle=[for(item in session.scene.records())if(item.id=="tower")item][0];
    check(!hiddenObstacle.visible&&hiddenObstacle.collisionEnabled&&hiddenObstacle.depth==0.2,
      "render visibility is independent from persisted collision geometry");
    var world=new RobotWorld();
    var simulation=new ApplicationSimulation(world);
    var monitor=new ReplayRobot("monitor/remote",new RobotRecording());world.attach(monitor);
    check(simulation.rebuild(session.sensors,session.scene), "reloaded robots build one shared simulation");
    simulation.setBackend(ApplicationSimulation.MUJOCO);
    check(simulation.pending(session.sensors,session.scene),"backend selection requires an explicit rebuild");
    simulation.setBackend(ApplicationSimulation.DETERMINISTIC);
    check(world.robot("monitor/remote")==monitor,"shared rebuild leaves unrelated remote adapters attached");
    var appliedRevision=simulation.appliedRevision;
    session.sensors.selectRobot("remote/readonly");
    world.attach(new ReplayRobot("remote/readonly",new RobotRecording()));
    session.sensors.setReadOnlyRobots(["remote/readonly","monitor/remote"]);
    check(!session.sensors.isEditable()&&session.sensors.properties().length==0,
      "remote robot sensor targets are read-only");
    check(!simulation.rebuild(session.sensors,session.scene)&&simulation.appliedRevision==appliedRevision&&
      world.robot("materia/robot")!=null,"remote target rejection preserves active simulated robots");
    check(session.sensors.removeRobotConfiguration("remote/readonly"),
      "unsupported remote configuration can be removed without touching the live world");
    var remote=world.detach("remote/readonly");if(remote!=null)remote.close();
    session.sensors.selectRobot("materia/robot");
    session.sensors.setReadOnlyRobots(["remote/readonly","monitor/remote"]);
    session.scene.setPosition("tower",1,0.5);simulation.start();
    check(simulation.rebuild(session.sensors,session.scene)&&simulation.isRunning()&&
      world.robot("monitor/remote")==monitor,
      "running scene rebuild preserves unrelated remote adapters");
    simulation.stop();
    var observation = simulation.step();
    for (index in 0...8) observation = simulation.step();
    check(observation.robotIds().length==3,"shared world publishes two simulated robots and its unchanged remote adapter");
    var firstRobot=observation.robot("materia/robot");
    if(firstRobot==null)throw "Shared simulation lost the first robot";
    var sawObstacle=false;
    for(frame in firstRobot.sensors.toArray())if(frame.kind=="lidar")
      for(value in frame.values.toArray())if(value<session.sensors.model.sensors[0].maxRange)sawObstacle=true;
    check(sawObstacle,"LiDAR observes geometry populated from the Materia scene");
    var visual=simulation.visualState();
    check(visual.length==2&&visual[1].position[1]==3.0&&visual[0].sensors.length>0,
      "runtime visualization exposes robot poses and sensor mounts without editing the document");
    check(visual[1].links.length==2&&visual[1].sensors[0].linkId=="arm",
      "articulated visualization exposes the link owning each sensor mount");
    var presentation=simulation.capturePresentationSnapshot();
    var publishedRobot=presentation.world.robot("materia/robot");
    if(publishedRobot==null)throw "Application presentation lost its RobotWorld publication";
    check(presentation.revision>0&&presentation.robots.length==2&&presentation.environment.length>0&&
      publishedRobot.sensors.length==firstRobot.sensors.length,
      "one application presentation snapshot combines a physics revision with world publications");
    check(publishedRobot.sensors.get(0).sourceTimestampNs==firstRobot.sensors.get(0).sourceTimestampNs,
      "presentation keeps each sensor's actual source timestamp");
    var writer = new McapRobotRecording(recordingPath);
    for(robotId in observation.robotIds()) {
      var robot=observation.robot(robotId);
      if(robot==null)throw "Shared simulation snapshot lost a robot";
      for(frame in robot.sensors.toArray())writer.recordSensor(robotId,frame);
    }
    writer.close();
    var loaded = McapRecordingReader.load(recordingPath);
    var recordedRobots=new Map<String,Bool>();
    for(entry in loaded.entries)recordedRobots.set(entry.robotId,true);
    check(recordedRobots.exists("materia/robot")&&recordedRobots.exists("materia/robot-b"),
      "recording retains observations for both simulated robots");
    var replay = new ReplayRobot("materia/robot", loaded);
    var replayed = replay.sensors();
    check(replayed.length > 0 && replayed[0].sensorId == session.sensors.model.sensors[0].id,
      "record/replay preserves configured sensor identity");
    check(replayed[0].mountPosition.get(0) == 0.25 && replayed[0].values.length > 0,
      "record/replay preserves sensor mount and measurements");
    replay.close(); simulation.dispose(); world.close();

    session.scene.setPositionXY("tower",1.0,3.0);
    session.scene.select("tower");
    check(new PropertyBinding(session.scene.properties()[9],session.scene.context())
      .apply(PropertyValue.Bool(true))==PropertyEditResult.Applied,
      "physics regression makes the observed obstacle dynamic");

    var mujocoWorld=new RobotWorld();
    var mujocoSimulation=new ApplicationSimulation(mujocoWorld,ApplicationSimulation.MUJOCO);
    check(mujocoSimulation.rebuild(session.sensors,session.scene),
      "MuJoCo-enabled tests require the MuJoCo backend to execute");
    var mujocoObservation=mujocoSimulation.step();
    var initialObjects=mujocoSimulation.environmentVisualState();
    var initialVisual=mujocoSimulation.visualState()[1];
    var initialArm=[for(link in initialVisual.links)if(link.id=="arm")link][0];
    var initialObstacleZ=[for(object in initialObjects)if(object.id=="tower")object.position[2]][0];
    var mountedRobot=mujocoObservation.robot("materia/robot-b");
    if(mountedRobot==null)throw "MuJoCo world lost the articulated robot";
    var mountedLidar=mountedRobot.sensors.get(0),initialHit=false;
    for(value in mountedLidar.values.toArray())if(value<10.0)initialHit=true;
    check(mountedLidar.linkId=="arm"&&initialHit,
      "joint-mounted LiDAR observes the dynamic obstacle in MuJoCo");
    mujocoWorld.submit("materia/robot-b",RobotCommand.JointTargets([
      JointTarget.position(0,0.6)
    ],null));
    for(index in 0...8)mujocoObservation=mujocoSimulation.step();
    var mujocoRobot=mujocoObservation.robot("materia/robot");
    if(mujocoRobot==null)throw "MuJoCo world lost the configured robot";
    check(mujocoRobot.sensors.length>0&&mujocoSimulation.visualState().length==2,
      "MuJoCo backend publishes the same observable multi-robot contract");
    var movedVisual=mujocoSimulation.visualState()[1];
    var movedArm=[for(link in movedVisual.links)if(link.id=="arm")link][0];
    var movedObstacleZ=[for(object in mujocoSimulation.environmentVisualState())
      if(object.id=="tower")object.position[2]][0];
    var armMoved=false;
    for(index in 0...4)if(Math.abs(movedArm.rotation[index]-initialArm.rotation[index])>0.00001)armMoved=true;
    check(movedObstacleZ<initialObstacleZ&&armMoved,
      "runtime overlays follow the moving obstacle and articulated sensor link");
      check(mujocoSimulation.reset(),"MuJoCo simulation resets through the shared lifecycle");
      check(mujocoSimulation.visualState()[1].position[1]==3.0,
        "MuJoCo reset restores the persisted robot pose");
      check(Math.abs([for(object in mujocoSimulation.environmentVisualState())
        if(object.id=="tower")object.position[2]][0]-0.1)<0.00001,
        "MuJoCo reset restores the moving obstacle pose");
      var mujocoPath=recordingPath+".mujoco",mujocoWriter=new McapRobotRecording(mujocoPath);
      for(frame in mujocoRobot.sensors.toArray())mujocoWriter.recordSensor("materia/robot",frame);
      mujocoWriter.close();var mujocoReplay=new ReplayRobot("materia/robot",McapRecordingReader.load(mujocoPath));
      check(mujocoReplay.sensors().length>0,"MuJoCo observations survive MCAP replay");
      mujocoReplay.close();FileSystem.deleteFile(mujocoPath);
      var mujocoStatus=mujocoPath+".incomplete.status";
      if(FileSystem.exists(mujocoStatus))FileSystem.deleteFile(mujocoStatus);
    mujocoSimulation.dispose();mujocoWorld.close();session.dispose();
    FileSystem.deleteFile(recordingPath);
    var statusPath = recordingPath + ".incomplete.status";
    if (FileSystem.exists(statusPath)) FileSystem.deleteFile(statusPath);
  }

  static function renderingParity():Void {
    var scene = new EditorScene();
    try {
      var view = scene.configureRenderView(new SceneView(), Transform.identity());
      check(view.selectionOverrideCount() == 1,
        "perspective view highlights the shared scene selection");
      scene.select("scene");
      view = scene.configureRenderView(new SceneView(), Transform.identity());
      check(view.selectionOverrideCount() == 0,
        "perspective view clears highlighting with the shared selection");
      scene.select("tower");
      scene.setVisible("tower", false);
      check(!scene.info("tower").visible(),
        "perspective snapshot observes shared visibility");
      var tower = scene.items()[1];
      check(nearValue(tower.red, 0.92) && nearValue(tower.green, 0.48) && nearValue(tower.blue, 0.22),
        "perspective snapshot retains shared object colour");
    } catch (error:Dynamic) {
      scene.dispose();
      throw error;
    }
    scene.dispose();
  }

  static function main():Int {
    var scene = new EditorScene();
    try {
      check(scene.items().length == 2, "two initial objects");
      check(scene.pick(-1.5, 0) == "box", "pick first object");
      check(scene.pick(1.1, 0) == "tower", "pick second object");
      check(scene.pick(8, 8) == "scene", "empty space clears selection");
      var tree = new EditorSceneTree(scene);
      check(tree.childCount("scene") == 2, "hierarchy reflects scene");
      check(tree.childKeyAt("scene", 1) == "tower", "stable hierarchy identity");

      var camera = new ViewportCamera(1.7, 35, -20);
      var viewport = new EditorSceneViewport(scene);
      var screen = camera.worldToViewport(EditorSceneViewport.ORIGIN_X - 150,
        EditorSceneViewport.ORIGIN_Y);
      check(viewport.pick(camera, screen.x, screen.y) == "box", "picking after pan and zoom");
      var originalPan = camera.panX;
      var originalRevision = tree.revision();
      var x = new PropertyBinding(scene.properties()[0], scene.context());
      check(x.apply(PropertyValue.Float(-2.5)) == PropertyEditResult.Applied, "position edit accepted");
      near(scene.info("box").localTransform().element(12), -2.5, "selected object moved");
      near(scene.info("tower").localTransform().element(12), 1.1, "other object unchanged");
      near(camera.panX, originalPan, "editing does not move camera");
      check(scene.pick(-2.5, 0) == "box", "picking follows edited geometry");
      check(scene.pick(-1.5, 0) == "scene", "old location no longer hit");
      check(tree.revision() > originalRevision, "scene revision invalidates panels");
      check(scene.document.isDirty && scene.document.canUndo, "edit recorded in document");

      scene.select("tower");
      var towerX = new PropertyBinding(scene.properties()[0], scene.context());
      towerX.apply(PropertyValue.Float(2.0));
      check(scene.document.history.undoCount == 2, "different objects do not coalesce edits");
      scene.document.undo();
      near(scene.info("tower").localTransform().element(12), 1.1, "undo tower edit");
      scene.document.undo();
      near(scene.info("box").localTransform().element(12), -1.5, "undo targets original selection");
      check(!scene.document.isDirty, "undo restores initial document state");
      scene.document.redo();
      near(scene.info("box").localTransform().element(12), -2.5, "redo original object");

      var visibility = new PropertyBinding(scene.properties()[2], scene.context());
      visibility.apply(PropertyValue.Bool(false));
      check(!scene.info("tower").visible(), "hide selected object");
      check(scene.pick(1.1, 0) == "scene", "hidden object cannot be picked");
      check(!scene.document.canRedo, "new edit clears redo branch");
      scene.select("box");
      scene.document.undo();
      check(scene.info("tower").visible(), "undo visibility after selection changes");
      check(scene.pick(1.1, 0) == "tower", "restored object can be picked");

      viewport.frameSelected(camera);
      var center = camera.worldToViewport(EditorSceneViewport.ORIGIN_X - 250,
        EditorSceneViewport.ORIGIN_Y);
      near(center.x, viewport.viewportWidth / 2, "frame selection centers X");
      near(center.y, viewport.viewportHeight / 2, "frame selection centers Y");
      scene.select("scene");
      check(scene.properties().length == 0 && !scene.context().hasSelection, "empty selection has no editable properties");
      scene.dispose();
      scene.dispose();
      rectangleProperties();
      viewportDragging();
      editingLifecycle();
      renderingParity();
      perspectiveCameraMath();
      perspectiveDragging();
      cadPlateMesh();
      simulationViewportSemantics();
      sensorConfiguration();
      sensorWorkflow();
      ScriptedSetupTests.run();
      SceneDocumentTests.run();
      Sys.println("Scene editing tests passed");
      return 0;
    } catch (error:Dynamic) {
      scene.dispose();
      Sys.println("Scene editing tests failed: " + Std.string(error));
      return 1;
    }
  }
}
