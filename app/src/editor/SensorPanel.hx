package app.editor;

import app.ApplicationSimulation;
import app.Main.ReferenceEditorApp;
import app.ScriptOwnership;
import Insets;
import LayoutAxis;
import LayoutStyle;
import nativekit.ui.core.TextStyleOverride;
import nativekit.ui.core.View;
import nativekit.ui.icons.IconName;
import nativekit.ui.widgets.KeyedView;
import nativekit.ui.widgets.controls.Button;
import nativekit.ui.widgets.controls.ButtonVariant;
import nativekit.ui.widgets.layout.Column;
import nativekit.ui.widgets.layout.Row;
import nativekit.ui.widgets.properties.PropertyInspector;
import nativekit.ui.widgets.scroll.ScrollView;
import nativekit.ui.widgets.text.Text;

/** Sensor dock panel and its script-owned controls. */
@:access(app.Main.ReferenceEditorApp)
class SensorPanel {
  public static function build(app:ReferenceEditorApp):View {
    var appearance = app.appearance;
    var world = app.world;
    var simulation = app.simulation;
    var session = app.session;
    var scene = app.scene;
    var sensors = app.sensors;
    var commands = app.commands;
    var viewportWidth = app.viewportWidth;
    var fillStyle = function() return ReferenceEditorApp.fillStyle();
    var actionRowStyle = function() return ReferenceEditorApp.actionRowStyle();
    var actionColumnStyle = function() return ReferenceEditorApp.actionColumnStyle();
    var sectionHeading = function(label:String) return app.sectionHeading(label);
    var textLines = function(key:String, lines:Array<String>) return ReferenceEditorApp.textLines(key, lines);
    var log = function(message:String) app.log(message);
    var documentChanged = function() app.documentChanged();
    var refreshScriptMaterialization = function(message:String) app.refreshScriptMaterialization(message);
    var style=fillStyle();style.padding=new Insets(12.0,12.0,12.0,12.0);
    style.background=appearance.theme.tokens.surface;
    var robotRows:Array<KeyedView> = [];
    var attachedIds = world.robotIds();
    var worldIds = attachedIds.copy();
    for(id in sensors.configuredRobotIds())if(worldIds.indexOf(id)<0)worldIds.push(id);
    worldIds.sort(Reflect.compare);
    var simulatedIds = simulation.simulatedRobotIds();
    for (id in worldIds) {
      var robotButton = new Button(id,null,function(){sensors.selectRobot(id);app.invalidateView();},"sensor-robot:"+id);
      robotButton.selected = id == sensors.robotId;
      robotButton.enabled = simulatedIds.indexOf(id)>=0 || sensors.configuredRobotIds().indexOf(id)>=0;
      robotRows.push(new KeyedView("robot:"+id,robotButton));
    }
    if (robotRows.length == 0) robotRows.push(new KeyedView("robot-id",new Text("Robot: "+sensors.robotId)));
    var rows:Array<KeyedView> = [];
    for(index in 0...sensors.model.sensors.length) {
      var sensor=sensors.model.sensors[index];
      var button=new Button(sensor.name+" · "+sensor.kind,null,function(){sensors.select(index);app.invalidateView();},"sensor:"+sensor.id);
      button.selected=index==sensors.selectedIndex;rows.push(new KeyedView("sensor:"+sensor.id,button));
    }
    var ownership=session.scriptOwnership;
    var addLidar=new Button("LiDAR",null,function(){sensors.add("lidar");commands.refresh();},"sensor-add-lidar");
    var addImu=new Button("IMU",null,function(){sensors.add("imu");commands.refresh();},"sensor-add-imu");
    var addCamera=new Button("Camera",null,function(){sensors.add("camera");commands.refresh();},"sensor-add-camera");
    var removeSensor=new Button("Remove",null,function(){sensors.removeSelected();commands.refresh();},"sensor-remove");
    addLidar.leadingIcon=IconName.Plus;addImu.leadingIcon=IconName.Plus;addCamera.leadingIcon=IconName.Plus;
    removeSensor.leadingIcon=IconName.Trash;
    addLidar.enabled=ownership==null;addImu.enabled=ownership==null;addCamera.enabled=ownership==null;removeSensor.enabled=ownership==null;
    var actions=new Row("sensor-actions",[
      new KeyedView("add-lidar",addLidar),new KeyedView("add-imu",addImu),new KeyedView("add-camera",addCamera),
      new KeyedView("remove",removeSensor)
    ],actionRowStyle());
    var applySimulation = new Button(simulation.appliedRevision == 0 ? "Apply" : "Rebuild", null,
      function() {
        log(simulation.rebuild(sensors,scene,session) ? "Shared simulation configuration applied" :
          "Simulation rebuild rejected: " + simulation.error);
        commands.refresh();
      }, "sensor-apply");
    applySimulation.variant = ButtonVariant.Primary;
    var runtimeActions=new Column("sensor-runtime-actions",[
      new KeyedView("configuration",new Row("sensor-configuration-actions",[
      new KeyedView("undo",new Button("Undo",null,function(){
        commands.execute("editor.undo");},"sensor-undo")),
      new KeyedView("redo",new Button("Redo",null,function(){
        commands.execute("editor.redo");},"sensor-redo")),
      new KeyedView("apply",applySimulation)],actionRowStyle())),
      new KeyedView("playback",new Row("sensor-playback-actions",[
      new KeyedView("run",new Button("Run",null,function(){
        try {simulation.start();log("Simulation running");} catch(error:Dynamic){log("Run rejected: "+Std.string(error));}
        commands.refresh();
      },"sensor-run")),
      new KeyedView("pause",new Button("Pause",null,function(){
        simulation.stop();log("Simulation paused");commands.refresh();
      },"sensor-pause")),
      new KeyedView("reset",new Button("Reset",null,function(){
        log(simulation.reset() ? "Shared simulation reset" : "No simulation to reset");
        commands.refresh();
      },"sensor-reset")),
      new KeyedView("design",new Button("Design",null,function(){
        simulation.clear();log("Returned to design mode");commands.refresh();
      },"sensor-design"))
      ],actionRowStyle()))
    ],actionColumnStyle());
    var backendActions=new Row("sensor-backend-actions",[
      new KeyedView("deterministic",new Button("Test backend",null,function(){
        if(ownership==null)simulation.setBackend(ApplicationSimulation.DETERMINISTIC);else try {
          ownership.setOverride(ScriptOwnership.SIMULATION_TARGET,"backend","integer",ApplicationSimulation.DETERMINISTIC);
          refreshScriptMaterialization("Physics backend override changed");
        } catch(error:Dynamic)log("Override rejected: "+Std.string(error));commands.refresh();
      },"sensor-backend-deterministic")),
      new KeyedView("mujoco",new Button("MuJoCo",null,function(){
        if(ownership==null)simulation.setBackend(ApplicationSimulation.MUJOCO);else try {
          ownership.setOverride(ScriptOwnership.SIMULATION_TARGET,"backend","integer",ApplicationSimulation.MUJOCO);
          refreshScriptMaterialization("Physics backend override changed");
        } catch(error:Dynamic)log("Override rejected: "+Std.string(error));commands.refresh();
      },"sensor-backend-mujoco"))],actionRowStyle());
    var content:Array<KeyedView> = [new KeyedView("heading",sectionHeading("SENSORS")),
      new KeyedView("apply-state",new Text(simulation.pending(sensors,scene)
        ? "Pending changes · rebuild required"
        : "Configuration applied",null,appearance.theme.tokens.textSecondary,TextStyleOverride.text(12.0))),
      new KeyedView("simulation-mode",new Text("Mode: "+(simulation.isActive()
        ? (simulation.isRunning()?"Running":"Paused") : "Design"))),
      new KeyedView("ownership",ownership==null?new Text("Origin: document"):
        textLines("script-origin",["Origin: script",ownership.reference,
          'configuration v${ownership.configurationVersion}'])),
      new KeyedView("robots-heading",sectionHeading("ROBOTS")),
      new KeyedView("robots",new Column("sensor-robots",robotRows)),
      new KeyedView("backend-heading",sectionHeading("PHYSICS · "+simulation.userBackendName().toUpperCase())),
      new KeyedView("backend-actions",backendActions),
      new KeyedView("devices-heading",sectionHeading("DEVICES")),
      new KeyedView("actions",actions),
      new KeyedView("list",new Column("sensor-list",rows)),
      new KeyedView("runtime-heading",sectionHeading("SIMULATION")),
      new KeyedView("runtime-actions",runtimeActions)];
    if(ownership!=null){
      var overrideLabel=ownership.overridesEnabled?"Disable overrides":"Enable overrides";
      content.insert(4,new KeyedView("script-actions",new Column("script-actions",[
        new KeyedView("source",new Row("script-source-actions",[
        new KeyedView("reload",new Button("Reload script",null,function(){
          try {var result=session.reloadScript();simulation.setBackend(result.backend);
            simulation.setTimestep(result.timestep);documentChanged();log("Script reloaded; Apply/Rebuild restarts simulation");}
          catch(error:Dynamic)log("Script reload rejected; active simulation unchanged: "+Std.string(error));
          commands.refresh();},"script-reload")),
        new KeyedView("overrides",new Button(overrideLabel,null,function(){
          ownership.setOverridesEnabled(!ownership.overridesEnabled);
          refreshScriptMaterialization(ownership.overridesEnabled?"Overrides enabled":"Overrides disabled");
        },"script-overrides"))])),
        new KeyedView("revert",new Row("script-revert-actions",[
        new KeyedView("revert-simulation",new Button("Revert simulation",null,function(){
          if(ownership.revertTarget(ScriptOwnership.SIMULATION_TARGET))
            refreshScriptMaterialization("Simulation settings reverted to script values");
        },"script-revert-simulation")),
        new KeyedView("remove-stale",new Button("Remove stale",null,function(){
          if(ownership.removeStaleOverrides())refreshScriptMaterialization("Stale overrides removed");
          },"script-remove-stale"))
        ]))
      ])));
      content.insert(5,new KeyedView("simulation-origins",textLines("script-simulation-origins",
        ownership.propertyOrigins(ScriptOwnership.SIMULATION_TARGET,["backend","timestep"]))));
      content.insert(6,new KeyedView("robot-origins",textLines("script-robot-origins",
        ["Robot pose"].concat(ownership.propertyOrigins(sensors.robotId,["position","rotation"])))));
    }
    var selected=sensors.selected();
    if(!sensors.isEditable())content.push(new KeyedView("read-only",new Text("Remote robot configuration is read-only")));
    if(selected!=null){
      if(ownership!=null){
        var sensorTarget=sensors.robotId+"/"+selected.id;
        var sensorProperties=["updateRate","noiseStddev","noiseSeed","mount.frameId","mount.position","mount.rotation"];
        if(selected.kind=="lidar"){sensorProperties.push("rayCount");sensorProperties.push("maxRange");
          sensorProperties.push("startAngleRadians");sensorProperties.push("fieldOfViewRadians");}
        content.push(new KeyedView("sensor-origin",textLines("script-sensor-origins",
          ["Value origins"].concat(ownership.propertyOrigins(sensorTarget,sensorProperties)))));
        var decreaseRate=new Button("Rate -1 Hz",null,function(){
            try {ownership.setSensorRate(sensors.robotId,selected.id,Math.max(0,selected.updateRate-1));
              refreshScriptMaterialization("Sensor rate override changed");}
            catch(error:Dynamic)log("Override rejected: "+Std.string(error));
          },"script-rate-decrease");
        var increaseRate=new Button("Rate +1 Hz",null,function(){
            try {ownership.setSensorRate(sensors.robotId,selected.id,selected.updateRate+1);
              refreshScriptMaterialization("Sensor rate override changed");}
            catch(error:Dynamic)log("Override rejected: "+Std.string(error));
          },"script-rate-increase");
        var revertRate=new Button("Revert rate",null,function(){
            if(ownership.revertSensorRate(sensors.robotId,selected.id))
              refreshScriptMaterialization("Sensor rate reverted to script value");
          },"script-rate-revert");
        decreaseRate.enabled=ownership.overridesEnabled;
        increaseRate.enabled=ownership.overridesEnabled;
        revertRate.enabled=ownership.overridesEnabled;
        var rateActions=new Row("script-rate-actions",[
          new KeyedView("decrease",decreaseRate),new KeyedView("increase",increaseRate),
          new KeyedView("revert",revertRate)]);
        content.push(new KeyedView("rate-actions",rateActions));
        var revertSensor=new Button("Revert selected sensor",null,function(){
          if(ownership.revertTarget(sensorTarget))refreshScriptMaterialization("Sensor overrides reverted");
        },"script-sensor-revert-all");
        revertSensor.enabled=ownership.overridesEnabled;
        content.push(new KeyedView("sensor-revert",revertSensor));
      }
      var editable = ownership == null;
      var inspector = app.sensorInspector;
      if (inspector == null || app.sensorInspectorSensor != selected ||
          app.sensorInspectorModel != sensors.model) {
        inspector = new PropertyInspector("sensor-inspector:" + selected.id,
          sensors.properties(), null, null, null, null, "Sensor configuration");
        app.sensorInspector = inspector;
        app.sensorInspectorSensor = selected;
        app.sensorInspectorModel = sensors.model;
      }
      inspector.style.width = LayoutAxis.stretch();
      inspector.style.height = LayoutAxis.fit();
      inspector.scrollable = false;
      inspector.labelWidth = viewportWidth < 820.0 ? 76.0 : 100.0;
      inspector.enabled = editable;
      content.push(new KeyedView("properties", inspector));
    }
    var diagnostics=sensors.diagnostics();
    if(diagnostics.length>0)content.push(new KeyedView("diagnostics",new Text(
      diagnostics[0].code+": "+diagnostics[0].message)));
    if(ownership!=null&&ownership.diagnostics.length>0)
      content.push(new KeyedView("script-diagnostics",textLines("script-diagnostic-lines",ownership.diagnostics)));
    var contentStyle=new LayoutStyle();contentStyle.width=LayoutAxis.stretch();
    contentStyle.height=LayoutAxis.fit();contentStyle.padding=new Insets(8.0,8.0,8.0,8.0);
    return new ScrollView("sensor-scroll",new Column("sensor-panel",content,contentStyle),style);
  }

}
