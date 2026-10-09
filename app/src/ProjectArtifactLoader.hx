package app;

import app.CncProgramPlayer.CncJob;
import toolpathkit.tool.CutterProfile;
import toolpathkit.tool.Tool;
import haxe.crypto.Sha256;
import haxe.io.Bytes;
import materia.project.SceneArtifact;
import materia.project.SceneArtifact.SceneArtifactData;
import materia.project.SceneArtifact.SceneArtifactPart;
import materia.project.MaterialLibrary;
import materia.project.MeshMassProperties;
import materia.assembly.AssemblyFrames;
import materia.assembly.AssemblyRecord.AssemblyFrame;
import cadkit.modeling.AssemblyState;
import nativekit.scene.GeometryData;
import cadbridge.AssemblySimulationBridge.AssemblyPhysicalPart;

private typedef ArtifactIndexedMesh = {positions:Array<Float>, indices:Array<Int>};

/** Materializes immutable artifact bytes without knowing how they were produced or stored. */
class ProjectArtifactLoader {
  final cache:Null<PreparedSceneCaches.PreparedSceneCacheContext>;
  public function new(?cache:PreparedSceneCaches.PreparedSceneCacheContext) this.cache = cache;

  public function load(snapshot:Bytes, ?control:ProjectLoadControl, ?preparedBytes:Bytes,
      persistPrepared:Bool = false):GeneratedAssemblyScene {
    var loaded = loadInternal(snapshot, control, preparedBytes, persistPrepared, true);
    if (loaded == null) throw "Project preparation produced no scene";
    return loaded;
  }

  /** Fast cache path: never computes missing preparation on the caller's thread. */
  public function tryLoadCached(snapshot:Bytes, ?control:ProjectLoadControl):Null<GeneratedAssemblyScene> {
    return loadInternal(snapshot, control, null, false, false);
  }

  function loadInternal(snapshot:Bytes, control:Null<ProjectLoadControl>, preparedBytes:Null<Bytes>,
      persistPrepared:Bool, allowPreparation:Bool):Null<GeneratedAssemblyScene> {
    if (control != null) control.throwIfCancelled();
    if (control != null) control.phase("Decoding the artifact");
    var artifact = SceneArtifact.decodeView(snapshot);
    if (control != null) control.phase("Hashing the artifact");
    var artifactHash = PreparedProjectCache.hex(Sha256.make(snapshot));
    if (control != null) control.phase("Setting up the assembly");
    var records:Array<SceneObjectData> = [];
    var geometryBySnapshot:Map<String, GeometryData> = new Map();
    var scale = artifact.metresPerUnit;
    var poses:Map<String, AssemblyFrame> = new Map();
    var componentUseCount = new Map<String, Int>();
    var runtimeState:Null<AssemblyState> = null;
    if (artifact.assemblyDefinition != null) {
      var initialState = artifact.assemblyState == null ? null
        : materia.assembly.AssemblyDefinitionFlattener.flattenState(artifact.assemblyDefinition, artifact.assemblyState);
      var model = cadkit.modeling.CompiledAssembly.takeOwnership(artifact.assemblyDefinition);
      artifact.assemblyDefinition = model.definition;
      runtimeState = AssemblyState.fromModel(model, initialState);
      for (occurrence in model.definition.occurrences) {
        poses.set(occurrence.id, runtimeState.worldPose(occurrence.id));
        var count = componentUseCount.get(occurrence.definition);
        componentUseCount.set(occurrence.definition, count == null ? 1 : count + 1);
      }
    }

    var boundsByDefinition:Map<String, {minimum:Array<Float>, maximum:Array<Float>}> = new Map();
    var geometryKeyByDefinition:Map<String, String> = new Map();
    var localCentersByDefinition:Map<String, Array<Float>> = new Map();
    var faceDescriptorsByDefinition:Map<String, String> = new Map();
    var physicalParts:Array<AssemblyPhysicalPart> = [];
    var machiningTarget = artifact.machining == null ? null : artifact.machining.target;
    var components = [for (part in artifact.parts) if (part.id != machiningTarget) part];
    if (control != null) control.phase("Identifying prepared data");
    var preparedCache = new PreparedProjectCache(artifactHash, cache);
    if (control != null) control.phase("Reading prepared geometry and physics");
    var prepared = preparedBytes == null ? preparedCache.read(components) : PreparedSceneCodec.decode(preparedBytes, components);
    var cacheHit = prepared != null;
    if (cacheHit && preparedBytes == null) Sys.println("Materia project: reused prepared geometry and physics");
    if (control != null) control.throwIfCancelled();
    if (prepared == null && !allowPreparation) return null;
    if (prepared == null) {
      if (control != null) control.phase("Preparing geometry and physics");
      prepared = ProjectScenePreparation.prepare(components, scale, control);
    }
    if (control != null) {
      control.phase("Restoring prepared geometry");
    }
    for (index in 0...components.length) {
      if (control != null) control.throwIfCancelled();
      var component = components[index], part = prepared[index];
      var minimum = part.geometry.minimum, maximum = part.geometry.maximum;
      var geometry = CadPreviewGeometry.fromPrepared(component, part.geometry, scale);
      var geometryKey = "materia.artifact-part/1:" + artifactHash + ":" + component.id;
      geometryBySnapshot.set(geometryKey, geometry);
      boundsByDefinition.set(component.id, {minimum: minimum, maximum: maximum});
      var faces = component.faceDescriptors;
      if (faces != null) faceDescriptorsByDefinition.set(component.id, faces);
      geometryKeyByDefinition.set(component.id, geometryKey);
      localCentersByDefinition.set(component.id, [
        (minimum[0] + maximum[0]) * 0.5,
        (minimum[1] + maximum[1]) * 0.5,
        (minimum[2] + maximum[2]) * 0.5]);
      physicalParts.push(part.physical);
    }
    if (!cacheHit || persistPrepared) {
      if (control != null) { control.throwIfCancelled(); control.phase("Saving prepared geometry and physics"); }
      preparedCache.write(prepared);
    }

    if (artifact.assemblyDefinition != null) {
      var parts = new Map<String, SceneArtifactPart>();
      for (part in artifact.parts) parts.set(part.id, part);
      for (occurrence in artifact.assemblyDefinition.occurrences) {
        var component = parts.get(occurrence.definition);
        if (component == null) throw 'Missing geometry definition "${occurrence.definition}"';
        addOccurrenceRecord(records, component, occurrence.id, occurrence.definition,
          poses.get(occurrence.id), componentUseCount.get(occurrence.definition),
          boundsByDefinition, geometryKeyByDefinition, scale);
      }
    } else {
      for (component in artifact.parts)
        addOccurrenceRecord(records, component, component.id, component.id, null, 1,
          boundsByDefinition, geometryKeyByDefinition, scale);
    }

    var project = artifact.project;
    var motions:Array<RobotMotionTrack> = [];
    if (project != null) {
      for (record in records) if (project.dynamicParts.indexOf(record.id.substr(8)) >= 0) record.dynamicBody = true;
      var definition = artifact.assemblyDefinition;
      if (definition != null) motions = [for (motion in project.motions)
        new RobotMotionTrack("assembly:" + definition.id, 0, motion.loop, motion.keys, motion.joint)];
    }

    return {objects: records, projectJob: project == null ? null : project.job, robotMotions: motions,
      geometryBySnapshot: geometryBySnapshot,
      assemblyDefinition: artifact.assemblyDefinition,
      assemblyState: runtimeState == null ? null : runtimeState.record(),
      assemblyModel: runtimeState == null ? null : runtimeState.model,
      localCentersByDefinition: localCentersByDefinition, faceDescriptorsByDefinition: faceDescriptorsByDefinition,
      metresPerUnit: scale, physical: {metresPerUnit: scale, parts: physicalParts},
      recipeDocument: artifact.recipeDocument, recipeDiagnostics: artifact.recipeDiagnostics,
      cncJob: machiningJob(artifact, records, scale), mobileBase: artifact.mobileBase, mission: artifact.mission,
      robotTools: artifact.robotTools, robotSensors: artifact.robotSensors, machineMotion: artifact.machineMotion};
  }

  /**
   * The machining job the generator made with its machine, ready to run: meshes in metres in the
   * stock part's frame and the tool table as tools. Parts the tool may enter (the stock, and those
   * the job sacrifices) do not collide: the stock simulation cuts and reports what touches them,
   * where physical contact would only stop the tool going in.
   */
  static function machiningJob(artifact:SceneArtifactData, records:Array<SceneObjectData>, scale:Float):Null<CncJob> {
    var machining = artifact.machining;
    if (machining == null) return null;
    function partMesh(id:String):ArtifactIndexedMesh {
      var mesh = artifactMesh([for (part in artifact.parts) if (part.id == id) part][0]);
      return {positions: [for (value in mesh.positions) value * scale], indices: mesh.indices};
    }
    var stockMesh:Null<ArtifactIndexedMesh> = null;
    var stock = machining.stock;
    if (stock != null) {
      var definition = artifact.assemblyDefinition;
      var occurrence = definition == null ? [] : [for (item in definition.occurrences) if (item.id == stock) item];
      if (occurrence.length == 1) stockMesh = partMesh(occurrence[0].definition);
    }
    var entered = machining.sacrificial == null ? [] : machining.sacrificial.copy();
    if (stock != null) entered.push(stock);
    for (id in entered) for (record in records) if (record.id == "project:" + id) record.collisionEnabled = false;
    var target = machining.target;
    return {source: machining.program, axes: machining.axes, spindle: machining.spindle,
      workOffset: machining.workOffset, loop: machining.loop == true,
      tools: [for (tool in machining.tools) Tool.shaped(tool.number, tool.length, CutterProfile.decode(tool.profile))],
      stock: stock, stockMesh: stockMesh, target: target == null ? null : partMesh(target),
      toolPart: machining.toolPart, loadedTool: machining.loadedTool, controller: machining.controller,
      powerUpOffsets: machining.powerUpOffsets, powerUpSideOffsets: machining.powerUpSideOffsets};
  }

  /** An artifact part's triangles, with vertices at the same place welded into one. */
  static function artifactMesh(part:SceneArtifactPart):ArtifactIndexedMesh {
    var corners:Array<Float> = [];
    for (index in 0...part.indexCount) {
      var vertex = part.indices.getInt32(index * 4);
      for (axis in 0...3) corners.push(part.vertices.getDouble(vertex * 24 + axis * 8));
    }
    return welded(corners, part.id);
  }

  /** Triangle corners (xyz each) as an indexed mesh: corners at the same place become one vertex. */
  static function welded(corners:Array<Float>, source:String):ArtifactIndexedMesh {
    var positions:Array<Float> = [], indices:Array<Int> = [];
    var byCorner = new Map<String, Int>();
    for (corner in 0...Std.int(corners.length / 3)) {
      var x = corners[corner * 3], y = corners[corner * 3 + 1], z = corners[corner * 3 + 2];
      if (!Math.isFinite(x) || !Math.isFinite(y) || !Math.isFinite(z)) throw 'Mesh $source has a non-finite vertex';
      var key = '$x,$y,$z';
      var index = byCorner.get(key);
      if (index == null) {
        index = Std.int(positions.length / 3);
        byCorner.set(key, index);
        positions.push(x);
        positions.push(y);
        positions.push(z);
      }
      indices.push(index);
    }
    return {positions: positions, indices: indices};
  }

  static function addOccurrenceRecord(records:Array<SceneObjectData>, component:SceneArtifactPart,
      occurrenceId:String, definitionId:String, pose:Null<AssemblyFrame>, useCount:Null<Int>,
      boundsByDefinition:Map<String, {minimum:Array<Float>, maximum:Array<Float>}>,
      geometryKeyByDefinition:Map<String, String>, scale:Float):Void {
    var bounds = boundsByDefinition.get(definitionId);
    if (bounds == null) throw 'Missing bounds for geometry definition "$definitionId"';
    var minimum = bounds.minimum, maximum = bounds.maximum;
    var centerX = (minimum[0] + maximum[0]) * 0.5;
    var centerY = (minimum[1] + maximum[1]) * 0.5;
    var centerZ = (minimum[2] + maximum[2]) * 0.5;
    var center = pose == null ? {x: centerX, y: centerY, z: centerZ}
      : AssemblyFrames.transformPoint(pose, centerX, centerY, centerZ);
    var label = useCount != null && useCount > 1 ? component.name + " · " + occurrenceId : component.name;
    var volume = component.volume == null
      ? MeshMassProperties.compute(component.vertices, component.indices).volume : component.volume;
    var materialId = component.materialId == null ? "neutral" : component.materialId;
    var density = component.materialDensity == null
      ? MaterialLibrary.require(materialId).physical.density : component.materialDensity;
    var mass = volume * scale * scale * scale * density;
    if (!Math.isFinite(mass) || mass <= 0) throw 'Invalid mass for generated part "$occurrenceId"';
    records.push({id: "project:" + occurrenceId, label: label, type: "cad-preview",
      x: center.x * scale, y: center.y * scale, z: center.z * scale,
      width: Math.max(0.000001, (maximum[0] - minimum[0]) * scale),
      height: Math.max(0.000001, (maximum[1] - minimum[1]) * scale),
      depth: Math.max(0.000001, (maximum[2] - minimum[2]) * scale),
      collisionEnabled: true, dynamicBody: false, mass: mass,
      red: component.red, green: component.green, blue: component.blue,
      appearance: component.appearance, materialId: component.materialId, visible: true,
      meshSnapshot: geometryKeyByDefinition.get(definitionId),
      rotation: pose == null ? null : [pose.qx, pose.qy, pose.qz, pose.qw]});
  }


}
