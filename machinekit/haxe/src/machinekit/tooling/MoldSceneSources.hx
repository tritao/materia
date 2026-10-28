package machinekit.tooling;

import materia.project.SceneArtifact.SceneArtifactData;
import materia.project.SceneSourceMap;
import materia.tooling.casting.CastingMoldPlan;
import materia.tooling.casting.MoldCores;
import materia.assembly.AssemblyFrames as F;

/** Scene ownership and provenance come from the same plan as physical tooling. */
class MoldSceneSources {
  public static function features(plan:CastingMoldPlan, casting:cadkit.modeling.Part):Array<SceneSourceFeature> {
    var shape = casting.shape, count = shape.subshapeCount(CadKit.ShapeKind.Face);
    var result:Array<SceneSourceFeature> = [{
      id: plan.document.input.id,
      label: "Casting",
      occurrence: CastingMoldView.REFERENCE_ID,
      faces: [],
      whole: true
    }];
    for (cavity in materia.casting.CastingCavities.derive(plan.document.input.envelope, plan.document.input.features)) {
      var local = machinekit.structural.CastingCavityCad.part(cavity), world:Null<cadkit.modeling.Part> = null;
      var indices:Array<Int> = [];
      try {
        world = machinekit.assembly.PosedParts.moved(local, plan.document.setup.sourceToMold);
        for (index in 0...count) {
          var face = shape.subshape(CadKit.ShapeKind.Face, index), common:Null<cadkit.Shape> = null;
          try {
            common = face.common(world.shape);
            var area = 0.0;
            for (i in 0...common.subshapeCount(CadKit.ShapeKind.Face)) {
              var patch = common.subshape(CadKit.ShapeKind.Face, i);
              try area += patch.faceArea() catch (error:Dynamic) {
                patch.close();
                throw error;
              }
              patch.close();
            }
            if (area > 1e-8) indices.push(index);
          } catch (error:Dynamic) {
            if (common != null) common.close();
            face.close();
            throw error;
          }
          common.close();
          face.close();
        }
      } catch (error:Dynamic) {
        if (world != null) world.close();
        local.close();
        throw error;
      }
      world.close();
      local.close();
      result.push({
        id: cavity.featureId,
        label: cavity.featureId,
        occurrence: CastingMoldView.REFERENCE_ID,
        faces: indices,
        whole: false
      });
    }
    return result;
  }

  public static function attach(data:SceneArtifactData, plan:CastingMoldPlan, features:Array<SceneSourceFeature>):Void {
    var definition = data.assemblyDefinition;
    if (definition == null) throw "Mold provenance requires its assembly";
    var groups = new Map<String, String>(), links = new Map<String, Array<String>>();
    function assign(id:String, path:String, sources:Array<String>):Void {
      if (groups.exists(id) && groups.get(id) != path) groups.set(id, "EPS cores/Shared supports");
      else groups.set(id, path);
      var ids = links.get(id);
      if (ids == null) {
        ids = [];
        links.set(id, ids);
      }
      for (source in sources) if (ids.indexOf(source) < 0) ids.push(source);
    }
    function insertSources(id:String):Array<String> {
      return [id].concat([for (feature in plan.document.input.features) if (feature.owner == id) feature.id]);
    }
    assign(CastingMoldView.REFERENCE_ID, "Casting", [plan.document.input.id]);
    for (panel in plan.panels.members) assign(panel.panel.id, "Mold shell", [plan.document.input.id]);
    for (former in plan.formers) assign(former.volume.id,
      "Formers/" + former.featureIds.join(" + "), former.featureIds);
    var supportPanels = [for (panel in plan.supportPanels) panel.panel.id => true];
    for (core in plan.cores) {
      var units = plan.coreUnits.filter(unit -> unit.coreIds.indexOf(core.id) >= 0);
      var sources:Array<String> = [], parents:Array<String> = [];
      for (unit in units) {
        if (parents.indexOf(unit.featureId) < 0) parents.push(unit.featureId);
        if (sources.indexOf(unit.featureId) < 0) sources.push(unit.featureId);
      }
      if (sources.length == 0) sources.push(core.featureId);
      if (parents.length == 0) parents.push(core.featureId);
      sources.sort(Reflect.compare);
      parents.sort(Reflect.compare);
      var path = "EPS cores/" + parents.join(" + ");
      for (id in MoldCores.assemblyMembers(core)) assign(id, path, sources);
      for (id in core.support.panelIds) if (supportPanels.exists(id)) assign(id, path, sources);
      for (id in core.support.bondedPanelIds) assign(id, path, sources);
      for (hardware in core.support.hardware) assign(hardware.id, path, sources);
    }
    for (insert in plan.document.input.inserts) {
      var id = plan.construction == SinglePocket ? insert.id : materia.tooling.ToolingIds.derive("insert-", insert.id);
      var occurrence = definition.occurrences.filter(item -> item.id == id);
      if (occurrence.length != 1) throw "Mold source insert has no unique occurrence: " + insert.id;
      features.push({id: insert.id, label: insert.label, occurrence: id, faces: [], whole: true});
      assign(id, "Inserts/" + insert.id, insertSources(insert.id));
    }
    for (fixture in plan.fixtures) {
      var path = "Inserts/" + fixture.insertId, source = insertSources(fixture.insertId);
      for (volume in MoldFixtures.volumes(fixture)) assign(volume.id, path, source);
      for (fixing in fixture.fixings) assign(fixing.hardwareId, path, source);
      for (mask in fixture.boreMasks) {
        assign(mask.hardwareId, path, source);
        if (mask.farEndId != null) assign(mask.farEndId, path, source);
      }
      switch fixture.support {
        case OnDatum(_, _, location):
          assign(location.primaryId, path, source);
          assign(location.secondaryId, path, source);
        case OnPanel(_):
      }
    }
    for (anchor in CastingMoldCad.anchorComponents(plan)) assign(
      anchor.id,
      "Inserts/" + anchor.insertId,
      insertSources(anchor.insertId)
    );
    for (seal in plan.seals) assign(seal.volume.id, "Inserts/" + seal.insertId, insertSources(seal.insertId));
    var members:Array<SceneSourceMember> = [];
    for (occurrence in definition.occurrences) {
      var source = links.get(occurrence.id);
      occurrence.includePath = groups.exists(occurrence.id) ? groups.get(occurrence.id) : "Hardware";
      if (source == null) source = [];
      source.sort(Reflect.compare);
      members.push({occurrence: occurrence.id, sources: source, shared: source.length == 0});
    }
    data.project = {dynamicParts: [], motions: [], sources: {features: features, members: members}};
  }
}
