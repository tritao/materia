package app.editor;

import app.EditorScene;
import app.SceneCodec;
import cadkit.parametric.features.ExtrudeFeature;
import cadkit.parametric.features.FilletFeature;
import materia.project.Appearance.Appearances;
import materia.project.MaterialLibrary;
import nativekit.ui.properties.PropertyDescriptor;
import nativekit.ui.properties.PropertyDescriptorOptions;
import nativekit.ui.properties.PropertyOption;
import nativekit.ui.properties.PropertyType;
import nativekit.ui.properties.PropertyValue;

/** Builds inspector descriptors against a retained scene and stable object identity. */
@:access(app.EditorScene)
class ScenePropertyProvider {
  public static function build(scene:EditorScene):Array<PropertyDescriptor> {
    var id = scene.selectedId;
    if (scene.object(id) == null) return [];
    // Capture identity in each binding: undo must still target this object after selection changes.
    var prefix = id + ":" + (scene.selectedFeatureKey == null ? "object" : scene.selectedFeatureKey) + ":";
    var result:Array<PropertyDescriptor> = [];
    if (!scene.kinematicOccurrences.exists(id))
      for (axis in 0...2) result.push(scene.positionProperty(id, axis, prefix));
    var visibility = new PropertyDescriptorOptions();
    visibility.category = "Rendering";
    result.push(new PropertyDescriptor(prefix + "visible", "Visible", PropertyType.Bool,
      function(_) return PropertyValue.Bool(scene.requiredObject(id).visible), function(_, value) {
        switch (value) {
          case PropertyValue.Bool(next): scene.setVisible(id, next);
          default: throw "Visibility requires a boolean";
        }
      }, visibility));
    var name = new PropertyDescriptorOptions();
    name.category = "Object";
    name.validator = function(_, value) return switch (value) {
      case PropertyValue.Text(text):
        StringTools.trim(text).length == 0 || SceneCodec.containsNul(text)
          ? "Name cannot be empty or contain NUL" : null;
      default: "Name requires text";
    };
    result.push(new PropertyDescriptor(prefix + "name", "Name", PropertyType.Text,
      function(_) {
        var item = scene.object(id);
        if (item == null) throw "Unknown scene object: " + id;
        return PropertyValue.Text(item.label);
      }, function(_, value) {
        switch (value) {
          case PropertyValue.Text(text): scene.setName(id, text);
          default: throw "Name requires text";
        }
      }, name));
    var importedShape = scene.requiredObject(id).kind == "cad-step";
    var genericPart = scene.requiredObject(id).kind == "cad-part";
    if (!importedShape && !genericPart) {
      result.push(scene.dimensionProperty(id, 0, prefix));
      result.push(scene.dimensionProperty(id, 1, prefix));
    }
    var colour = new PropertyDescriptorOptions();
    colour.category = "Rendering";
    colour.validator = function(_, value) return switch (value) {
      case PropertyValue.Text(text): EditorScene.decodeColour(text) == null ? "Colour requires #RRGGBB" : null;
      default: "Colour requires #RRGGBB";
    };
    // Property history stores the displayed text. Retain exact channel snapshots so
    // undoing a hex edit restores values loaded from a document without quantizing them.
    var colourSnapshots:Map<String, Array<Float>> = new Map();
    var initialColour = scene.requiredObject(id);
    colourSnapshots.set(EditorScene.encodeColour(initialColour.red, initialColour.green, initialColour.blue),
      [initialColour.red, initialColour.green, initialColour.blue]);
    result.push(new PropertyDescriptor(prefix + "colour", "Colour", PropertyType.Text,
      function(_) {
        var item = scene.requiredObject(id);
        return PropertyValue.Text(EditorScene.encodeColour(item.red, item.green, item.blue));
      }, function(_, value) {
        switch (value) {
          case PropertyValue.Text(text):
            var key = text.toUpperCase();
            var channels = colourSnapshots.get(key);
            if (channels == null) channels = EditorScene.decodeColour(key);
            if (channels == null) throw "Colour requires #RRGGBB";
            var current = scene.requiredObject(id);
            colourSnapshots.set(EditorScene.encodeColour(current.red, current.green, current.blue),
              [current.red, current.green, current.blue]);
            scene.setColour(id, channels[0], channels[1], channels[2]);
          default: throw "Colour requires #RRGGBB";
        }
      }, colour));
    var finishOptions = new PropertyDescriptorOptions();
    finishOptions.category = "Rendering";
    var sourceFinish = scene.componentFinishes.get(id);
    if (sourceFinish != null) finishOptions.options.push(new PropertyOption("component", "Component finish"));
    for (material in MaterialLibrary.all())
      finishOptions.options.push(new PropertyOption(material.id, material.name));
    finishOptions.options.push(new PropertyOption("custom", "Custom"));
    finishOptions.validator = function(_, value) return switch (value) {
      case PropertyValue.Enum(key):
        key == "component" && sourceFinish != null || key == "custom" ||
          MaterialLibrary.get(key) != null ? null : "Unknown finish";
      default: "Finish requires a preset";
    };
    result.push(new PropertyDescriptor(prefix + "finish", "Finish", PropertyType.Enum,
      function(_) {
        var item = scene.requiredObject(id);
        var appearance = item.appearance;
        if (sourceFinish != null && EditorScene.sameFinish(appearance, sourceFinish.appearance) &&
            item.red == sourceFinish.red && item.green == sourceFinish.green && item.blue == sourceFinish.blue)
          return PropertyValue.Enum("component");
        if (appearance == null) return PropertyValue.Enum("neutral");
        var preset = MaterialLibrary.get(appearance.finish);
        return PropertyValue.Enum(preset != null && Appearances.same(appearance,
          MaterialLibrary.appearance(preset.id)) ? preset.id : "custom");
      }, function(_, value) {
        switch (value) {
          case PropertyValue.Enum("component") if (sourceFinish != null): scene.resetComponentFinish(id);
          case PropertyValue.Enum("custom"): return;
          case PropertyValue.Enum(key):
            var preset = MaterialLibrary.get(key);
            if (preset == null) throw "Unknown finish";
            scene.setFinish(id, MaterialLibrary.appearance(key));
          default: throw "Finish requires a preset";
        }
      }, finishOptions));
    for (channel in ["metallic", "roughness"]) {
      var field = channel;
      var settings = new PropertyDescriptorOptions();
      settings.category = "Rendering";
      settings.minimum = 0.0;
      settings.maximum = 1.0;
      settings.step = 0.05;
      settings.validator = function(_, value) {
        var number:Null<Float> = switch (value) {
          case PropertyValue.Float(next): next;
          case PropertyValue.Int(next): next;
          default: null;
        };
        return number == null || !EditorScene.validColour(number) ? "Value must be between 0 and 1" : null;
      };
      result.push(new PropertyDescriptor(prefix + field, field == "metallic" ? "Metallic" : "Roughness",
        PropertyType.Float, function(_) {
          var current = scene.requiredObject(id).appearance;
          if (current == null) current = Appearances.neutral();
          return PropertyValue.Float(field == "metallic" ? current.metallic : current.roughness);
        }, function(_, value) {
          var number:Float = switch (value) {
            case PropertyValue.Float(next): next;
            case PropertyValue.Int(next): next;
            default: throw "Finish value requires a number";
          };
          var current = scene.requiredObject(id).appearance;
          if (current == null) current = Appearances.neutral();
          scene.setFinish(id, {finish: current.finish,
            metallic: field == "metallic" ? number : current.metallic,
            roughness: field == "roughness" ? number : current.roughness});
        }, settings));
    }
    if (!importedShape && !genericPart)
      result.push(scene.dimensionProperty(id, 2, prefix));
    var kindProvider = ObjectKindRegistry.find(scene.requiredObject(id).kind);
    if (kindProvider != null)
      for (property in kindProvider.properties(scene, id, prefix)) result.push(property);
    var selectedFeature = scene.selectedCadFeature(id);
    if (selectedFeature != null && selectedFeature.active && Std.isOfType(selectedFeature, ExtrudeFeature)) {
      var extrusion:ExtrudeFeature = cast selectedFeature;
      if (extrusion.amount != null)
        result.push(scene.extrusionDepthProperty(id, extrusion.id.toInt(), prefix));
    }
    if (selectedFeature != null && selectedFeature.active && Std.isOfType(selectedFeature, FilletFeature))
      result.push(scene.filletRadiusProperty(id, selectedFeature.id.toInt(), prefix));
    if (scene.activeSketchEdit != null && scene.activeSketchObjectId == id)
      scene.appendSketchDraftProperties(result, scene.activeSketchEdit, prefix);
    var assemblyProperties = scene.assemblyPropertyProvider;
    if (assemblyProperties != null)
      for (property in assemblyProperties(id)) result.push(property);
    result.push(scene.boolProperty(id,"collision","Collision",function(item)return item.collisionEnabled,
      "Physics",prefix));
    result.push(scene.boolProperty(id,"dynamic","Dynamic body",function(item)return item.dynamicBody,
      "Physics",prefix));
    result.push(scene.numberProperty(id,"mass","Mass",function(item)return item.mass,
      0.000001,1000000.0,"kg","Physics",prefix));
    return result;
  }

}
