package app.editor;

import app.editor.ExampleCatalog.ExampleEntry;

typedef ExampleFamily = {
  final id:String;
  final title:String;
  final category:String;
  final examples:Array<ExampleEntry>;
}

/** Start-page navigation state and searchable groups; opening still uses the original catalogue entries. */
class ExampleBrowser {
  public static final categories = ["All", "Robotics & handling", "Welding", "Machining", "CAD & design", "People & simulation"];
  public var query:String = "";
  public var category:String = "All";
  public var selectedFamily:Null<String> = null;

  public function new() {}

  public function selectCategory(value:String):Void {
    category = value;
    selectedFamily = null;
  }

  public function search(value:String):Void {
    query = value;
    selectedFamily = null;
  }

  public static function families(entries:Array<ExampleEntry>):Array<ExampleFamily> {
    var result:Array<ExampleFamily> = [];
    var groups:Map<String, ExampleFamily> = new Map();
    for (entry in entries) {
      var id = familyId(entry.id);
      var group = groups.get(id);
      if (group == null) {
        group = {id: id, title: familyTitle(id, entry.title), category: categoryOf(entry.id), examples: []};
        groups.set(id, group);
        result.push(group);
      }
      group.examples.push(entry);
    }
    return result;
  }

  public function filtered(entries:Array<ExampleEntry>):Array<ExampleFamily> {
    var words = [for (word in StringTools.trim(query).toLowerCase().split(" ")) if (word.length > 0) word];
    return [for (family in families(entries)) if ((category == "All" || family.category == category) && matches(family, words)) family];
  }

  static function matches(family:ExampleFamily, words:Array<String>):Bool {
    // Match a variant as a whole, so unrelated words from separate variants do not produce a false result.
    for (entry in family.examples) {
      var text = (family.title + " " + family.category + " " + entry.title + " " + entry.description.join(" ")).toLowerCase();
      var match = true;
      for (word in words) if (text.indexOf(word) < 0) match = false;
      if (match) return true;
    }
    return false;
  }

  public static function familyId(id:String):String return switch id {
    case "robot-welder", "robot-welder-seam", "robot-welder-post", "robot-welder-weave", "robot-welder-multipass": "robot-welding";
    case "mobile-robot-welder", "mobile-welding-mission": "mobile-welding";
    case "cobot-reach500", "cobot-reach850", "cobot-reach900", "cobot-reach1300": "cobots";
    case "bench-mill", "enclosed-bench-mill": "bench-mills";
    default: id;
  };

  static function familyTitle(id:String, fallback:String):String return switch id {
    case "robot-welding": "Robot welding";
    case "mobile-welding": "Mobile welding";
    case "cobots": "Collaborative robots";
    case "bench-mills": "Bench mills";
    default: fallback;
  };

  public static function categoryOf(id:String):String return switch id {
    case "robot-welder", "robot-welder-seam", "robot-welder-post", "robot-welder-weave", "robot-welder-multipass",
      "mobile-robot-welder", "mobile-welding-mission", "gantry-welder", "track-welder": "Welding";
    case "bench-mill", "enclosed-bench-mill", "cnc-router": "Machining";
    case "motor-shaft-bearings", "cad-modeling": "CAD & design";
    case "two-robot", "worker-rack-to-table", "worker-gallery": "People & simulation";
    case "picking-station", "robot-arm", "cobot-reach500", "cobot-reach850", "cobot-reach900", "cobot-reach1300", "gantry-picker", "mobile-base": "Robotics & handling";
    default: throw 'Example "$id" needs a Start-page category';
  };

  public static function variantTitle(entry:ExampleEntry):String return switch entry.id {
    case "robot-welder": "Complete weldment";
    case "robot-welder-seam": "Single seam";
    case "robot-welder-post": "Tube post";
    case "robot-welder-weave": "Woven seam · 7 mm";
    case "robot-welder-multipass": "Three-pass seam · 10 mm";
    case "mobile-robot-welder": "Explore the cell";
    case "mobile-welding-mission": "Run the welding mission";
    default: entry.title;
  };
}
