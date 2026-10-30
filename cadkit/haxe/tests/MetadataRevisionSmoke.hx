import cadkit.parametric.Document;
import cadkit.parametric.DocumentCodec;
import cadkit.parametric.TypedProperty;
import cadkit.parametric.ElementReference;
import cadkit.parametric.DefinitionInput;
import cadkit.parametric.DefinitionOutput;
import cadkit.parametric.InstanceElement;

class MetadataRevisionSmoke {
  static function changed(doc:Document, mutate:Void->Void):Void {
    var before = doc.metadataRevision;
    mutate();
    if (doc.metadataRevision <= before) throw "Metadata revision did not advance";
  }
  static function main():Int {
    var doc = new Document();
    var membership = doc.metadataRevision;
    var first = doc.createObject("First");
    if (doc.metadataRevision <= membership) throw "Object insertion was not published";
    membership = doc.metadataRevision;
    var second = doc.createObject("Second");
    if (doc.metadataRevision <= membership) throw "Second object insertion was not published";
    changed(doc, () -> first.rename("Renamed"));
    changed(doc, () -> first.setProperty(TypedProperty.text("note", "one")));
    var unchanged = doc.metadataRevision;
    first.setProperty(TypedProperty.text("note", "one"));
    if (doc.metadataRevision != unchanged) throw "An identical property invalidated the model";
    changed(doc, () -> first.setProperty(TypedProperty.text("note", "two")));
    changed(doc, () -> { if (!doc.undo()) throw "Missing undo"; });
    changed(doc, () -> { if (!doc.redo()) throw "Missing redo"; });
    changed(doc, () -> first.removeProperty("note"));
    changed(doc, () -> first.restoreProperty("note", TypedProperty.integer("note", 4)));
    membership = doc.metadataRevision;
    var edge = doc.createRelationship("contains", new ElementReference(doc.id, first.id), new ElementReference(doc.id, second.id));
    if (doc.metadataRevision <= membership) throw "Relationship insertion was not published";
    changed(doc, () -> edge.restoreEndpoints(new ElementReference(doc.id, second.id), new ElementReference(doc.id, first.id)));
    changed(doc, () -> edge.setProperty(TypedProperty.boolean("enabled", true)));
    changed(doc, () -> doc.removeRelationship(edge.id));
    changed(doc, () -> { if (!doc.undo()) throw "Missing relationship undo"; });
    membership = doc.metadataRevision;
    var definition = doc.createDefinition("Type", "metadata-test", [new DefinitionInput("width", "scalar", "1", 10)],
      [new DefinitionOutput("body", DefinitionOutput.Geometry)]);
    if (doc.metadataRevision <= membership) throw "Definition insertion was not published";
    changed(doc, () -> definition.setDefault("width", 20));
    var oldRevision = definition.revision;
    changed(doc, () -> definition.restoreDefault("width", 10, oldRevision, false));
    changed(doc, () -> definition.setProperty(TypedProperty.text("note", "type")));
    var instance = new InstanceElement(doc, new cadkit.parametric.ElementId(), "Instance", definition.id);
    changed(doc, () -> instance.restoreOverride("width", 30));
    changed(doc, () -> instance.restoreDefinitionId(definition.id));
    changed(doc, () -> doc.restoreDefinitionRemoval(definition));
    changed(doc, () -> doc.restoreDefinitionInsertion(definition, 0));
    doc.removeRelationship(edge.id);
    changed(doc, () -> doc.removeElement(second.id));
    changed(doc, () -> { if (!doc.undo()) throw "Missing element undo"; });
    var decoded = DocumentCodec.decode(DocumentCodec.encode(doc), false, false);
    if (decoded.metadataRevision <= 0 || decoded.elementCount() != doc.elementCount()) throw "Codec did not publish metadata";
    decoded.close();
    changed(doc, doc.close);
    Sys.println("PASS: monotonic metadata revisions cover edits, restoration, history and codec membership");
    return 0;
  }
}
