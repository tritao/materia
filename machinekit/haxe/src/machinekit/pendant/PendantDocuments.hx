package machinekit.pendant;

import cadkit.parametric.DocumentCodec;
import cadkit.parametric.TimelineModel;
import materia.project.SceneArtifact.SceneArtifactHistory;
import materia.project.SceneSourceMap;

/**
 * The recorded plate, shell and board for one planned case, kept open together. The scene's geometry comes from these,
 * so the face numbers of each step are the scene's face numbers. Close it when the scene is built.
 */
class PendantDocuments {
  public final pendantCase:PendantCase;
  public final plate:PendantCaseDocument;
  public final shell:PendantCaseDocument;
  public final board:PendantCaseDocument;
  /** What the parts' occurrences are prefixed with in the scene (see `PendantMachine.prefix`). */
  public final prefix:String;

  public function new(pendantCase:PendantCase, prefix:String = "") {
    this.pendantCase = pendantCase;
    this.prefix = prefix;
    plate = PendantCaseDocument.plate(pendantCase);
    shell = PendantCaseDocument.shell(pendantCase);
    board = PendantCaseDocument.board(pendantCase);
  }

  public function close():Void {
    plate.document.close();
    shell.document.close();
    board.document.close();
  }

  /** Each part's recorded steps, for a viewer that rolls the part back. */
  public function histories():Array<SceneArtifactHistory> return [
    {occurrence: prefix + "case/plate", document: DocumentCodec.encode(plate.document)},
    {occurrence: prefix + "case/shell", document: DocumentCodec.encode(shell.document)},
    {occurrence: prefix + "case/board", document: DocumentCodec.encode(board.document)}
  ];

  /**
   * One source feature per top-level step of each part, in authoring order. A tool, a sketch or a placement is part of
   * the step that applies it, so it has no row of its own. The group leads the label (`Fasteners · Screw post`).
   */
  public function sources():SceneSourceMap {
    var features:Array<SceneSourceFeature> = [];
    for (part in [{occurrence: prefix + "case/plate", document: plate}, {occurrence: prefix + "case/shell", document: shell},
      {occurrence: prefix + "case/board", document: board}]) for (row in TimelineModel.of(part.document.document)) {
      if (row.parent >= 0) continue;
      features.push({
        id: part.occurrence + "#" + row.id,
        label: row.group == "" ? row.label : row.group + " · " + row.label,
        occurrence: part.occurrence,
        faces: row.faces,
        whole: false
      });
    }
    return {features: features, members: []};
  }
}
