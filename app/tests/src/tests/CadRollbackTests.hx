package tests;

import app.CadDocumentSession;
import app.CadPlateModel;

/** The rollback marker publishes the earlier shape without entering history, and a failed move changes nothing. */
class CadRollbackTests {
  static var checks = 0;
  static function check(ok:Bool, message:String):Void {
    checks++;
    if (!ok) throw message;
  }
  static function volume(session:CadDocumentSession):Float {
    var shape = session.copyPublishedShape();
    var result = shape.volume();
    shape.close();
    return result;
  }
  public static function main():Int {
    run();
    return 0;
  }
  public static function run():Void {
    var session = new CadDocumentSession(CadPlateModel.create(0.1, 0.05, 0.01, 0.01));
    try {
      var drilled = volume(session), revision = session.revision;
      session.rollTo(0);
      var block = volume(session);
      check(block > drilled + 1.0, "The base block is larger than the drilled plate: " + block + " / " + drilled);
      check(Math.abs(block - 100 * 50 * 10) < 1e-3, "The base block is the plate before the hole: " + block);
      check(session.document.rollbackIndex == 0 && session.revision > revision, "Rolling back publishes a new revision");
      session.rollTo(-1);
      check(Math.abs(volume(session) - drilled) < 1e-3, "Rolling forward restores the drilled plate");
      check(session.document.rollbackIndex == -1, "The marker is cleared");
      var before = session.revision, failed = false;
      try session.rollTo(99) catch (error:Dynamic) failed = true;
      check(failed, "An out-of-range marker is rejected");
      check(session.document.rollbackIndex == -1 && session.revision == before, "A rejected move changes nothing");
      check(Math.abs(volume(session) - drilled) < 1e-3, "A rejected move keeps the published shape");
      check(!session.document.undo(), "Moving the marker added nothing to history");
    } catch (error:Dynamic) {
      session.close();
      throw error;
    }
    session.close();
    Sys.println("CAD rollback: " + checks + " checks passed");
  }
}
