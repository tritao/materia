package machinekit.component;

import machinekit.assembly.Diagnostics.Diagnostic;
import materia.assembly.AssemblyRecord.AssemblyFrame;

/**
 * Value comparisons for plain records and arrays. Records and arrays are reference types, so
 * `Equality.equals` compares them by identity; two equal descriptions must be compared by content.
 * Wire records are compared at the call site by `JsonWire.encode`, the content a document stores.
 */
class Same {
  public static function strings(a:Array<String>, b:Array<String>):Bool {
    if (a.length != b.length) return false;
    for (i in 0...a.length) if (a[i] != b[i]) return false;
    return true;
  }

  public static function frame(a:AssemblyFrame, b:AssemblyFrame):Bool return a.x == b.x && a.y == b.y && a.z == b.z
    && a.qx == b.qx && a.qy == b.qy && a.qz == b.qz && a.qw == b.qw;

  public static function connectors(a:Array<Connector>, b:Array<Connector>):Bool {
    if (a.length != b.length) return false;
    for (i in 0...a.length) if (a[i].name != b[i].name || a[i].role != b[i].role || !frame(a[i].frame, b[i].frame)) return false;
    return true;
  }

  public static function ports(a:Array<ComponentPort>, b:Array<ComponentPort>):Bool {
    if (a.length != b.length) return false;
    for (i in 0...a.length) if (a[i].name != b[i].name || a[i].kind != b[i].kind || a[i].role != b[i].role
      || a[i].iface != b[i].iface || a[i].required != b[i].required || a[i].connector != b[i].connector) return false;
    return true;
  }

  public static function diagnostics(a:Array<Diagnostic>, b:Array<Diagnostic>):Bool {
    if (a.length != b.length) return false;
    for (i in 0...a.length) if (a[i].severity != b[i].severity || a[i].code != b[i].code || a[i].subject != b[i].subject
      || a[i].message != b[i].message) return false;
    return true;
  }

  /** Purchasable identity of each line: what the quote and the part-number key depend on. */
  public static function bom(a:Array<BomItem>, b:Array<BomItem>):Bool {
    if (a.length != b.length) return false;
    for (i in 0...a.length) {
      var x = a[i], y = b[i];
      if (x.partNumber != y.partNumber || x.description != y.description || x.quantity != y.quantity
        || x.material != y.material || x.familyId != y.familyId || x.manufacturingVariant != y.manufacturingVariant
        || x.manufacturingProcess != y.manufacturingProcess || x.typeId != y.typeId || x.valuesKey != y.valuesKey
        || x.fingerprint != y.fingerprint || x.maturity != y.maturity) return false;
    }
    return true;
  }
}
