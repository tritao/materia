import haxe.io.Bytes;
import machinekit.cache.Codecs;
import machinekit.cache.DiskStore;
import cadkit.modeling.Part;
import machinekit.cache.Memo;
import machinekit.cache.Memos;
import machinekit.component.Solids;
import machinekit.cache.Parallel;
import sys.FileSystem;

/** The persistent tier of `Memo`: round trips, invalidation, damage, eviction, verify mode, concurrent writers. */
class PersistentMemoTests {
  static var passed = 0;
  static var serial = 0;

  static function check(ok:Bool, message:String):Void {
    if (!ok) throw message;
    passed++;
  }

  static function freshStore(cap:Float = 1e9):DiskStore {
    serial++;
    var root = Sys.getEnv("TMPDIR");
    if (root == null || root == "") root = "/tmp";
    var directory = root + "/materia-memo-test-" + Sys.getPid() + "-" + serial;
    var store = new DiskStore(directory, cap);
    store.clear();
    return store;
  }

  static function memo(name:String, store:DiskStore, identity:String, runs:Array<Int>):Memo<Array<Float>> {
    return new Memo<Array<Float>>(name, 64, list -> list.copy()).persistIn(store, identity, Codecs.floats, Codecs.unfloats);
  }

  static function compute(runs:Array<Int>, seed:Float):Array<Float> {
    runs[0]++;
    return [seed, seed / 3, -0.0, 1e-300, 1e300, 0.1 + 0.2];
  }

  static function sameFloats(a:Array<Float>, b:Array<Float>):Bool {
    if (a.length != b.length) return false;
    for (i in 0...a.length) if (!(a[i] == b[i]) || 1 / a[i] != 1 / b[i]) return false;
    return true;
  }

  /** A second memo of the same name stands for the next process: it must find what the first stored. */
  static function roundTrip():Void {
    var store = freshStore();
    var runs = [0];
    var first = memo("persist-a", store, "id1", runs);
    var a = first.get("k1", () -> compute(runs, 7));
    check(runs[0] == 1 && store.count() == 1, "a miss computes once and stores one entry");
    first.get("k1", () -> compute(runs, 7));
    check(runs[0] == 1, "a memory hit computes nothing");
    var next = memo("persist-a", store, "id1", runs);
    var b = next.get("k1", () -> compute(runs, 99));
    check(runs[0] == 1, "a later memo finds the stored value without computing");
    check(sameFloats(a, b), "the stored value equals the computed one to the bit (including -0.0)");
    check(next.stats().diskHits == 1 && next.stats().misses == 1, "the disk hit is counted");
    b.push(1);
    check(next.get("k1", () -> compute(runs, 99)).length == 6, "a value read from disk is still copied out");
    next.get("k2", () -> compute(runs, 8));
    check(runs[0] == 2 && store.count() == 2, "another key is another entry");
    check(first.get(null, () -> compute(runs, 1)) != null && runs[0] == 3 && store.count() == 2, "a null key is never stored");
    store.clear();
  }

  static function invalidation():Void {
    var store = freshStore();
    var runs = [0];
    memo("persist-b", store, "id1", runs).get("k", () -> compute(runs, 1));
    memo("persist-b", store, "id2", runs).get("k", () -> compute(runs, 1));
    check(runs[0] == 2, "another code identity never reads the entry");
    memo("persist-c", store, "id1", runs).get("k", () -> compute(runs, 1));
    check(runs[0] == 3, "another memo name never reads the entry");
    memo("persist-b", store, "id1", runs).get("k", () -> compute(runs, 1));
    check(runs[0] == 3, "the original identity still hits");
    check(store.count() == 3, "each identity has its own entry");
    var offStore = new Memo<Array<Float>>("persist-d", 4);
    offStore.get("k", () -> compute(runs, 1));
    check(runs[0] == 4 && store.count() == 3, "a memo without the tier stores nothing");
    var nullIdentity = new Memo<Array<Float>>("persist-e", 4).persist(null, Codecs.floats, Codecs.unfloats);
    nullIdentity.get("k", () -> compute(runs, 1));
    nullIdentity.get("k2", () -> compute(runs, 1));
    check(runs[0] == 6, "a null identity leaves the tier off");
    store.clear();
  }

  static function damage():Void {
    var store = freshStore();
    var runs = [0];
    var key = "memo/1\npersist-f\nid1\nk";
    memo("persist-f", store, "id1", runs).get("k", () -> compute(runs, 5));
    var path = store.fileFor(key);
    check(FileSystem.exists(path), "the entry file is where the key says");
    sys.io.File.saveBytes(path, Bytes.ofString("garbage"));
    var value = memo("persist-f", store, "id1", runs).get("k", () -> compute(runs, 5));
    check(runs[0] == 2 && value.length == 6, "a damaged entry is a miss, recomputed");
    memo("persist-f", store, "id1", runs).get("k", () -> compute(runs, 5));
    check(runs[0] == 2, "and it was rewritten whole");
    // Truncated: a length header that promises more than is there.
    var good = sys.io.File.getBytes(path);
    sys.io.File.saveBytes(path, Bytes.view(good, 0, 10));
    memo("persist-f", store, "id1", runs).get("k", () -> compute(runs, 5));
    check(runs[0] == 3, "a truncated entry is a miss");
    sys.io.File.saveBytes(path, Bytes.alloc(0));
    memo("persist-f", store, "id1", runs).get("k", () -> compute(runs, 5));
    check(runs[0] == 4, "an empty entry is a miss");
    // An unwritable directory never fails the run.
    var blocked = new DiskStore("/proc/materia-memo-nowhere", 1e9);
    var m = memo("persist-g", blocked, "id1", runs);
    m.get("k", () -> compute(runs, 5));
    m.clear();
    m.get("k", () -> compute(runs, 5));
    check(runs[0] == 6, "an unwritable store only costs the speed-up");
    store.clear();
  }

  static function eviction():Void {
    var list = [for (i in 0...100) 1.0 * i];
    var entrySize = Codecs.floats(list).length + 0.0;
    var store = freshStore(entrySize * 10.5);
    var runs = [0];
    var m = memo("persist-h", store, "id1", runs);
    for (i in 0...40) m.get("k" + i, () -> {
      runs[0]++;
      return [for (j in 0...100) 1.0 * j + i];
    });
    store.evict();
    check(store.usage() <= entrySize * 10.5, "the store stays under its cap: " + store.usage() + " vs " + entrySize * 10.5);
    check(store.count() >= 5 && store.count() <= 10, "eviction keeps a useful number of entries: " + store.count());
    var count = store.count();
    store.evict();
    check(store.count() == count, "evicting under the cap deletes nothing");
    store.clear();
    check(store.count() == 0 && store.usage() == 0, "clear empties the store");
  }

  static function verify():Void {
    var store = freshStore();
    var runs = [0];
    memo("persist-i", store, "id1", runs).get("k", () -> compute(runs, 1));
    DiskStore.forceVerify(true);
    try {
      memo("persist-i", store, "id1", runs).get("k", () -> compute(runs, 1));
      check(runs[0] == 2, "verify mode recomputes a disk hit");
      var message = "";
      try memo("persist-i", store, "id1", runs).get("k", () -> compute(runs, 2)) catch (error:Dynamic) message = Std.string(error);
      check(message.indexOf("stale") >= 0 && message.indexOf("persist-i") >= 0, "verify mode fails on a differing recomputation: " + message);
      var fresh = memo("persist-i", store, "id1", runs).get("k2", () -> compute(runs, 3));
      check(fresh.length == 6, "verify mode still computes and stores a miss");
      memo("persist-i", store, "id1", runs).get("k2", () -> compute(runs, 3));
    } catch (error:Dynamic) {
      DiskStore.forceVerify(null);
      throw error;
    }
    DiskStore.forceVerify(null);
    store.clear();
  }

  static function concurrent():Void {
    if (Parallel.threads() < 2) Parallel.setThreads(4);
    var store = freshStore();
    var runs = [0];
    // Separate memos stand for separate processes writing the same keys at once.
    var results = Parallel.all([for (t in 0...8) () -> {
      var m = memo("persist-j", store, "id1", [0]);
      var sum = 0.0;
      for (round in 0...3) for (k in 0...12) sum += m.get("k" + k, () -> [for (j in 0...2000) 1.0 * k + j * 0.5])[k];
      sum;
    }]);
    for (r in results) check(r == results[0], "every thread saw the same values");
    check(store.count() == 12, "each key was stored exactly once: " + store.count());
    var again = memo("persist-j", store, "id1", runs);
    for (k in 0...12) {
      var v = again.get("k" + k, () -> {
        runs[0]++;
        return [];
      });
      check(v.length == 2000 && v[5] == 1.0 * k + 2.5, "stored values are whole after concurrent writes");
    }
    check(runs[0] == 0, "all twelve were served from disk");
    store.clear();
  }

  static function meshCodec():Void {
    var mesh = {positions: [0.1, -0.0, 1e-300, 123456789.123456789], normals: [0.0, 0.0, 1.0, 0.5], indices: [0, 1, 2, 2147483647, -1]};
    var back = Codecs.undisplayMesh(Codecs.displayMesh(mesh));
    check(sameFloats(back.positions, mesh.positions) && sameFloats(back.normals, mesh.normals), "mesh floats round-trip exactly");
    check(back.indices.join(",") == mesh.indices.join(","), "mesh indices round-trip");
    var empty = Codecs.undisplayMesh(Codecs.displayMesh({positions: [], normals: [], indices: []}));
    check(empty.positions.length == 0 && empty.indices.length == 0, "an empty mesh round-trips");
    var failed = false;
    try Codecs.undisplayMesh(Bytes.ofString("xx")) catch (error:Dynamic) failed = true;
    check(failed, "a short buffer is refused");
    var extra = Codecs.displayMesh(mesh);
    var padded = Bytes.alloc(extra.length + 1);
    for (i in 0...extra.length) padded.set(i, extra.get(i));
    failed = false;
    try Codecs.undisplayMesh(padded) catch (error:Dynamic) failed = true;
    check(failed, "trailing bytes are refused");
  }

  /** The compiler's module identities and the native build, wired through `persist`. */
  static function realIdentities():Void {
    var memoId = machinekit.cache.CodeIdentity.of("machinekit.cache.Memo");
    var storeId = machinekit.cache.CodeIdentity.of("machinekit.cache.DiskStore");
    check(memoId != null && memoId.length == 32, "a module in the program has a 32-character identity: " + memoId);
    check(storeId != null && storeId != memoId, "different modules have different identities");
    check(memoId == machinekit.cache.CodeIdentity.of("machinekit.cache.Memo"), "an identity is stable within a run");
    check(machinekit.cache.CodeIdentity.of("machinekit.no.SuchModule") == null, "an unknown module has no identity");
    var native = machinekit.cache.Memos.nativeIdentity();
    check(native != null && native.indexOf("cadkit:") == 0, "the CadKit build beside the program has an identity: " + native);
    check(native == machinekit.cache.Memos.nativeIdentity(), "and it is stable");

    var store = freshStore();
    DiskStore.useProcess(store);
    var runs = [0];
    function make(module:String):Memo<Array<Float>> return new Memo<Array<Float>>("persist-real", 8, list -> list.copy())
      .persist(machinekit.cache.CodeIdentity.of(module), Codecs.floats, Codecs.unfloats);
    make("machinekit.cache.Memo").get("k", () -> compute(runs, 1));
    make("machinekit.cache.Memo").get("k", () -> compute(runs, 1));
    check(runs[0] == 1 && store.count() == 1, "with real identities a second memo reads what the first stored");
    make("machinekit.cache.DiskStore").get("k", () -> compute(runs, 1));
    check(runs[0] == 2 && store.count() == 2, "another module's identity is another entry");
    make("machinekit.no.SuchModule").get("k", () -> compute(runs, 1));
    make("machinekit.no.SuchModule").get("k", () -> compute(runs, 1));
    check(runs[0] == 4 && store.count() == 2, "an unknown module leaves the tier off");
    DiskStore.useProcess(null);
    make("machinekit.cache.Memo").get("other", () -> compute(runs, 1));
    make("machinekit.cache.Memo").get("other", () -> compute(runs, 1));
    check(runs[0] == 6 && store.count() == 2, "with no process store (MATERIA_MEMO=off) nothing is stored");
    store.clear();
  }


  static function block(runs:Array<Int>, width:Float):Part {
    runs[0]++;
    return Solids.cut(Part.box(width, 20, 30), [Part.cylinderSpan(4, -5, 40, 2, 3), Part.cylinderSpan(2, 5, 40, -3, -6)]);
  }

  static function bytesOf(part:Part):String {
    var b = part.shape.toBytes();
    var buf = new StringBuf();
    for (i in 0...b.length) buf.add(StringTools.hex(b.get(i), 2));
    return buf.toString();
  }

  /** Solids: the value is the canonical form on a miss, a disk hit and with the tier off, and verify mode accepts it. */
  static function shapes():Void {
    var store = freshStore();
    DiskStore.useProcess(store);
    var runs = [0];
    var first = Memos.persistentParts("persist-shape", 8, "PersistentMemoTests");
    var a = first.get("k", () -> block(runs, 10));
    check(runs[0] == 1 && store.count() == 1, "a solid miss builds once and stores one entry");
    var aBytes = bytesOf(a);
    var again = first.get("k", () -> block(runs, 10));
    check(runs[0] == 1 && bytesOf(again) == aBytes, "a memory hit is the same bytes");
    // Meshing a handed-out copy must not change what the memo hands out next.
    a.shape.tessellate(0.1);
    check(bytesOf(first.get("k", () -> block(runs, 10))) == aBytes, "meshing a copy leaves the kept value alone");
    var next = Memos.persistentParts("persist-shape", 8, "PersistentMemoTests");
    var b = next.get("k", () -> block(runs, 10));
    check(runs[0] == 1 && next.stats().diskHits == 1, "a later memo restores the solid from disk without building");
    check(bytesOf(b) == aBytes, "the restored solid has the miss's bytes");
    check(b.shape.tessellate(0.1).vertexCount == a.shape.tessellate(0.1).vertexCount, "and the same mesh size");
    check(Math.abs(b.volume() - a.volume()) == 0, "and exactly the same volume");
    DiskStore.useProcess(null);
    var off = Memos.persistentParts("persist-shape", 8, "PersistentMemoTests");
    var c = off.get("k", () -> block(runs, 10));
    check(runs[0] == 2 && bytesOf(c) == aBytes, "with the tier off the computed value is the same canonical bytes");
    DiskStore.useProcess(store);
    DiskStore.forceVerify(true);
    try {
      var vm = Memos.persistentParts("persist-shape", 8, "PersistentMemoTests");
      var v = vm.get("k", () -> block(runs, 10));
      check(runs[0] == 3 && bytesOf(v) == aBytes, "verify mode recomputes and accepts an honest hit");
      var message = "";
      try Memos.persistentParts("persist-shape", 8, "PersistentMemoTests").get("k", () -> block(runs, 11)) catch (error:Dynamic) message = Std.string(error);
      check(message.indexOf("stale") >= 0, "verify mode refuses a solid whose recomputation differs: " + message);
    } catch (error:Dynamic) {
      DiskStore.forceVerify(null);
      DiskStore.useProcess(null);
      throw error;
    }
    DiskStore.forceVerify(null);
    DiskStore.useProcess(null);
    store.clear();
  }

  static function sceneBytes(part:materia.project.SceneArtifact.SceneArtifactPart):String {
    part.id = "p";
    part.name = "p";
    var b = materia.project.SceneArtifact.encode({lengthUnit: "mm", metresPerUnit: 0.001, parts: [part]});
    var buf = new StringBuf();
    for (i in 0...b.length) buf.add(StringTools.hex(b.get(i), 2));
    return buf.toString();
  }

  /** Scene parts: keyed on the solid's content, identical from memory, from disk and recomputed; other inputs differ. */
  static function sceneParts():Void {
    var store = freshStore();
    DiskStore.useProcess(store);
    Memos.clearAll();
    var runs = [0];
    var material = "steel-s355";
    try materia.project.MaterialLibrary.require(material) catch (error:Dynamic) material = new machinekit.hobbing.Hob().materialId;
    var make = (id:String, name:String, w:Float, m:String, deflection:Null<Float>) -> machinekit.assembly.AssemblyPreview.part(id, name, block(runs, w), m, deflection);
    var a = make("a", "A", 10, material, null);
    check(a.id == "a" && a.name == "A" && a.vertexCount > 0, "a part carries the caller's id and name");
    var aBytes = sceneBytes(a);
    var b = make("b", "B", 10, material, null);
    check(b.id == "b" && b.name == "B", "a hit carries the new caller's id and name");
    check(sceneBytes(b) == aBytes, "the same solid gives the same part from memory");
    check(store.count() >= 1, "the part was stored");
    Memos.clearAll();
    var c = make("c", "C", 10, material, null);
    check(sceneBytes(c) == aBytes, "and from disk");
    var diskHits = 0;
    for (line in Memos.report()) if (line.indexOf("scene-part") == 0 && line.indexOf("(1 from disk)") >= 0) diskHits++;
    check(diskHits == 1, "that was a disk hit");
    check(sceneBytes(make("d", "D", 11, material, null)) != aBytes, "another solid is another part");
    check(sceneBytes(make("e", "E", 10, material, 0.05)) != aBytes, "another tessellation deflection is another part");
    DiskStore.forceVerify(true);
    try {
      Memos.clearAll();
      check(sceneBytes(make("f", "F", 10, material, null)) == aBytes, "verify mode recomputes a scene part and agrees");
    } catch (error:Dynamic) {
      DiskStore.forceVerify(null);
      DiskStore.useProcess(null);
      throw error;
    }
    DiskStore.forceVerify(null);
    DiskStore.useProcess(null);
    store.clear();
  }

  /** The native identity names the build, not where OCCT is installed. */
  static function nativeIdentity():Void {
    var identity = Memos.nativeIdentity();
    check(identity != null, "a program with a CadKit build beside it has a native identity");
    check(identity == Memos.nativeIdentity(), "the native identity is stable within a run");
    check(identity.indexOf("occt:" + cadkit.Shape.occtVersion()) >= 0, "and carries the OCCT version: " + identity);
    for (variable in ["CADKIT_OCCT_DIR", "HOME"]) {
      var path = Sys.getEnv(variable);
      if (path != null && path.length > 3) check(identity.indexOf(path) < 0, "and no location (" + variable + ")");
    }
    check(haxeon.build.CodeIdentity.loadedLibraries("libNoSuchLibrary") == null, "no libraries match an unknown prefix");
  }

  public static function main():Void {
    roundTrip();
    invalidation();
    damage();
    eviction();
    verify();
    concurrent();
    meshCodec();
    shapes();
    nativeIdentity();
    sceneParts();
    realIdentities();
    Sys.println("persistent memo tests: " + passed + " checks passed");
  }
}
