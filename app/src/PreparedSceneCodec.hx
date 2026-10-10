package app;

import haxe.Json;
import haxe.crypto.Sha256;
import haxe.io.Bytes;
import haxe.io.BytesOutput;
import cadbridge.AssemblySimulationBridge.AssemblyPhysicalPart;
import app.PreparedProjectPart;

/** Validated portable binary data shared by cache storage and worker messages. */
class PreparedSceneCodec {
  static inline var LIMIT:Int = 150000000;
  public static function decode(bytes:Bytes, parts:Array<materia.project.SceneArtifact.SceneArtifactPart>):Array<PreparedProjectPart> {
    if (bytes.length > LIMIT) throw "Prepared cache is too large";
    if (bytes.length < 40 || bytes.getInt32(0) != 0x5052544d) throw "Invalid prepared cache header";
    var end = bytes.length - 32;
    var digest = Sha256.make(Bytes.view(bytes, 0, end));
    for (i in 0...32) if (bytes.get(end + i) != digest.get(i)) throw "Prepared cache checksum mismatch";
    var length = bytes.getInt32(4);
    if (length <= 0 || length > end - 8) throw "Invalid prepared cache metadata";
    var entries:Array<Dynamic> = Json.parse(bytes.getString(8, length));
    if (entries.length != parts.length) throw "Prepared cache part count mismatch";
    var offset = 8 + length;
    var result:Array<PreparedProjectPart> = [];
    for (i in 0...parts.length) {
      var source = parts[i], entry = entries[i];
      if (entry.id != source.id) throw "Prepared cache part identity mismatch";
      var hullCount:Int = entry.hullCount;
      if (hullCount < 12 || hullCount > cadkit.ConvexHullVertices.MAX_VERTICES * 3 || hullCount % 3 != 0)
        throw "Invalid prepared hull count";
      var size = source.vertexCount * 12;
      if (size <= 0 || size > Std.int((end - offset - (21 + hullCount) * 8) / 2))
        throw "Invalid prepared streams";
      function numbers(count:Int):Array<Float> {
        var values:Array<Float> = [];
        for (index in 0...count) { values.push(bytes.getDouble(offset)); offset += 8; }
        return values;
      }
      var minimum = numbers(3), maximum = numbers(3), scalars = numbers(3);
      finiteArray(minimum, 3); finiteArray(maximum, 3);
      for (axis in 0...3) if (minimum[axis] > maximum[axis]) throw "Invalid prepared bounds";
      var physical:AssemblyPhysicalPart = {id: source.id, materialId: entry.materialId,
        volume: scalars[0], density: scalars[1], collisionErrorRatio: scalars[2],
        centerOfMass: numbers(3), inertia: numbers(9), collisionHull: numbers(hullCount),
        collisionWarning: entry.collisionWarning};
      var positions = Bytes.view(bytes, offset, size), normals = Bytes.view(bytes, offset + size, size);
      offset += size * 2;
      if (physical.id != source.id || physical.materialId == null || !Math.isFinite(physical.volume) ||
          physical.volume < 0 || !Math.isFinite(physical.density) || physical.density <= 0)
        throw "Invalid prepared physical properties";
      finiteArray(physical.centerOfMass, 3); finiteArray(physical.inertia, 9);
      if (physical.collisionHull == null || physical.collisionHull.length % 3 != 0 ||
          physical.collisionHull.length > cadkit.ConvexHullVertices.MAX_VERTICES * 3)
        throw "Invalid prepared collision hull";
      finiteArray(physical.collisionHull, physical.collisionHull.length);
      var errorRatio = physical.collisionErrorRatio;
      if (errorRatio == null || !Math.isFinite(errorRatio)) throw "Invalid prepared hull error";
      result.push({id: source.id, geometry: {minimum: minimum, maximum: maximum,
        positions: positions, normals: normals}, physical: physical});
    }
    if (offset != end) throw "Trailing prepared cache data";
    return result;
  }

  public static function encode(parts:Array<PreparedProjectPart>):Bytes {
    var entries = [for (part in parts) {
      var hull = part.physical.collisionHull;
      if (hull == null) throw "Missing collision hull";
      {id: part.id, materialId: part.physical.materialId,
        collisionWarning: part.physical.collisionWarning, hullCount: hull.length};
    }];
    var metadata = Bytes.ofString(Json.stringify(entries));
    var output = new BytesOutput();
    output.setBigEndian(false);
    output.writeInt32(0x5052544d);
    output.writeInt32(metadata.length);
    output.write(metadata);
    for (part in parts) {
      var errorRatio = part.physical.collisionErrorRatio;
      var hull = part.physical.collisionHull;
      if (errorRatio == null || hull == null) throw "Missing collision properties";
      // Preserve every float64 bit, including small inertias and signed zero. JSON is only for labels.
      for (values in [part.geometry.minimum, part.geometry.maximum,
          [part.physical.volume, part.physical.density, errorRatio],
          part.physical.centerOfMass, part.physical.inertia, hull])
        for (value in values) output.writeDouble(value);
      output.write(part.geometry.positions); output.write(part.geometry.normals);
    }
    var payload = output.getBytes();
    if (payload.length > LIMIT - 32) throw "Prepared data is too large";
    var bytes = Bytes.alloc(payload.length + 32);
    bytes.blit(0, payload, 0, payload.length);
    bytes.blit(payload.length, Sha256.make(payload), 0, 32);
    return bytes;
  }

  static function finiteArray(values:Array<Float>, length:Int):Void {
    if (values == null || values.length != length) throw "Invalid prepared numeric array";
    for (value in values) if (!Math.isFinite(value)) throw "Non-finite prepared value";
  }

}
