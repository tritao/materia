typedef StraightArmPaths = {var layoutPath:String; var vendorDirectory:String; var preparationPath:String;}

/** Resolve the immutable prepared variant and reject stale design/generator inputs. */
class StraightArmPreparation {
  public static final DIRECTORY = "machinekit/examples/zeroerr-arm";
  public static function paths():StraightArmPaths {
    var path = DIRECTORY+"/build/straight/current.json";
    if (!sys.FileSystem.exists(path)) throw "Prepare the straight variant first: run zeroerr-arm/prepare-straight.py";
    var raw:Dynamic = haxe.Json.parse(sys.io.File.getContent(path));
    var result:StraightArmPaths = {layoutPath:Std.string(raw.layoutPath),vendorDirectory:Std.string(raw.vendorDirectory),
      preparationPath:Std.string(raw.preparationPath)};
    var record:Dynamic = haxe.Json.parse(sys.io.File.getContent(result.preparationPath));
    if (Std.string(record.inputs.configSha256) != haxe.crypto.Sha256.encode(sys.io.File.getContent(DIRECTORY+"/straight-link-config.json")) ||
        Std.string(record.inputs.generatorSha256) != haxe.crypto.Sha256.encode(sys.io.File.getContent(DIRECTORY+"/straight-link.cpp")) ||
        Std.string(record.inputs.preparerSha256) != haxe.crypto.Sha256.encode(sys.io.File.getContent(DIRECTORY+"/prepare-straight.py")) ||
        Std.string(record.inputs.baselineLayoutSha256) != haxe.crypto.Sha256.encode(sys.io.File.getContent(DIRECTORY+"/revised-layout.json")) ||
        Std.string(Reflect.field(record.files,"layout.json")) != haxe.crypto.Sha256.encode(sys.io.File.getContent(result.layoutPath)))
      throw "Straight variant is stale or changed; rerun prepare-straight.py";
    return result;
  }
}
