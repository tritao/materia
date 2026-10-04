package robotkit.auth;

import haxe.Json;
import haxe.io.Path;
import haxe.crypto.Sha256;

/** Strict, versioned operator configuration. No anonymous or loopback fallback. */
class RobotAuthorization {
  final principals:Map<String, AuthPrincipal> = [];
  final hashes:Map<String, String> = [];
  final deployments:Map<String, String> = [];
  public function new(path:String) {
    if (path == null || path.length == 0) throw "robotd requires --auth=FILE";
    var root:Dynamic = Json.parse(sys.io.File.getContent(path));
    if (Reflect.field(root,"schemaVersion") != 1) throw "Unsupported robot authorization schema version";
    keys(root,["schemaVersion","identities","deployments"]);
    var identities:Dynamic = Reflect.field(root,"identities");
    if (!Std.isOfType(identities,Array) || (cast identities:Array<Dynamic>).length == 0)
      throw "Authorization configuration requires identities";
    for (value in (cast identities:Array<Dynamic>)) {
      keys(value,["id","tokenSha256","permissions"]);
      var id = text(value,"id"), digest = text(value,"tokenSha256");
      if (principals.exists(id) || !~/^[0-9a-f]{64}$/.match(digest)) throw "Invalid or duplicate authorization identity";
      var raw:Dynamic = Reflect.field(value,"permissions");
      if (!Std.isOfType(raw,Array)) throw "Authorization permissions must be an array";
      var permissions:Array<String> = [];
      for (permission in (cast raw:Array<Dynamic>)) {
        if ((permission != "observe" && permission != "command" && permission != "deployment") || permissions.contains(permission))
          throw "Unknown or duplicate authorization permission";
        permissions.push(permission);
      }
      principals.set(id,new AuthPrincipal(id,permissions)); hashes.set(id,digest);
    }
    var configured:Dynamic = Reflect.field(root,"deployments");
    if (configured != null) for (name in Reflect.fields(configured)) {
      var relative = text(configured,name);
      deployments.set(name,Path.isAbsolute(relative) ? relative : Path.join([Path.directory(path),relative]));
    }
  }
  public function authenticate(identity:String, token:String):AuthPrincipal {
    if (identity == null || token == null || token.length < 16 || token.length > 4096)
      throw "Authentication failed";
    var expected = hashes.get(identity);
    var actual = tokenDigest(token);
    var different = 0;
    // Compare all digest bytes even for an unknown identity.
    var reference = expected == null ? StringTools.lpad("", "0", 64) : expected;
    for (index in 0...64) different |= StringTools.fastCodeAt(reference,index) ^ StringTools.fastCodeAt(actual,index);
    if (expected == null || different != 0) throw "Authentication failed";
    return principals.get(identity);
  }
  public function deploymentPath(principal:AuthPrincipal, name:String):String {
    if (principal == null || !principal.mayDeploy) throw "Deployment permission denied";
    var path = deployments.get(name);
    if (path == null) throw "Deployment is not configured";
    return path;
  }
  public static function tokenDigest(token:String):String {
    var digest = Sha256.make(haxe.io.Bytes.ofString(token));
    var result = new StringBuf();
    for (i in 0...digest.length) result.add(StringTools.hex(digest.get(i),2).toLowerCase());
    return result.toString();
  }
  static function text(value:Dynamic,key:String):String {
    var result:Dynamic = Reflect.field(value,key);
    if (!Std.isOfType(result,String) || StringTools.trim(result).length == 0) throw "Authorization text field is required";
    return result;
  }
  static function keys(value:Dynamic,allowed:Array<String>):Void {
    if (value == null) throw "Authorization record is required";
    for (key in Reflect.fields(value)) if (!allowed.contains(key)) throw "Unknown authorization field";
  }
}
