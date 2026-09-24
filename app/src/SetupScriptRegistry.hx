package app;

import app.examples.TwoRobotSetupScript;

/** Explicit allow-list of compiled, unsandboxed Haxe setup scripts. */
class SetupScriptRegistry {
  public static inline var TWO_ROBOT_SOURCE_SHA256 = ScriptSourceManifest.TWO_ROBOT_SHA256;
  static final providers:Map<String,Void->SetupScript> = builtins();
  static final identities:Map<String,ScriptPackageIdentity> = builtinIdentities();
  static function builtinIdentities():Map<String,ScriptPackageIdentity> {
    var result = new Map<String,ScriptPackageIdentity>();
    result.set(TwoRobotSetupScript.REFERENCE, {
      packageId: "materia.examples",
      packageVersion: "1",
      sourceSha256: TWO_ROBOT_SOURCE_SHA256
    });
    return result;
  }
  static function builtins():Map<String,Void->SetupScript>{
    var result=new Map<String,Void->SetupScript>();
    result.set(TwoRobotSetupScript.REFERENCE,function()return new TwoRobotSetupScript());
    return result;
  }
  public static function references():Array<String>{var result=[for(key in providers.keys())key];result.sort(Reflect.compare);return result;}
  public static function provider(reference:String):Void->SetupScript {
    var result=providers.get(reference);if(result==null)throw 'Unknown setup script "$reference"';return result;
  }
  public static function identity(reference:String):ScriptPackageIdentity {
    var result = identities.get(reference);
    if (result == null) throw 'Unknown setup script identity "$reference"';
    return {packageId: result.packageId, packageVersion: result.packageVersion,
      sourceSha256: result.sourceSha256};
  }
  /** Register or replace an explicitly allowed compiled Haxe setup provider. */
  public static function register(reference:String,factory:Void->SetupScript,
      packageIdentity:ScriptPackageIdentity):Void {
    if(reference==null||StringTools.trim(reference).length==0||factory==null
      || packageIdentity==null || StringTools.trim(packageIdentity.packageId).length==0
      || StringTools.trim(packageIdentity.packageVersion).length==0
      || !~/^[0-9a-f]{64}$/.match(packageIdentity.sourceSha256))
      throw "Invalid setup script registration";
    providers.set(reference,factory);
    identities.set(reference, {packageId: packageIdentity.packageId,
      packageVersion: packageIdentity.packageVersion,
      sourceSha256: packageIdentity.sourceSha256});
  }
}
