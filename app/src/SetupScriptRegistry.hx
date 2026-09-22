package app;

import app.examples.TwoRobotSetupScript;

/** Explicit allow-list of compiled, unsandboxed Haxe setup scripts. */
class SetupScriptRegistry {
  static final providers:Map<String,Void->SetupScript> = builtins();
  static function builtins():Map<String,Void->SetupScript>{
    var result=new Map<String,Void->SetupScript>();
    result.set(TwoRobotSetupScript.REFERENCE,function()return new TwoRobotSetupScript());
    return result;
  }
  public static function references():Array<String>{var result=[for(key in providers.keys())key];result.sort(Reflect.compare);return result;}
  public static function provider(reference:String):Void->SetupScript {
    var result=providers.get(reference);if(result==null)throw 'Unknown setup script "$reference"';return result;
  }
  /** Register or replace an explicitly allowed compiled Haxe setup provider. */
  public static function register(reference:String,factory:Void->SetupScript):Void {
    if(reference==null||StringTools.trim(reference).length==0||factory==null)
      throw "Invalid setup script registration";
    providers.set(reference,factory);
  }
}
