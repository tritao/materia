package app;

/** Compiled Haxe setup script. Evaluation returns data/configuration only. */
interface SetupScript {
  function reference():String;
  function version():Int;
  function evaluate(output:ScriptedSetup):Void;
}
