class AssemblyFocused {
  public static function main():Void {
    AssemblyModelSmoke.run();
    AssemblyDocumentsSmoke.run();
    AssemblyCouplingSmoke.run();
    AssemblyLoopSmoke.run();
    AssemblyDragSmoke.run();
  }
}
