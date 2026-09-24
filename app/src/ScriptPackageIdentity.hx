package app;

/** Identity of the compiled package that supplies a setup script. */
typedef ScriptPackageIdentity = {
  var packageId:String;
  var packageVersion:String;
  var sourceSha256:String;
}
