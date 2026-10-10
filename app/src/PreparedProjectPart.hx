package app;

import app.PortableCadGeometry.PreparedCadGeometry;
import cadbridge.AssemblySimulationBridge.AssemblyPhysicalPart;

typedef PreparedProjectPart = {
  var id:String;
  var geometry:PreparedCadGeometry;
  var physical:AssemblyPhysicalPart;
};

