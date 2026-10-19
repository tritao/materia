package machinekit.enclosure;

import machinekit.assembly.PosedParts.PartBox;

typedef EnclosureLayoutInput = {
  var swept:PartBox;
  var loading:PartBox;
  var plinth:PartBox;
  var floorZ:Float;
  var margin:Float;
  var openingWidth:Float;
}

/** A front-loading shell fitted to declared swept bounds; all dimensions in machine-frame mm. */
class EnclosureLayout {
  public final input:EnclosureLayoutInput;
  public final lowX:Float;
  public final highX:Float;
  public final frontY:Float;
  public final rearY:Float;
  public final roofZ:Float;
  public final doorX:Float;
  public final doorWidth:Float;
  public final stroke:Float;
  public function new(input:EnclosureLayoutInput) {
    for(box in [input.swept,input.loading,input.plinth])
      if(!Math.isFinite(box.minX+box.minY+box.minZ+box.maxX+box.maxY+box.maxZ)||box.minX>=box.maxX||box.minY>=box.maxY||box.minZ>=box.maxZ)
        throw "Enclosure layout needs finite nonempty bounds";
    if(!(input.margin>=50)||!(input.openingWidth>0)||!Math.isFinite(input.margin+input.openingWidth+input.floorZ))throw "Invalid enclosure layout allowances";
    this.input=input;doorX=(input.loading.minX+input.loading.maxX)/2;
    doorWidth=Math.max(input.openingWidth,input.loading.maxX-input.loading.minX+2*input.margin);stroke=doorWidth+10;
    lowX=Math.min(input.swept.minX-input.margin,doorX-doorWidth/2-input.margin);
    highX=Math.max(input.swept.maxX+input.margin,doorX+doorWidth/2+stroke+input.margin);
    frontY=input.swept.minY-input.margin;rearY=input.swept.maxY+input.margin;
    roofZ=Math.max(input.swept.maxZ,input.loading.maxZ)+input.margin;
    if(input.floorZ+40>=input.loading.minZ || input.floorZ>=roofZ)throw "Tray datum obstructs the loading opening";
    if(input.plinth.minX<=lowX || input.plinth.maxX>=highX || input.plinth.minY<=frontY || input.plinth.maxY>=rearY)
      throw "Plinth interface must fit inside tray";
  }
  public function width():Float return highX-lowX;
  public function depth():Float return rearY-frontY;
  public function height():Float return roofZ-input.floorZ;
  public function centreX():Float return (lowX+highX)/2;
  public function centreY():Float return (frontY+rearY)/2;
}
