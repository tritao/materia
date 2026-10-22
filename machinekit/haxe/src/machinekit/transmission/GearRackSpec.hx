package machinekit.transmission;

import machinekit.catalog.Catalog;
import machinekit.catalog.CatalogMetadata.DimensionKind;
import machinekit.catalog.CatalogMetadata.Conformance;

typedef GearRackSpec = {
    var designation:String;
    var normalModule:Float;
    var lengthMm:Float;
    var teeth:Int;
    var faceWidthMm:Float;
    var faceWidthPlusToleranceMm:Float;
    var overallHeightMm:Float;
    var pitchHeightMm:Float;
    var holeHeightMm:Float;
    var holePitchMm:Float;
    var holeEndMm:Float;
    var holeCount:Int;
    var boreMm:Float;
    var counterboreMm:Float;
    var counterboreDepthMm:Float;
    var pinEndMm:Float;
    var pinPilotMm:Float;
    var massKg:Float;
    var force15N:Float;
    var force20N:Float;
    var sectionForceFactor:Float;
}

/** ATLANTA HPR quality 6, heat-treatable steel; table rows, not interpolated sizes. */
class GearRackCatalog {
    public static inline var SOURCE="https://www.motiontech.com.au/wp-content/uploads/2026/01/Atlanta-Servo-Drive-Systems.pdf";
    public static inline var PDF_SHA256="b0666c07fd2f0d0ec97c4541138a997e31de71a6871fbc53b117fc7b5cbcae4e";
    static var table:Null<Catalog<GearRackSpec>>;
    static function row(code:String,module:Float,length:Float,teeth:Int,width:Float,pitchHeight:Float,mass:Float):GearRackSpec
        return {designation:code,normalModule:module,lengthMm:length,teeth:teeth,
            faceWidthMm:width,faceWidthPlusToleranceMm:0.4,overallHeightMm:width,pitchHeightMm:pitchHeight,
            holeHeightMm:8,holePitchMm:125,holeEndMm:62.5,holeCount:Std.int(length/125),
            boreMm:7,counterboreMm:11,counterboreDepthMm:7,pinEndMm:31.7,pinPilotMm:5.7,massKg:mass,
            force15N:module==1.5?4500:8000,force20N:module==1.5?6000:11500,
            sectionForceFactor:length==500?0.5:1};
    public static function catalog():Catalog<GearRackSpec> {
        if(table==null)table=new Catalog("ATLANTA HPR helical rack",r->r.designation,[
            row("29-15-055",1.5,500,100,19,17.5,1.30),
            row("29-15-105",1.5,1000,200,19,17.5,2.60),
            row("29-20-055",2,500,75,24,22,2.10),
            row("29-20-105",2,1000,150,24,22,4.10)
        ],r->({source:SOURCE+"#page=76",standard:"ATLANTA HPR quality 6",standardEdition:"manufacturer catalogue retrieved 2026-10-22; PDF SHA256 "+PDF_SHA256,
            dimensionKind:Mixed,conformance:NominalEnvelope,sources:[SOURCE+"#page=101",SOURCE+"#page=102",SOURCE+"#page=148",SOURCE+"#page=150"],
            verifiedFields:["designation","normalModule","lengthMm","teeth","faceWidthMm","faceWidthPlusToleranceMm","overallHeightMm","pitchHeightMm","holeHeightMm","holePitchMm","holeEndMm","holeCount","boreMm","counterboreMm","counterboreDepthMm","pinEndMm","pinPilotMm","massKg","force15N","force20N","sectionForceFactor"]}));
        return table;
    }
}
