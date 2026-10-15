// Derive a shorter one-piece link while retaining both supplier end interfaces.
#include <STEPControl_Reader.hxx>
#include <STEPControl_Writer.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <BRepAlgoAPI_Common.hxx>
#include <BRepAlgoAPI_Cut.hxx>
#include <BRepAlgoAPI_Fuse.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopExp_Explorer.hxx>
#include <gp_Trsf.hxx>
#include <BRepOffsetAPI_ThruSections.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepTools.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Wire.hxx>
#include <gp_Vec.hxx>
#include <fstream>
#include <stdexcept>
#include <iostream>
static int solids(const TopoDS_Shape& s) {int n=0;for(TopExp_Explorer i(s,TopAbs_SOLID);i.More();i.Next())++n;return n;}
static double volume(const TopoDS_Shape& s) {GProp_GProps p;BRepGProp::VolumeProperties(s,p,1e-8);return p.Mass();}
static TopoDS_Shape band(const TopoDS_Shape& s,double low,double high) {
 BRepAlgoAPI_Common op(s,BRepPrimAPI_MakeBox(gp_Pnt(-200,low,-200),400,high-low,400).Shape());
 if(!op.IsDone())throw std::runtime_error("Link band failed");return op.Shape();
}
int main(int argc,char** argv) {
 try {
  if(argc!=4)throw std::runtime_error("usage: shorten-reference source.STEP output.STEP audit.json");
  STEPControl_Reader r;if(r.ReadFile(argv[1])!=IFSelect_RetDone||!r.TransferRoots())throw std::runtime_error("Import failed");
  auto source=r.OneShape();if(solids(source)!=1||!BRepCheck_Analyzer(source).IsValid())throw std::runtime_error("Source must be one valid solid");
  const double near=-130,far=-359,removed=189;
  gp_Trsf move;move.SetTranslation(gp_Vec(0,removed,0));
  auto proximal=band(source,near,200);
  auto originalDistal=band(source,-1000,far);
  auto distal=BRepBuilderAPI_Transform(originalDistal,move,true).Shape();
  auto cap=[](const TopoDS_Shape& shape,double y) {
    for(TopExp_Explorer e(shape,TopAbs_FACE);e.More();e.Next()) {
      auto face=TopoDS::Face(e.Current());BRepAdaptor_Surface surface(face);
      if(surface.GetType()==GeomAbs_Plane && std::abs(surface.Plane().Location().Y()-y)<1e-5 &&
          std::abs(surface.Plane().Axis().Direction().Y())>.999999) return face;
    }
    throw std::runtime_error("Missing cut section face");
  };
  auto a=cap(proximal,near),b=cap(distal,far+removed);
  auto outerA=BRepTools::OuterWire(a),outerB=BRepTools::OuterWire(b);
  TopoDS_Wire innerA,innerB;int countA=0,countB=0;
  for(TopExp_Explorer e(a,TopAbs_WIRE);e.More();e.Next()) if(!e.Current().IsSame(outerA)){innerA=TopoDS::Wire(e.Current());++countA;}
  for(TopExp_Explorer e(b,TopAbs_WIRE);e.More();e.Next()) if(!e.Current().IsSame(outerB)){innerB=TopoDS::Wire(e.Current());++countB;}
  if(countA!=1||countB!=1)throw std::runtime_error("Link sections must each have one continuous hollow duct");
  auto loft=[](const TopoDS_Wire& a,const TopoDS_Wire& b){
    BRepOffsetAPI_ThruSections op(true,false,1e-7);op.AddWire(a);op.AddWire(b);op.Build();
    if(!op.IsDone())throw std::runtime_error("Transition loft failed");return op.Shape();
  };
  BRepAlgoAPI_Cut hollow(loft(outerA,outerB),loft(innerA,innerB));
  if(!hollow.IsDone())throw std::runtime_error("Hollow transition failed");
  BRepAlgoAPI_Fuse first(proximal,hollow.Shape());
  if(!first.IsDone())throw std::runtime_error("Proximal transition fuse failed");
  BRepAlgoAPI_Fuse joined(first.Shape(),distal);joined.SimplifyResult();
  auto out=joined.Shape();if(!joined.IsDone()||solids(out)!=1||!BRepCheck_Analyzer(out).IsValid())throw std::runtime_error("Shortened link must be one connected valid solid");
  if(volume(out)>=volume(source))throw std::runtime_error("Shortening did not remove material");
  STEPControl_Writer w;if(w.Transfer(out,STEPControl_AsIs)!=IFSelect_RetDone||w.Write(argv[2])!=IFSelect_RetDone)throw std::runtime_error("Export failed");
  std::ofstream audit(argv[3]);audit.precision(17);
  audit<<"{\"nearCutY\":-130,\"farCutY\":-359,\"removedMm\":189,\"transitionLengthMm\":40,\"method\":\"preserve-ends-and-hollow-loft\",\"validConnectedSolid\":true,\"sourceVolumeMm3\":"<<volume(source)<<",\"derivedVolumeMm3\":"<<volume(out)<<"}\n";
  return 0;
 }catch(const std::exception& e){std::cerr<<e.what()<<"\n";return 1;}
}
