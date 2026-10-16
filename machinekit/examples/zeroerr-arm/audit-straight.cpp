// Independent checks on exported/reimported prototype parts, not in-memory generator shapes.
#include <STEPControl_Reader.hxx>
#include <BRepAlgoAPI_Common.hxx>
#include <BRepAlgoAPI_Fuse.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopExp_Explorer.hxx>
#include <Standard_Failure.hxx>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <stdexcept>
#include <cmath>
struct Part {std::string role,path;TopoDS_Shape shape;};
static int solids(const TopoDS_Shape&s){int n=0;for(TopExp_Explorer e(s,TopAbs_SOLID);e.More();e.Next())++n;return n;}
static double volume(const TopoDS_Shape&s){GProp_GProps p;BRepGProp::VolumeProperties(s,p,1e-8);return std::abs(p.Mass());}
static double overlap(const TopoDS_Shape&a,const TopoDS_Shape&b){BRepAlgoAPI_Common op(a,b);if(!op.IsDone()||!BRepCheck_Analyzer(op.Shape()).IsValid())throw std::runtime_error("Indeterminate exported-part overlap");return solids(op.Shape())?volume(op.Shape()):0;}
int main(int n,char**v){try{
 if(n<9||(n-5)%2)throw std::runtime_error("usage: audit-straight bore_mm lowY highY output.json role file ...");
 double bore=std::stod(v[1]),low=std::stod(v[2]),high=std::stod(v[3]);
 auto duct=BRepPrimAPI_MakeCylinder(gp_Ax2(gp_Pnt(0,low,0),gp_Dir(0,1,0)),bore/2-.001,high-low).Shape();
 std::vector<Part> parts;int structural=0;TopoDS_Shape joined;double blocked=0,coverOverlap=0;
 for(int i=5;i<n;i+=2){STEPControl_Reader r;if(r.ReadFile(v[i+1])!=IFSelect_RetDone||!r.TransferRoots())throw std::runtime_error("Exported part import failed");
  auto shape=r.OneShape();if(solids(shape)!=1||!BRepCheck_Analyzer(shape).IsValid()||volume(shape)<=0)throw std::runtime_error("Exported part must be one valid solid");
  std::string role=v[i];parts.push_back({role,v[i+1],shape});blocked+=overlap(shape,duct);
  if(role=="tube"||role=="adapter"){
   ++structural;if(joined.IsNull())joined=shape;else{BRepAlgoAPI_Fuse f(joined,shape);if(!f.IsDone())throw std::runtime_error("Exported structural join failed");joined=f.Shape();}
  }
 }
 if(structural!=3||solids(joined)!=1||!BRepCheck_Analyzer(joined).IsValid())throw std::runtime_error("Exported structural pieces do not form one valid connected solid");
 for(size_t i=0;i<parts.size();++i)if(parts[i].role=="cover")for(size_t j=0;j<parts.size();++j)
  if(i!=j&&(parts[j].role!="cover"||i<j))coverOverlap+=overlap(parts[i].shape,parts[j].shape);
 std::ofstream out(v[4]);out.precision(17);out<<"{\"importedValidSolids\":"<<parts.size()<<",\"joinedStructuralSolidValid\":true,\"testedAxialDuctDiameterMm\":"<<bore-.002<<",\"ductMaterialOverlapMm3\":"<<blocked<<",\"coverMaterialOverlapMm3\":"<<coverOverlap<<",\"indeterminateQueries\":0}\n";
 if(blocked>1e-5||coverOverlap>1e-5)throw std::runtime_error("Exported parts block the duct or collide with the covers");
 return 0;
 }catch(const Standard_Failure&e){std::cerr<<e.what()<<"\n";return 1;}catch(const std::exception&e){std::cerr<<e.what()<<"\n";return 1;}}
