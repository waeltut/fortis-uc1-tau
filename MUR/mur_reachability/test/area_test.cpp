#include "mur_reachability/area.hpp"
#include <cassert>
#include <iostream>
int main() {
  mr::SparseArea left,right;
  mr::insert_area(left,{0,0,0},2);mr::insert_area(right,{0,0,1},2);
  assert(mr::intersect_areas(left,right).empty()); // XY overlap is insufficient.
  mr::insert_area(right,{0,0,0},1);
  auto both=mr::intersect_areas(left,right);assert(both.at({0,0,0})==1);
  mr::insert_area(right,{0,0,0},2);
  both=mr::intersect_areas(left,right);assert(both.at({0,0,0})==2);
  mr::insert_area(left,{-2,3,23},1);
  std::array<mr::SparseArea,3> areas{left,right,both};auto bounds=mr::common_bounds(areas,24);
  assert(bounds.x==-2&&bounds.y==0&&bounds.width==3&&bounds.height==4);
  auto dense=mr::dense_area(left,bounds);assert(dense[bounds.index({-2,3,23})]==1);
  assert(dense[bounds.index({-1,2,5})]==0); // Unknown stays unknown.
  mr::Projection ring;
  for(int x=0;x<3;++x)for(int y=0;y<3;++y)if(x!=1||y!=1)ring[{x,y}]=1;
  assert(mr::boundary(ring).size()==16); // 12 outer edges + 4 hole edges.
  mr::Projection pair{{{0,0},1},{{1,0},1}};assert(mr::boundary(pair).size()==6);
  mr::Projection separate{{{0,0},1},{{5,5},1}};assert(mr::boundary(separate).size()==8);
  assert(mr::area_key(.025,-.025,0,.05,24)==mr::AreaKey(0,-1,0));
  assert(mr::area_key(.025,-.025,6.283185307179586,.05,24)==mr::AreaKey(0,-1,0));
  assert(mr::area_key(.025,-.025,-.2617993877991494,.05,24)==mr::AreaKey(0,-1,23));
  assert(mr::project_area(left,true).size()==1);
  auto empty=mr::common_bounds({},24);assert(empty.count()==0);
  std::cout<<"Heading-aware intersection, status, dense indexing, holes, disconnected boundaries and angle wrapping passed\n";
}
