#pragma once
#include "mur_reachability/area.hpp"
namespace mr {
inline std::array<AreaKey,4> neighbours(const AreaKey& k) {
  auto x=std::get<0>(k),y=std::get<1>(k);auto h=std::get<2>(k);
  return {AreaKey{x-1,y,h},AreaKey{x+1,y,h},AreaKey{x,y-1,h},AreaKey{x,y+1,h}};
}
struct RefineBounds {
  long long xmin=0,xmax=-1,ymin=0,ymax=-1;
  bool contains(const AreaKey& k)const{return std::get<0>(k)>=xmin&&std::get<0>(k)<=xmax&&std::get<1>(k)>=ymin&&std::get<1>(k)<=ymax;}
};
} // namespace mr
