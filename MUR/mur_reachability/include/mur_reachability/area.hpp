#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <map>
#include <stdexcept>
#include <tuple>
#include <vector>
namespace mr {
using AreaKey=std::tuple<long long,long long,unsigned>;
using SparseArea=std::map<AreaKey,uint8_t>;
using Cell2=std::pair<long long,long long>;
using Projection=std::map<Cell2,uint8_t>;
struct Edge {Cell2 from,to;};
inline AreaKey area_key(double x,double y,double yaw,double resolution,unsigned bins) {
  if(!std::isfinite(x)||!std::isfinite(y)||!std::isfinite(yaw)||resolution<=0||bins==0)
    throw std::runtime_error("Invalid area pose");
  constexpr double tau=6.2831853071795864769;
  double angle=std::fmod(yaw,tau);if(angle<0)angle+=tau;
  unsigned k=static_cast<unsigned>(std::llround(angle*bins/tau))%bins;
  // Input poses are exact cell centres. Rounding protects floating point boundary noise.
  return {std::llround(x/resolution-.5),std::llround(y/resolution-.5),k};
}
inline void insert_area(SparseArea& a,const AreaKey& key,uint8_t state) {
  if(state!=1&&state!=2)throw std::runtime_error("Invalid area status");
  a[key]=std::max(a[key],state);
}
inline SparseArea intersect_areas(const SparseArea& a,const SparseArea& b) {
  SparseArea result;
  for(const auto& entry:a) {
    auto other=b.find(entry.first);
    if(other!=b.end())result[entry.first]=(entry.second==2&&other->second==2)?2:1;
  }
  return result;
}
inline Projection project_area(const SparseArea& a,bool verified_only) {
  Projection result;
  for(const auto& entry:a) {
    if(verified_only&&entry.second!=2)continue;
    Cell2 p{std::get<0>(entry.first),std::get<1>(entry.first)};
    result[p]=std::max(result[p],entry.second);
  }
  return result;
}
inline std::vector<Edge> boundary(const Projection& cells) {
  std::vector<Edge> edges;
  for(const auto& entry:cells) {
    auto x=entry.first.first,y=entry.first.second;
    if(!cells.count({x,y-1}))edges.push_back({{x,y},{x+1,y}});
    if(!cells.count({x+1,y}))edges.push_back({{x+1,y},{x+1,y+1}});
    if(!cells.count({x,y+1}))edges.push_back({{x+1,y+1},{x,y+1}});
    if(!cells.count({x-1,y}))edges.push_back({{x,y+1},{x,y}});
  }
  return edges;
}
struct AreaBounds {
  long long x=0,y=0;unsigned width=0,height=0,bins=0;
  size_t count()const{return static_cast<size_t>(width)*height*bins;}
  size_t index(const AreaKey& key)const {
    auto dx=std::get<0>(key)-x,dy=std::get<1>(key)-y;auto k=std::get<2>(key);
    if(dx<0||dy<0||static_cast<unsigned long long>(dx)>=width||static_cast<unsigned long long>(dy)>=height||k>=bins)
      throw std::runtime_error("Area index outside bounds");
    return static_cast<size_t>(dx)+width*(static_cast<size_t>(dy)+static_cast<size_t>(height)*k);
  }
};
inline AreaBounds common_bounds(const std::array<SparseArea,3>& areas,unsigned bins) {
  AreaBounds b;b.bins=bins;bool first=true;long long xmax=0,ymax=0;
  for(const auto& area:areas)for(const auto& entry:area) {
    long long x=std::get<0>(entry.first),y=std::get<1>(entry.first);
    if(first){b.x=x;b.y=y;xmax=x;ymax=y;first=false;}
    b.x=std::min(b.x,x);b.y=std::min(b.y,y);xmax=std::max(xmax,x);ymax=std::max(ymax,y);
  }
  if(first)return b;
  auto w=xmax-b.x+1,h=ymax-b.y+1;
  if(w<=0||h<=0||w>10000||h>10000||bins==0||static_cast<double>(w)*h*bins>20000000)
    throw std::runtime_error("Area grid exceeds supported size");
  b.width=static_cast<unsigned>(w);b.height=static_cast<unsigned>(h);return b;
}
inline std::vector<uint8_t> dense_area(const SparseArea& a,const AreaBounds& bounds) {
  std::vector<uint8_t> result(bounds.count(),0);
  for(const auto& entry:a)result[bounds.index(entry.first)]=entry.second;
  return result;
}
} // namespace mr
