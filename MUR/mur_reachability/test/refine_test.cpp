#include "mur_reachability/refine.hpp"
#include <cassert>
#include <deque>
#include <set>
#include <iostream>
int main(){
  auto neighbours=mr::neighbours({-1,2,7});
  for(auto k:neighbours){assert(std::get<2>(k)==7);assert(std::abs(std::get<0>(k)+1)+std::abs(std::get<1>(k)-2)==1);}
  mr::RefineBounds bounds{-2,2,-2,2};
  assert(!bounds.contains({3,0,7}));
  // Synthetic IK oracle: a 5x5 region with a one-cell hole; no filling across rejection.
  std::deque<mr::AreaKey> queue{{-2,-2,7}};std::set<mr::AreaKey> seen,accepted;
  while(!queue.empty()){
    auto k=queue.front();queue.pop_front();if(!bounds.contains(k)||!seen.insert(k).second)continue;
    bool valid=std::get<0>(k)!=0||std::get<1>(k)!=0;
    if(!valid)continue;
    accepted.insert(k);for(auto next:mr::neighbours(k))queue.push_back(next);
  }
  assert(accepted.size()==24);assert(!accepted.count({0,0,7}));
  assert(!accepted.count({1,1,8}));
  mr::SparseArea area;for(auto k:accepted)mr::insert_area(area,k,2);
  assert(mr::boundary(mr::project_area(area,true)).size()==24); // 20 outer + 4 hole edges
  std::cout<<"Neighbour steps, heading preservation, search bounds and verified-only growth around a hole passed\n";
}
