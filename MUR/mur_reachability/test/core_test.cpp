#include "mur_reachability/core.hpp"
#include <cassert>
#include <cstdio>
#include <iostream>
#include <random>
#include <filesystem>
using namespace std::chrono_literals;
int main() {
  const double h=std::sqrt(.5);
  mr::Sample s;s.p={.5,.2,.4};s.q={0,0,0,1};s.joints={.1,.2,.3,.4,.5,.6};
  mr::Transform mount{{.1,.2,.312},{0,0,0,1}};
  mr::Transform goal{{.625,2.625,.812},{0,0,h,h}};
  mr::SearchOptions opts;opts.yaw_bins=4;opts.angle_tolerance=.01;opts.height_tolerance=.01;
  auto deadline=[](){return std::chrono::steady_clock::now()+5s;};
  auto r=mr::search({s},mount,goal,.1,opts,deadline());
  assert(r.candidates.size()==1);auto c=r.candidates.front();
  assert(std::abs(c.x-1.025)<1e-10&&std::abs(c.y-2.025)<1e-10);
  assert(std::abs(c.yaw-1.5707963267948966)<1e-10);
  // Tilted mounting frame: expected world coordinates computed independently.
  mount.q={h,0,0,h};goal.p={1.225,2.625,.612};goal.q={.5,.5,.5,.5};
  r=mr::search({s},mount,goal,.1,opts,deadline());assert(r.candidates.size()==1);
  assert(std::abs(r.candidates.front().x-1.025)<1e-10);
  assert(std::abs(r.candidates.front().y-2.025)<1e-10);
  for(auto& q:goal.q)q=-q;
  assert(mr::search({s},mount,goal,.1,opts,deadline()).candidates.size()==1);
  goal.p[2]+=1;assert(mr::search({s},mount,goal,.1,opts,deadline()).candidates.empty());
  auto expired=mr::search({s},mount,goal,.1,opts,std::chrono::steady_clock::now()-1s);assert(expired.truncated);
  mr::Cache cache;cache.metadata="format test";cache.dofs={6,6};cache.samples[0]={s};cache.samples[1]={s,s};
  const auto path=(std::filesystem::temp_directory_path()/("mur_reachability_test_"+std::to_string(std::chrono::steady_clock::now().time_since_epoch().count())+".bin")).string();
  mr::save(cache,path);auto read=mr::load(path);assert(read.metadata==cache.metadata&&read.samples[1].size()==2);
  assert(read.samples[0][0].joints==s.joints&&read.samples[0][0].q==s.q);
  {std::fstream f(path,std::ios::in|std::ios::out|std::ios::binary);f.seekp(18);f.put('x');}
  bool caught=false;try{mr::load(path);}catch(const std::exception&){caught=true;}assert(caught);std::remove(path.c_str());
  std::cout<<"Transforms, planar inversion, quaternion sign, height rejection, deadline and cache-integrity tests passed\n";
  // Synthetic lookup timing only; not a robot/ROS/IK benchmark.
  std::mt19937 gen(7);std::uniform_real_distribution<double> u(-1,1);std::vector<mr::Sample> samples;samples.reserve(300000);
  for(int i=0;i<300000;++i){mr::Sample t;t.p={u(gen),u(gen),u(gen)};t.q=mr::normalize({u(gen),u(gen),u(gen),u(gen)});t.joints=s.joints;samples.push_back(std::move(t));}
  opts=mr::SearchOptions{};goal={{1,2,.8},{0,0,0,1}};mount={{0,0,.3},{0,0,0,1}};
  auto start=std::chrono::steady_clock::now();r=mr::search(samples,mount,goal,0,opts,deadline());
  auto elapsed=std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count();
  std::cout<<"Synthetic 300000-sample, 24-heading lookup: "<<elapsed<<" s, "<<r.candidates.size()<<" suggestions (no IK/ROS)\n";
}
