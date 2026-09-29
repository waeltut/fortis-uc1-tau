#pragma once
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <limits>
#include <map>
#include <sstream>
#include <stdexcept>
#include <tuple>
#include <vector>
namespace mr {
using Vec=std::array<double,3>;
using Quat=std::array<double,4>; // x,y,z,w
inline Quat multiply(const Quat& a,const Quat& b) {
  return {a[3]*b[0]+a[0]*b[3]+a[1]*b[2]-a[2]*b[1],
          a[3]*b[1]-a[0]*b[2]+a[1]*b[3]+a[2]*b[0],
          a[3]*b[2]+a[0]*b[1]-a[1]*b[0]+a[2]*b[3],
          a[3]*b[3]-a[0]*b[0]-a[1]*b[1]-a[2]*b[2]};
}
inline Quat normalize(Quat q) {
  double n=0;for(auto v:q)n+=v*v;
  if(!std::isfinite(n)||n<1e-16)throw std::runtime_error("Invalid quaternion");
  for(auto& v:q) { v/=std::sqrt(n); }
  return q;
}
inline Vec rotate(const Quat& q,const Vec& p) {
  auto r=multiply(multiply(q,{p[0],p[1],p[2],0}),{-q[0],-q[1],-q[2],q[3]});
  return {r[0],r[1],r[2]};
}
struct Transform {
  Vec p{};Quat q{0,0,0,1};
  Vec apply(const Vec& a)const{auto b=rotate(q,a);for(unsigned i=0;i<3;++i)b[i]+=p[i];return b;}
};
struct Sample {Vec p{};Quat q{0,0,0,1};std::vector<double> joints;};
struct Cache {std::string metadata;std::array<unsigned,2> dofs{};std::array<std::vector<Sample>,2> samples;};
inline uint64_t fnv(const unsigned char* data,size_t size,uint64_t h=14695981039346656037ULL) {
  for(size_t i=0;i<size;++i){h^=data[i];h*=1099511628211ULL;}return h;
}
inline std::string fingerprint(const std::string& text) {
  std::ostringstream s;s<<std::hex<<fnv(reinterpret_cast<const unsigned char*>(text.data()),text.size());return s.str();
}
// Explicit little-endian integers/IEEE doubles; checksum covers metadata and records.
class Writer {
  std::ofstream out_;uint64_t sum_=14695981039346656037ULL;
public:
  explicit Writer(const std::string& path):out_(path,std::ios::binary){if(!out_)throw std::runtime_error("Cannot write "+path);}
  void raw(const void* p,size_t n,bool hash=true){out_.write(static_cast<const char*>(p),n);if(hash)sum_=fnv(static_cast<const unsigned char*>(p),n,sum_);}
  void integer(uint64_t v,bool hash=true){unsigned char a[8];for(unsigned i=0;i<8;++i)a[i]=(v>>(8*i))&255;raw(a,8,hash);}
  void number(double d){static_assert(sizeof(double)==8);uint64_t v;std::memcpy(&v,&d,8);integer(v);}
  void finish(){integer(sum_,false);out_.flush();if(!out_)throw std::runtime_error("Cache write failed");}
};
class Reader {
  std::ifstream in_;uint64_t sum_=14695981039346656037ULL;
public:
  explicit Reader(const std::string& path):in_(path,std::ios::binary){if(!in_)throw std::runtime_error("Cannot read "+path);}
  void raw(void* p,size_t n,bool hash=true){in_.read(static_cast<char*>(p),n);if(!in_)throw std::runtime_error("Truncated cache");if(hash)sum_=fnv(static_cast<unsigned char*>(p),n,sum_);}
  uint64_t integer(bool hash=true){unsigned char a[8];raw(a,8,hash);uint64_t v=0;for(unsigned i=0;i<8;++i)v|=uint64_t(a[i])<<(8*i);return v;}
  double number(){auto v=integer();double d;std::memcpy(&d,&v,8);if(!std::isfinite(d))throw std::runtime_error("Nonfinite cache value");return d;}
  void finish(){auto expected=sum_;if(integer(false)!=expected)throw std::runtime_error("Cache checksum mismatch");if(in_.peek()!=std::char_traits<char>::eof())throw std::runtime_error("Trailing cache data");}
};
inline void save(const Cache& cache,const std::string& path) {
  Writer w(path);w.raw("MURRCH01",8);w.integer(cache.metadata.size());w.raw(cache.metadata.data(),cache.metadata.size());
  for(unsigned a=0;a<2;++a){w.integer(cache.dofs[a]);w.integer(cache.samples[a].size());for(const auto& s:cache.samples[a]){
    if(s.joints.size()!=cache.dofs[a])throw std::runtime_error("Invalid sample dimensions");
    for(auto v:s.p) { w.number(v); }
    for(auto v:s.q) { w.number(v); }
    for(auto v:s.joints) { w.number(v); }
  }}w.finish();
}
inline Cache load(const std::string& path) {
  Reader r(path);char magic[8];r.raw(magic,8);if(std::memcmp(magic,"MURRCH01",8))throw std::runtime_error("Unsupported cache format");
  Cache c;auto length=r.integer();if(length>64000000)throw std::runtime_error("Cache metadata too large");
  c.metadata.resize(length);r.raw(c.metadata.data(),length);
  for(unsigned a=0;a<2;++a){auto dofs=r.integer(),count=r.integer();if(dofs<1||dofs>32||count>5000000)throw std::runtime_error("Invalid cache size");
    c.dofs[a]=static_cast<unsigned>(dofs);c.samples[a].reserve(count);
    for(uint64_t i=0;i<count;++i){Sample s;for(auto& v:s.p)v=r.number();for(auto& v:s.q)v=r.number();
      double n=0;for(auto v:s.q)n+=v*v;if(std::abs(n-1)>1e-6)throw std::runtime_error("Nonunit cached quaternion");
      s.joints.resize(dofs);for(auto& v:s.joints)v=r.number();c.samples[a].push_back(std::move(s));}
  }r.finish();return c;
}
struct Candidate {double x=0,y=0,yaw=0,score=0;std::vector<size_t> seeds;};
struct SearchOptions {double resolution=.05,height_tolerance=.04,angle_tolerance=.35;unsigned yaw_bins=24,max_candidates=2000,seeds_per_cell=3;};
struct SearchResult {std::vector<Candidate> candidates;bool truncated=false;};
inline SearchResult search(const std::vector<Sample>& samples,const Transform& base_from_reference,
                           const Transform& goal,double base_z,const SearchOptions& opt,
                           std::chrono::steady_clock::time_point deadline) {
  if(opt.resolution<=0||opt.height_tolerance<=0||opt.angle_tolerance<.001||opt.angle_tolerance>3.141592653589793||
     opt.yaw_bins<1||opt.max_candidates<1||opt.seeds_per_cell<1)throw std::runtime_error("Invalid lookup options");
  using Key=std::tuple<long long,long long,unsigned>;
  std::map<Key,Candidate> bins;
  std::vector<double> cs(opt.yaw_bins),sn(opt.yaw_bins),yaw(opt.yaw_bins);
  std::vector<Quat> rotations(opt.yaw_bins);
  for(unsigned b=0;b<opt.yaw_bins;++b){yaw[b]=6.2831853071795864769*b/opt.yaw_bins;cs[b]=std::cos(yaw[b]);sn[b]=std::sin(yaw[b]);rotations[b]={0,0,std::sin(yaw[b]/2),std::cos(yaw[b]/2)};}
  const double min_dot=std::cos(opt.angle_tolerance/2);
  SearchResult result;
  for(size_t i=0;i<samples.size();++i) {
    if(i%512==0 && std::chrono::steady_clock::now()>=deadline){result.truncated=true;break;}
    const auto& s=samples[i];const auto p=base_from_reference.apply(s.p);
    double dz=std::abs(base_z+p[2]-goal.p[2]);if(dz>opt.height_tolerance)continue;
    const auto q=multiply(base_from_reference.q,s.q);
    for(unsigned b=0;b<opt.yaw_bins;++b) {
      const auto world_q=multiply(rotations[b],q);double dot=0;for(unsigned k=0;k<4;++k)dot+=world_q[k]*goal.q[k];dot=std::abs(dot);
      if(dot<min_dot)continue;
      double bx=goal.p[0]-(cs[b]*p[0]-sn[b]*p[1]);double by=goal.p[1]-(sn[b]*p[0]+cs[b]*p[1]);
      auto ix=static_cast<long long>(std::floor(bx/opt.resolution)),iy=static_cast<long long>(std::floor(by/opt.resolution));
      double score=dz/opt.height_tolerance+(1-std::min(1.0,dot))/(1-min_dot);
      Key key{ix,iy,b};auto it=bins.find(key);
      if(it==bins.end()) {
        if(bins.size()>=static_cast<size_t>(opt.max_candidates)*20){result.truncated=true;continue;}
        Candidate c;c.x=(ix+.5)*opt.resolution;c.y=(iy+.5)*opt.resolution;c.yaw=yaw[b];c.score=score;c.seeds={i};bins.emplace(key,std::move(c));}
      else {
        auto& c=it->second;
        if(score<c.score){c.score=score;c.seeds.insert(c.seeds.begin(),i);if(c.seeds.size()>opt.seeds_per_cell)c.seeds.pop_back();}
        else if(c.seeds.size()<opt.seeds_per_cell)c.seeds.push_back(i);
      }
    }
  }
  for(auto& entry:bins)result.candidates.push_back(std::move(entry.second));
  std::sort(result.candidates.begin(),result.candidates.end(),[](const Candidate& a,const Candidate& b){return a.score<b.score;});
  if(result.candidates.size()>opt.max_candidates){result.candidates.resize(opt.max_candidates);result.truncated=true;}
  return result;
}
} // namespace mr
