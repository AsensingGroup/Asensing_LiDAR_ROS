#pragma once

#include "utility/yaml_reader.hpp"
#include "source/source.hpp"

namespace asensing
{
namespace lidar
{

class NodeManager
{
public:

  void init(const YAML::Node& config);
  void start();
  void stop();

  ~NodeManager();
  NodeManager() = default;

  std::vector<Source::Ptr> GetSources() const {return sources_;}

  int RosbagNum = 0;

private:

  std::vector<Source::Ptr> sources_;
};

}  // namespace lidar
}  // namespace asensing

