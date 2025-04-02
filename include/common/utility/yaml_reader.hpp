/******************************************************************************
 * Copyright 2020 RoboSense All rights reserved.
 * Suteng Innovation Technology Co., Ltd. www.robosense.ai

 * This software is provided to you directly by RoboSense and might
 * only be used to access RoboSense LiDAR. Any compilation,
 * modification, exploration, reproduction and redistribution are
 * restricted without RoboSense's prior consent.

 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESSED OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL ROBOSENSE BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************/

#pragma once
#include <yaml-cpp/yaml.h>
// #include "utility/common.h"

template <typename T>
inline bool yamlRead(const YAML::Node& yaml, const std::string& key, T& out_val)
{
  if (!yaml[key] || yaml[key].Type() == YAML::NodeType::Null)
  {
    // EOL_LOG::EOL_LOG_WARN(" : Not set " + key + " value !!!");
    return false;
  }
  else
  {
    out_val = yaml[key].as<T>();
    return true;
  }
}

template <typename T>
inline void yamlReadAbort(const YAML::Node& yaml, const std::string& key, T& out_val)
{
  if (!yaml[key] || yaml[key].Type() == YAML::NodeType::Null)
  {
    // EOL_LOG::EOL_LOG_ERROR("  : Not set " + key + " value, Aborting!!!");
    exit(-1);
  }
  else
  {
    out_val = yaml[key].as<T>();
  }
}

/**
 * @brief  read target yaml node
 * @note   read the yaml node with the specific key value, if failed, will set the default value as the output value
 * @param  &yaml: the yaml node
 * @param  &key: the key
 * @param  &out_val: output value
 * @retval true: success false: failed
 */
template <typename T>
inline bool yamlRead(const YAML::Node& yaml, const std::string& key, T& out_val, const T& default_val)
{
  if (!yaml[key] || yaml[key].Type() == YAML::NodeType::Null)
  {
    out_val = default_val;
    return false;
  }
  else
  {
    out_val = yaml[key].as<T>();
    return true;
  }
}

/**
 * @brief  get the subnode
 * @note   get the subnode of the input yaml node with the subnode name , if failed, the progress will end
 * @param  &yaml: the yaml node
 * @param  &node: the name of the subnode
 * @retval the subnode
 */
inline YAML::Node yamlSubNodeAbort(const YAML::Node& yaml, const std::string& node)
{
  YAML::Node ret = yaml[node.c_str()];
  if (!ret)
  {
    // EOL_LOG::EOL_LOG_ERROR("  : Cannot find subnode " + node + " . Aborting!!!");
    exit(-1);
  }
  return ret;
}
/**
 * @brief  get the subnode
 * @note   get the subnode of the input yaml node with the subnode id (only use in subnode with list structure), if
 * failed, the progress will end
 * @param  &yaml: the yaml node
 * @param  &id: the id of the subnode
 * @retval the subnode
 */
inline YAML::Node yamlSubNodeAbort(const YAML::Node& yaml, const unsigned int& id)
{
  if (id >= yaml.size())
  {
    // EOL_LOG::EOL_LOG_ERROR("  : Input id is over range! Aborting!!!");

    exit(-1);
  }
  YAML::Node ret = yaml[id];
  return ret;
}
inline YAML::Node yamlSubNode(const YAML::Node& yaml, const std::string& node)
{
  YAML::Node ret = yaml[node.c_str()];
  if (!ret)
  {
    // EOL_LOG::EOL_LOG_WARN(" : Cannot find subnode " + node + ". Returning Null!");
    return YAML::Node();
  }
  return ret;
}
inline YAML::Node yamlSubNode(const YAML::Node& yaml, const unsigned int& id)
{
  if (id >= yaml.size())
  {
    // EOL_LOG::EOL_LOG_WARN(" : Input id is over range! Returning Null!");
    return YAML::Node();
  }
  YAML::Node ret = yaml[id];
  return ret;
}

inline bool yamlSubNode(const YAML::Node &node, const std::string &key, YAML::Node &ret) {
  try {
    ret = node[key];
  } catch (std::exception &e) {
    return false;
  }
  return true;
}