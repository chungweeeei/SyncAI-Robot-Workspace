// Copyright (c) 2018 Intel Corporation
//
// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 2.1 of the License, or (at your option) any later version.
//
// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// Lesser General Public License for more details.

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "syncai_amcl/amcl_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<syncai_amcl::AmclNode>();
  // Second-phase init: safe to use shared_from_this() now that the node is
  // owned by a shared_ptr.
  node->configure();

  // MultiThreadedExecutor + the node's MutuallyExclusive callback group keeps
  // the TF buffer timeout timer serviceable on a separate thread, avoiding a
  // deadlock when a data callback blocks on a TF lookup.
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();

  return 0;
}
