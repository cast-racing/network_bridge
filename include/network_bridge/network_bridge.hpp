/*
==============================================================================
MIT License

Copyright (c) 2024 Ethan M Brown

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
==============================================================================
*/

#pragma once

#include <vector>
#include <string>
#include <memory>
#include <map>
#include <condition_variable>
#include <mutex>
#include <unordered_map>
#include <rclcpp/rclcpp.hpp>
#include <iac_msgs/srv/set_float32.hpp>
#include <iac_msgs/srv/set_string.hpp>
#include <iac_msgs/srv/ghost_car_command.hpp>
#include <iac_msgs/srv/opponent_states_command.hpp>

#include "network_bridge/subscription_manager.hpp"
#include "network_interfaces/network_interface_base.hpp"

/**
 * @class NetworkBridge
 * @brief A class that provides bridging of ROS2 topics over a network interface.
 *
 * The `NetworkBridge` class is derived from the `rclcpp::Node` class and provides functionality for sending and receiving telemetry data over network.
 * It handles the setup of a network interface, parsing and creating headers, compressing and decompressing data, and error handling.
 * The class also manages the ROS 2 subscriptions, timers, and publishers associated with the communication.
 */
class NetworkBridge : public rclcpp::Node
{
public:
  /**
   * @brief Constructs a NetworkBridge object.
   *
   * This constructor initializes a NetworkBridge object with the specified node name.
   *
   * @param node_name The name of the ROS 2 node.
   */
  explicit NetworkBridge(const std::string & node_name);

  /**
   * @brief Loads parameters, loads network interface, and opens the network interface.
   *
   * It should be called before any other functions are called on the object.
   */
  virtual void initialize();

protected:
  /**
   * @brief Loads default parameters and creates subsciption managers for each topic.
   */
  virtual void load_parameters();

  /**
   * @brief Loads the network interface as a dynamic plugin and initializes it.
   */
  virtual void load_network_interface();

  /**
   * @brief Callback function for handling received data.
   *
   * @param data The data to be received, represented as a span.
   */
  virtual void receive_data(std::span<const uint8_t> data);

  /**
   * @brief Sends data to the network interface.
   *
   * @param manager A shared pointer to the SubscriptionManager object.
   */
  virtual void send_data(std::shared_ptr<SubscriptionManager> manager);

  /**
   * @brief Sends a service bridge packet over the network interface.
   *
   * Service packets reuse the same compression/header path as topic messages,
   * but are routed internally by reserved bridge topics.
   */
  virtual void send_service_packet(
    const std::string & packet_kind, const std::vector<uint8_t> & payload);

  /**
   * @brief Handles a received service request or response packet.
   */
  virtual void handle_service_packet(
    const std::string & packet_kind, std::span<const uint8_t> payload);

  /**
   * @brief Creates a header for the message.
   *
   * @param topic The topic of the message.
   * @param msg_type The type of the message.
   * @param header The header to be created.
   */
  virtual std::vector<uint8_t> create_header(
    const std::string & topic, const std::string & msg_type);

  /**
   * @brief Parses the header of the message.
   *
   * @param header The header to be parsed.
   * @param topic The topic of the message.
   * @param msg_type The type of the message.
   * @param time The time the message was sent.
   */
  virtual void parse_header(
    const std::vector<uint8_t> & header, std::string & topic,
    std::string & msg_type, double & time);

  /**
   * Compresses the given data using Zstandard compression algorithm.
   *
   * @param data The input data to be compressed.
   * @param compressed_data [out] The vector to store the compressed data.
   * @param zstd_compression_level The compression level to be used (default: 3).
   */
  virtual void compress(
    std::vector<uint8_t> const & data, std::vector<uint8_t> & compressed_data,
    int zstd_compression_level = 3);

  /**
   * @brief Decompresses the given compressed data.
   *
   * This function takes a span of compressed data and decompresses it,
   * storing the result in the provided data vector
   *
   * @param compressed_data The compressed data to be decompressed.
   * @param data [out] The vector to store the decompressed data.
   */
  virtual void decompress(
    std::span<const uint8_t> compressed_data, std::vector<uint8_t> & data);

  /**
   * @brief A class template that provides a plugin loader for network interfaces.
   *
   * @tparam InterfaceT The interface type that the loaded plugins must implement.
   */
  pluginlib::ClassLoader<network_bridge::NetworkInterface> loader_;

  /**
   * @brief The name of the network interface plugin.
   */
  std::string network_interface_name_;

  /**
   * @brief A shared pointer to an instance of the `network_bridge::NetworkInterface` class.
   */
  std::shared_ptr<network_bridge::NetworkInterface> network_interface_;

  /**
   * @brief A vector of the SubscriptionManager's for each topic.
   *
   * These are stored to keep them from going out of scope, but are not used directly.
   */
  std::vector<std::shared_ptr<SubscriptionManager>> sub_mgrs_;

  /**
   * @brief A vector of timers for sending each received topic over network.
   *
   * These are stroed to keep them from going out of scope.
   *
   * @see rclcpp::TimerBase
   */
  std::vector<rclcpp::TimerBase::SharedPtr> timers_;

  /**
   * @brief A map that stores the publisher object against the topic name.
   */
  std::map<std::string, rclcpp::GenericPublisher::SharedPtr> publishers_;

  /**
   * @brief the namespace for the publishers.
   */
  std::string publish_namespace_;

  struct SetFloat32ServerBridge
  {
    std::string service_name;
    std::string remote_name;
    int timeout_ms;
    rclcpp::Service<iac_msgs::srv::SetFloat32>::SharedPtr server;
  };

  struct SetFloat32ClientBridge
  {
    std::string service_name;
    std::string remote_name;
    rclcpp::Client<iac_msgs::srv::SetFloat32>::SharedPtr client;
  };

  struct PendingSetFloat32Response
  {
    bool completed = false;
    bool success = false;
    std::mutex mutex;
    std::condition_variable condition;
  };

  // Shared pending response struct for all new service types
  struct PendingServiceResponse
  {
    bool completed = false;
    bool success = false;
    std::mutex mutex;
    std::condition_variable condition;
  };

  struct SetStringServerBridge
  {
    std::string service_name, remote_name;
    int timeout_ms;
    rclcpp::Service<iac_msgs::srv::SetString>::SharedPtr server;
  };
  struct SetStringClientBridge
  {
    std::string service_name, remote_name;
    rclcpp::Client<iac_msgs::srv::SetString>::SharedPtr client;
  };

  struct GhostCarCommandServerBridge
  {
    std::string service_name, remote_name;
    int timeout_ms;
    rclcpp::Service<iac_msgs::srv::GhostCarCommand>::SharedPtr server;
  };
  struct GhostCarCommandClientBridge
  {
    std::string service_name, remote_name;
    rclcpp::Client<iac_msgs::srv::GhostCarCommand>::SharedPtr client;
  };

  struct OpponentStatesCommandServerBridge
  {
    std::string service_name, remote_name;
    int timeout_ms;
    rclcpp::Service<iac_msgs::srv::OpponentStatesCommand>::SharedPtr server;
  };
  struct OpponentStatesCommandClientBridge
  {
    std::string service_name, remote_name;
    rclcpp::Client<iac_msgs::srv::OpponentStatesCommand>::SharedPtr client;
  };

  void load_service_parameters();
  void setup_set_float32_server_bridge(
    const std::string & service_name, const std::string & remote_name,
    int timeout_ms);
  void setup_set_float32_client_bridge(
    const std::string & service_name, const std::string & remote_name);
  bool call_remote_set_float32(
    const SetFloat32ServerBridge & bridge, float data, bool & success);
  void handle_set_float32_request(std::span<const uint8_t> payload);
  void handle_set_float32_response(std::span<const uint8_t> payload);

  void setup_set_string_server_bridge(
    const std::string & service_name, const std::string & remote_name, int timeout_ms);
  void setup_set_string_client_bridge(
    const std::string & service_name, const std::string & remote_name);
  bool call_remote_set_string(
    const SetStringServerBridge & bridge,
    const iac_msgs::srv::SetString::Request & request, bool & success);
  void handle_set_string_request(std::span<const uint8_t> payload);

  void setup_ghost_car_command_server_bridge(
    const std::string & service_name, const std::string & remote_name, int timeout_ms);
  void setup_ghost_car_command_client_bridge(
    const std::string & service_name, const std::string & remote_name);
  bool call_remote_ghost_car_command(
    const GhostCarCommandServerBridge & bridge,
    const iac_msgs::srv::GhostCarCommand::Request & request, bool & success);
  void handle_ghost_car_command_request(std::span<const uint8_t> payload);

  void setup_opponent_states_command_server_bridge(
    const std::string & service_name, const std::string & remote_name, int timeout_ms);
  void setup_opponent_states_command_client_bridge(
    const std::string & service_name, const std::string & remote_name);
  bool call_remote_opponent_states_command(
    const OpponentStatesCommandServerBridge & bridge,
    const iac_msgs::srv::OpponentStatesCommand::Request & request, bool & success);
  void handle_opponent_states_command_request(std::span<const uint8_t> payload);

  void handle_new_service_response(std::span<const uint8_t> payload);
  void send_failure_response(
    const std::string & type, const std::string & name, uint64_t request_id);

  template<typename SrvT>
  bool call_remote_service(
    const std::string & type_name,
    const std::string & remote_name,
    int timeout_ms,
    const typename SrvT::Request & request);

  template<typename SrvT, typename ClientBridgeT>
  void do_handle_serialized_service_request(
    const std::string & type_name,
    std::span<const uint8_t> payload,
    std::vector<ClientBridgeT> & client_bridges);

  std::vector<SetFloat32ServerBridge> set_float32_server_bridges_;
  std::vector<SetFloat32ClientBridge> set_float32_client_bridges_;
  std::unordered_map<uint64_t, std::shared_ptr<PendingSetFloat32Response>>
  pending_set_float32_responses_;
  std::mutex pending_set_float32_mutex_;

  std::vector<SetStringServerBridge> set_string_server_bridges_;
  std::vector<SetStringClientBridge> set_string_client_bridges_;

  std::vector<GhostCarCommandServerBridge> ghost_car_command_server_bridges_;
  std::vector<GhostCarCommandClientBridge> ghost_car_command_client_bridges_;

  std::vector<OpponentStatesCommandServerBridge> opponent_states_command_server_bridges_;
  std::vector<OpponentStatesCommandClientBridge> opponent_states_command_client_bridges_;

  std::unordered_map<uint64_t, std::shared_ptr<PendingServiceResponse>>
  pending_service_responses_;
  std::mutex pending_service_mutex_;

  std::mutex network_write_mutex_;
  uint64_t next_service_request_id_ = 1;
};
