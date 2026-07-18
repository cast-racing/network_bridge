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

#include "network_bridge/network_bridge.hpp"

#include <zstd.h>
#include <span>
#include <fstream>
#include <bit>
#include <array>

#include <rclcpp/serialization.hpp>
#include <pluginlib/class_loader.hpp>
#include <std_msgs/msg/string.hpp>

#include "network_interfaces/network_interface_base.hpp"

namespace
{
constexpr char kServiceRequestPacket[] = "__network_bridge_service_request__";
constexpr char kServiceResponsePacket[] = "__network_bridge_service_response__";
constexpr char kSetFloat32Type[] = "iac_msgs/srv/SetFloat32";
constexpr char kSetStringType[] = "planning_msgs/srv/SetString";
constexpr char kGhostCarCommandType[] = "planning_msgs/srv/GhostCarCommand";
constexpr char kOpponentStatesCommandType[] = "planning_msgs/srv/TacticalStatesCommand";

template<typename T>
void append_pod(std::vector<uint8_t> & buffer, const T & value)
{
  const auto bytes = std::bit_cast<std::array<uint8_t, sizeof(T)>>(value);
  buffer.insert(buffer.end(), bytes.begin(), bytes.end());
}

template<typename T>
bool read_pod(std::span<const uint8_t> payload, size_t & offset, T & value)
{
  if (offset + sizeof(T) > payload.size()) {
    return false;
  }
  std::array<uint8_t, sizeof(T)> bytes{};
  std::copy(
    payload.begin() + static_cast<std::ptrdiff_t>(offset),
    payload.begin() + static_cast<std::ptrdiff_t>(offset + sizeof(T)),
    bytes.begin());
  value = std::bit_cast<T>(bytes);
  offset += sizeof(T);
  return true;
}

void append_string(std::vector<uint8_t> & buffer, const std::string & value)
{
  const uint32_t size = static_cast<uint32_t>(value.size());
  append_pod(buffer, size);
  buffer.insert(buffer.end(), value.begin(), value.end());
}

bool read_string(std::span<const uint8_t> payload, size_t & offset, std::string & value)
{
  uint32_t size = 0;
  if (!read_pod(payload, offset, size)) {
    return false;
  }
  if (offset + size > payload.size()) {
    return false;
  }
  value.assign(
    reinterpret_cast<const char *>(payload.data() + offset),
    static_cast<size_t>(size));
  offset += size;
  return true;
}
template<typename MsgT>
std::vector<uint8_t> serialize_ros_msg(const MsgT & msg)
{
  rclcpp::Serialization<MsgT> serializer;
  rclcpp::SerializedMessage serialized;
  serializer.serialize_message(&msg, &serialized);
  const auto & rcl = serialized.get_rcl_serialized_message();
  return std::vector<uint8_t>(rcl.buffer, rcl.buffer + rcl.buffer_length);
}

template<typename MsgT>
bool deserialize_ros_msg(std::span<const uint8_t> data, MsgT & msg)
{
  rclcpp::Serialization<MsgT> serializer;
  rclcpp::SerializedMessage serialized(data.size());
  auto & rcl = serialized.get_rcl_serialized_message();
  std::copy(data.begin(), data.end(), rcl.buffer);
  rcl.buffer_length = data.size();
  try {
    serializer.deserialize_message(&serialized, &msg);
    return true;
  } catch (const std::exception &) {
    return false;
  }
}
}  // namespace

NetworkBridge::NetworkBridge(const std::string & node_name)
: Node(node_name),
  loader_("network_bridge", "network_bridge::NetworkInterface") {}

void NetworkBridge::initialize()
{
  load_parameters();
  load_service_parameters();
  load_network_interface();
  network_interface_->open();
}

void NetworkBridge::load_parameters()
{
  this->declare_parameter(
    "network_interface",
    std::string("network_bridge::UdpInterface"));
  this->get_parameter("network_interface", network_interface_name_);

  bool publish_stale_data;
  this->declare_parameter("publish_stale_data", false);
  this->get_parameter("publish_stale_data", publish_stale_data);
  // Defaults
  this->declare_parameter("default_rate", 5.0);
  this->declare_parameter("default_zstd_level", 3);

  float default_rate;
  int default_zstd_level;
  this->get_parameter("default_rate", default_rate);
  this->get_parameter("default_zstd_level", default_zstd_level);

  this->declare_parameter("publish_namespace", "");
  this->get_parameter("publish_namespace", publish_namespace_);

  if (!publish_namespace_.empty()) {
    if (publish_namespace_.front() != '/') {
      publish_namespace_.insert(0, "/");
    }
    if (publish_namespace_.back() == '/') {
      publish_namespace_.pop_back();
    }
    RCLCPP_INFO(
      this->get_logger(),
      "Topics will be published under the namespace %s",
      publish_namespace_.c_str());
  }

  std::string subscribe_namespace;
  this->declare_parameter("subscribe_namespace", "");
  this->get_parameter("subscribe_namespace", subscribe_namespace);

  if (!subscribe_namespace.empty()) {
    if (subscribe_namespace.front() != '/') {
      subscribe_namespace.insert(0, "/");
    }
    if (subscribe_namespace.back() == '/') {
      subscribe_namespace.pop_back();
    }
    RCLCPP_INFO(
      this->get_logger(),
      "Topics will be subscribed to under the namespace %s",
      subscribe_namespace.c_str());
  }

  // Load topics information
  this->declare_parameter<std::vector<std::string>>(
    "topics",
    std::vector<std::string>{});

  std::vector<std::string> topics;
  this->get_parameter("topics", topics);

  for (const auto & topic : topics) {
    std::string rate_param_name = topic + ".rate";
    std::string zstd_level_param_name = topic + ".zstd_level";

    this->declare_parameter<double>(rate_param_name, default_rate);
    this->declare_parameter<int>(zstd_level_param_name, default_zstd_level);

    float rate;
    int zstd_level;

    this->get_parameter(rate_param_name, rate);
    this->get_parameter(zstd_level_param_name, zstd_level);

    auto manager = std::make_shared<SubscriptionManager>(
      shared_from_this(), topic, subscribe_namespace,
      zstd_level, publish_stale_data);
    sub_mgrs_.push_back(manager);

    int ms = static_cast<int>(1000.0 / rate);
    auto timer = this->create_wall_timer(
      std::chrono::milliseconds(ms),
      [this, manager]() {
        send_data(manager);
      });

    timers_.push_back(timer);

    RCLCPP_INFO(
      this->get_logger(),
      "Topic: %s, Rate: %f Hz", topic.c_str(), rate);
  }
}

void NetworkBridge::load_network_interface()
{
  try {
    network_interface_ = loader_.createSharedInstance(network_interface_name_);

    network_interface_->initialize(
      shared_from_this(),
      std::bind(
        &NetworkBridge::receive_data,
        this,
        std::placeholders::_1));

    RCLCPP_INFO(
      this->get_logger(),
      "Loaded network interface: %s", network_interface_name_.c_str());
  } catch (const pluginlib::PluginlibException & ex) {
    RCLCPP_FATAL(
      this->get_logger(),
      "Failed to load network interface: %s", ex.what());
    rclcpp::shutdown();
    exit(1);
  }
}

void NetworkBridge::receive_data(std::span<const uint8_t> data)
{
  auto now = std::chrono::system_clock::now();

  // Decompress data
  std::vector<uint8_t> decompressed_data;
  try {
    decompress(data, decompressed_data);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Decompression Failed: %s", e.what());
  }

  std::string topic;
  std::string type;
  double current_time;
  parse_header(decompressed_data, topic, type, current_time);

  if (topic.empty() || type.empty()) {
    RCLCPP_ERROR(this->get_logger(), "Malformed header!");
    return;
  }

  int header_length = sizeof(current_time) + topic.size() + type.size() + 2;

  std::span<const uint8_t> payload(
    decompressed_data.begin() + header_length, decompressed_data.end());

  float delay = rclcpp::Clock().now().seconds() - current_time;
  RCLCPP_DEBUG(
    this->get_logger(),
    "Received %lu bytes on topic %s with type %s",
    data.size(), topic.c_str(), type.c_str());
  RCLCPP_DEBUG(
    this->get_logger(),
    "Decompressed data size: %lu", decompressed_data.size());
  RCLCPP_DEBUG(this->get_logger(), "Delay: %f ms", delay * 1000);

  if (topic == kServiceRequestPacket || topic == kServiceResponsePacket) {
    handle_service_packet(topic, payload);
    return;
  }

  if (publishers_.find(topic) == publishers_.end()) {
    // Create a QoS configuration with reliability and durability settings
    rclcpp::QoS qos(10);

    // Set QoS to Reliable
    qos.reliable();

    // Set QoS to Transient Local Durability
    qos.transient_local();
    publishers_[topic] = this->create_generic_publisher(
      publish_namespace_ + topic, type, qos);
  }

  rclcpp::SerializedMessage msg(payload.size());
  std::copy(
    payload.begin(), payload.end(),
    msg.get_rcl_serialized_message().buffer);

  msg.get_rcl_serialized_message().buffer_length = payload.size();
  publishers_[topic]->publish(msg);

  auto end = std::chrono::system_clock::now();
  RCLCPP_DEBUG(
    this->get_logger(),
    "Receive time: %f ms",
    std::chrono::duration<double, std::milli>(end - now).count());
}

void NetworkBridge::send_data(std::shared_ptr<SubscriptionManager> manager)
{
  const std::vector<uint8_t> & data = manager->get_data();

  if (data.empty()) {
    RCLCPP_DEBUG(
      this->get_logger(),
      "SubscriptionManager %s has no data", manager->topic_.c_str());
    return;
  }

  auto now = std::chrono::system_clock::now();
  const std::string & topic = manager->topic_;
  const std::string & type = manager->msg_type_;

  auto header = create_header(topic, type);

  // Form message
  std::vector<uint8_t> message;
  message.reserve(header.size() + data.size());
  message.insert(message.end(), header.begin(), header.end());
  message.insert(message.end(), data.begin(), data.end());

  // Compress data
  std::vector<uint8_t> compressed_data;
  try {
    compress(message, compressed_data, manager->zstd_compression_level_);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Compression Failed: %s", e.what());
    return;
  }

  // Send data
  {
    std::lock_guard<std::mutex> lock(network_write_mutex_);
    network_interface_->write(compressed_data);
  }
  auto end = std::chrono::system_clock::now();
  RCLCPP_DEBUG(
    this->get_logger(),
    "Send time: %f ms",
    std::chrono::duration<double, std::milli>(end - now).count());
}

void NetworkBridge::send_service_packet(
  const std::string & packet_kind, const std::vector<uint8_t> & payload)
{
  auto header = create_header(packet_kind, "network_bridge/ServicePacket");

  std::vector<uint8_t> message;
  message.reserve(header.size() + payload.size());
  message.insert(message.end(), header.begin(), header.end());
  message.insert(message.end(), payload.begin(), payload.end());

  std::vector<uint8_t> compressed_data;
  try {
    compress(message, compressed_data, 3);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Service packet compression failed: %s", e.what());
    return;
  }

  {
    std::lock_guard<std::mutex> lock(network_write_mutex_);
    network_interface_->write(compressed_data);
  }
}

void NetworkBridge::handle_service_packet(
  const std::string & packet_kind, std::span<const uint8_t> payload)
{
  size_t offset = 0;
  std::string service_type;
  if (!read_string(payload, offset, service_type)) {
    RCLCPP_ERROR(this->get_logger(), "Malformed service packet: cannot read type");
    return;
  }

  const bool is_request = (packet_kind == kServiceRequestPacket);

  if (service_type == kSetFloat32Type) {
    if (is_request) handle_set_float32_request(payload);
    else handle_set_float32_response(payload);
  } else if (service_type == kSetStringType) {
    if (is_request) handle_set_string_request(payload);
    else handle_new_service_response(payload);
  } else if (service_type == kGhostCarCommandType) {
    if (is_request) handle_ghost_car_command_request(payload);
    else handle_new_service_response(payload);
  } else if (service_type == kOpponentStatesCommandType) {
    if (is_request) handle_opponent_states_command_request(payload);
    else handle_new_service_response(payload);
  } else {
    RCLCPP_WARN(
      this->get_logger(),
      "Received service packet with unknown type: %s", service_type.c_str());
  }
}

void NetworkBridge::load_service_parameters()
{
  this->declare_parameter<std::vector<std::string>>(
    "service_servers", std::vector<std::string>{});
  this->declare_parameter<std::vector<std::string>>(
    "service_clients", std::vector<std::string>{});

  std::vector<std::string> service_servers;
  std::vector<std::string> service_clients;
  this->get_parameter("service_servers", service_servers);
  this->get_parameter("service_clients", service_clients);

  for (const auto & service_name : service_servers) {
    const std::string type_param = service_name + ".type";
    const std::string remote_param = service_name + ".remote_name";
    const std::string timeout_param = service_name + ".timeout_ms";

    this->declare_parameter<std::string>(type_param, "");
    this->declare_parameter<std::string>(remote_param, service_name);
    this->declare_parameter<int>(timeout_param, 1000);

    std::string service_type;
    std::string remote_name;
    int timeout_ms = 1000;
    this->get_parameter(type_param, service_type);
    this->get_parameter(remote_param, remote_name);
    this->get_parameter(timeout_param, timeout_ms);

    if (service_type == kSetFloat32Type) {
      setup_set_float32_server_bridge(service_name, remote_name, timeout_ms);
    } else if (service_type == kSetStringType) {
      setup_set_string_server_bridge(service_name, remote_name, timeout_ms);
    } else if (service_type == kGhostCarCommandType) {
      setup_ghost_car_command_server_bridge(service_name, remote_name, timeout_ms);
    } else if (service_type == kOpponentStatesCommandType) {
      setup_opponent_states_command_server_bridge(service_name, remote_name, timeout_ms);
    } else {
      RCLCPP_WARN(
        this->get_logger(),
        "Service bridge for %s uses unsupported type %s",
        service_name.c_str(), service_type.c_str());
      continue;
    }
  }

  for (const auto & service_name : service_clients) {
    const std::string type_param = service_name + ".type";
    const std::string remote_param = service_name + ".remote_name";

    this->declare_parameter<std::string>(type_param, "");
    this->declare_parameter<std::string>(remote_param, service_name);

    std::string service_type;
    std::string remote_name;
    this->get_parameter(type_param, service_type);
    this->get_parameter(remote_param, remote_name);

    if (service_type == kSetFloat32Type) {
      setup_set_float32_client_bridge(service_name, remote_name);
    } else if (service_type == kSetStringType) {
      setup_set_string_client_bridge(service_name, remote_name);
    } else if (service_type == kGhostCarCommandType) {
      setup_ghost_car_command_client_bridge(service_name, remote_name);
    } else if (service_type == kOpponentStatesCommandType) {
      setup_opponent_states_command_client_bridge(service_name, remote_name);
    } else {
      RCLCPP_WARN(
        this->get_logger(),
        "Service bridge for %s uses unsupported type %s",
        service_name.c_str(), service_type.c_str());
      continue;
    }
  }
}

void NetworkBridge::setup_set_float32_server_bridge(
  const std::string & service_name, const std::string & remote_name,
  int timeout_ms)
{
  SetFloat32ServerBridge bridge;
  bridge.service_name = service_name;
  bridge.remote_name = remote_name;
  bridge.timeout_ms = timeout_ms;

  bridge.server = this->create_service<iac_msgs::srv::SetFloat32>(
    service_name,
    [this, service_name](
      const std::shared_ptr<iac_msgs::srv::SetFloat32::Request> request,
      std::shared_ptr<iac_msgs::srv::SetFloat32::Response> response) {
      auto bridge_it = std::find_if(
        set_float32_server_bridges_.begin(),
        set_float32_server_bridges_.end(),
        [&service_name](const auto & bridge_item) {
          return bridge_item.service_name == service_name;
        });
      if (bridge_it == set_float32_server_bridges_.end()) {
        RCLCPP_ERROR(
          this->get_logger(),
          "No service bridge config found for %s", service_name.c_str());
        response->success = false;
        return;
      }

      bool success = false;
      response->success = call_remote_set_float32(*bridge_it, request->data, success) && success;
    });

  RCLCPP_INFO(
    this->get_logger(),
    "Service server bridge: %s -> %s (%s)",
    service_name.c_str(), remote_name.c_str(), kSetFloat32Type);

  set_float32_server_bridges_.push_back(bridge);
}

void NetworkBridge::setup_set_float32_client_bridge(
  const std::string & service_name, const std::string & remote_name)
{
  SetFloat32ClientBridge bridge;
  bridge.service_name = service_name;
  bridge.remote_name = remote_name;
  bridge.client = this->create_client<iac_msgs::srv::SetFloat32>(remote_name);

  RCLCPP_INFO(
    this->get_logger(),
    "Service client bridge: %s -> local %s (%s)",
    service_name.c_str(), remote_name.c_str(), kSetFloat32Type);

  set_float32_client_bridges_.push_back(bridge);
}

bool NetworkBridge::call_remote_set_float32(
  const SetFloat32ServerBridge & bridge, float data, bool & success)
{
  uint64_t request_id = 0;
  auto pending = std::make_shared<PendingSetFloat32Response>();
  {
    std::lock_guard<std::mutex> lock(pending_set_float32_mutex_);
    request_id = next_service_request_id_++;
    pending_set_float32_responses_[request_id] = pending;
  }

  std::vector<uint8_t> payload;
  append_string(payload, kSetFloat32Type);
  append_string(payload, bridge.remote_name);
  append_pod(payload, request_id);
  append_pod(payload, data);
  send_service_packet(kServiceRequestPacket, payload);

  std::unique_lock<std::mutex> lock(pending->mutex);
  const bool completed = pending->condition.wait_for(
    lock,
    std::chrono::milliseconds(bridge.timeout_ms),
    [&pending]() {return pending->completed;});

  {
    std::lock_guard<std::mutex> pending_lock(pending_set_float32_mutex_);
    pending_set_float32_responses_.erase(request_id);
  }

  if (!completed) {
    RCLCPP_WARN(
      this->get_logger(),
      "Timed out waiting for bridged service response from %s",
      bridge.remote_name.c_str());
    return false;
  }

  success = pending->success;
  return true;
}

void NetworkBridge::handle_set_float32_request(std::span<const uint8_t> payload)
{
  size_t offset = 0;
  std::string service_type;
  std::string service_name;
  uint64_t request_id = 0;
  float data = 0.0f;

  if (!read_string(payload, offset, service_type) ||
    !read_string(payload, offset, service_name) ||
    !read_pod(payload, offset, request_id) ||
    !read_pod(payload, offset, data))
  {
    RCLCPP_ERROR(this->get_logger(), "Malformed service request packet");
    return;
  }

  if (service_type != kSetFloat32Type) {
    RCLCPP_WARN(
      this->get_logger(),
      "Received unsupported service request type %s", service_type.c_str());
    return;
  }

  auto bridge_it = std::find_if(
    set_float32_client_bridges_.begin(),
    set_float32_client_bridges_.end(),
    [&service_name](const auto & bridge) {
      return bridge.service_name == service_name || bridge.remote_name == service_name;
    });

  if (bridge_it == set_float32_client_bridges_.end()) {
    RCLCPP_WARN(
      this->get_logger(),
      "No local client bridge configured for service %s", service_name.c_str());
    std::vector<uint8_t> response_payload;
    append_string(response_payload, kSetFloat32Type);
    append_string(response_payload, service_name);
    append_pod(response_payload, request_id);
    append_pod(response_payload, static_cast<uint8_t>(0));
    send_service_packet(kServiceResponsePacket, response_payload);
    return;
  }

  auto request = std::make_shared<iac_msgs::srv::SetFloat32::Request>();
  request->data = data;

  auto client = bridge_it->client;
  if (!client->service_is_ready()) {
    RCLCPP_WARN(
      this->get_logger(),
      "Local service %s is not ready", bridge_it->remote_name.c_str());
    std::vector<uint8_t> response_payload;
    append_string(response_payload, kSetFloat32Type);
    append_string(response_payload, service_name);
    append_pod(response_payload, request_id);
    append_pod(response_payload, static_cast<uint8_t>(0));
    send_service_packet(kServiceResponsePacket, response_payload);
    return;
  }

  client->async_send_request(
    request,
    [this, service_name, request_id](
      rclcpp::Client<iac_msgs::srv::SetFloat32>::SharedFuture future) {
      bool success = false;
      try {
        success = future.get()->success;
      } catch (const std::exception & e) {
        RCLCPP_ERROR(
          this->get_logger(),
          "Local bridged service call failed: %s", e.what());
      }

      std::vector<uint8_t> response_payload;
      append_string(response_payload, kSetFloat32Type);
      append_string(response_payload, service_name);
      append_pod(response_payload, request_id);
      append_pod(response_payload, static_cast<uint8_t>(success ? 1 : 0));
      send_service_packet(kServiceResponsePacket, response_payload);
    });
}

void NetworkBridge::handle_set_float32_response(std::span<const uint8_t> payload)
{
  size_t offset = 0;
  std::string service_type;
  std::string service_name;
  uint64_t request_id = 0;
  uint8_t success = 0;

  if (!read_string(payload, offset, service_type) ||
    !read_string(payload, offset, service_name) ||
    !read_pod(payload, offset, request_id) ||
    !read_pod(payload, offset, success))
  {
    RCLCPP_ERROR(this->get_logger(), "Malformed service response packet");
    return;
  }

  (void)service_name;

  if (service_type != kSetFloat32Type) {
    RCLCPP_WARN(
      this->get_logger(),
      "Received unsupported service response type %s", service_type.c_str());
    return;
  }

  std::shared_ptr<PendingSetFloat32Response> pending;
  {
    std::lock_guard<std::mutex> lock(pending_set_float32_mutex_);
    auto it = pending_set_float32_responses_.find(request_id);
    if (it == pending_set_float32_responses_.end()) {
      RCLCPP_WARN(
        this->get_logger(),
        "Received response for unknown service request id %lu", request_id);
      return;
    }
    pending = it->second;
  }

  {
    std::lock_guard<std::mutex> lock(pending->mutex);
    pending->success = success != 0;
    pending->completed = true;
  }
  pending->condition.notify_one();
}

std::vector<uint8_t> NetworkBridge::create_header(
  const std::string & topic,
  const std::string & msg_type)
{
  double current_time = rclcpp::Clock().now().seconds();
  auto current_time_bytes =
    std::bit_cast<std::array<uint8_t, sizeof(current_time)>>(current_time);

  int header_length =
    current_time_bytes.size() + topic.size() + 1 + msg_type.size() + 1;

  std::vector<uint8_t> header;
  header.reserve(header_length);

  header.insert(
    header.end(), current_time_bytes.begin(), current_time_bytes.end());

  header.insert(header.end(), topic.begin(), topic.end());
  header.push_back('\0');

  header.insert(header.end(), msg_type.begin(), msg_type.end());
  header.push_back('\0');
  return header;
}

void NetworkBridge::parse_header(
  const std::vector<uint8_t> & header,
  std::string & topic, std::string & msg_type,
  double & time)
{
  // Add 4 for minimum usable header size
  // (1 char for topic, 1 for msg_type, and 2 null terminators)
  if (header.size() < sizeof(time) + 4) {
    RCLCPP_ERROR(this->get_logger(), "Malformed header!");
    return;
  }

  time = std::bit_cast<double>(header.data());
  topic = reinterpret_cast<const char *>(header.data() + sizeof(time));
  msg_type = reinterpret_cast<const char *>(
    header.data() + sizeof(time) + topic.size() + 1);
}

void NetworkBridge::compress(
  std::vector<uint8_t> const & data,
  std::vector<uint8_t> & compressed_data,
  int zstd_compression_level)
{
  size_t compressedCapacity = ZSTD_compressBound(data.size());

  // Resize the output buffer to the capacity needed
  compressed_data.resize(compressedCapacity);

  // Compress the data
  size_t compressedSize = ZSTD_compress(
    compressed_data.data(), compressedCapacity, data.data(), data.size(),
    zstd_compression_level);

  // Check for errors
  if (ZSTD_isError(compressedSize)) {
    throw std::runtime_error(ZSTD_getErrorName(compressedSize));
  }

  // Resize compressed_data to actual compressed size
  compressed_data.resize(compressedSize);
}

void NetworkBridge::decompress(
  std::span<const uint8_t> compressed_data,
  std::vector<uint8_t> & data)
{
  // Find the size of the original uncompressed data
  size_t decompressed_size = ZSTD_getFrameContentSize(
    compressed_data.data(), compressed_data.size());

  // Check if the size is known and valid
  if (decompressed_size == ZSTD_CONTENTSIZE_ERROR) {
    throw std::runtime_error("Not compressed by Zstd");
  } else if (decompressed_size == ZSTD_CONTENTSIZE_UNKNOWN) {
    throw std::runtime_error("Original size unknown");
  }

  // Resize the output buffer to the size of the uncompressed data
  data.resize(decompressed_size);

  // Decompress the data
  size_t decompressed_result = ZSTD_decompress(
    data.data(), decompressed_size, compressed_data.data(),
    compressed_data.size());

  // Check for errors during decompression
  if (ZSTD_isError(decompressed_result)) {
    throw std::runtime_error(ZSTD_getErrorName(decompressed_result));
  }
}

// ─── Shared helpers ───────────────────────────────────────────────────────────

void NetworkBridge::send_failure_response(
  const std::string & type, const std::string & name, uint64_t request_id)
{
  std::vector<uint8_t> payload;
  append_string(payload, type);
  append_string(payload, name);
  append_pod(payload, request_id);
  append_pod(payload, static_cast<uint8_t>(0));
  send_service_packet(kServiceResponsePacket, payload);
}

void NetworkBridge::handle_new_service_response(std::span<const uint8_t> payload)
{
  size_t offset = 0;
  std::string service_type, service_name;
  uint64_t request_id = 0;
  uint8_t success = 0;

  if (!read_string(payload, offset, service_type) ||
    !read_string(payload, offset, service_name) ||
    !read_pod(payload, offset, request_id) ||
    !read_pod(payload, offset, success))
  {
    RCLCPP_ERROR(this->get_logger(), "Malformed service response packet");
    return;
  }
  (void)service_name;

  std::shared_ptr<PendingServiceResponse> pending;
  {
    std::lock_guard<std::mutex> lock(pending_service_mutex_);
    auto it = pending_service_responses_.find(request_id);
    if (it == pending_service_responses_.end()) {
      RCLCPP_WARN(
        this->get_logger(),
        "Received response for unknown service request id %lu", request_id);
      return;
    }
    pending = it->second;
  }

  {
    std::lock_guard<std::mutex> lock(pending->mutex);
    pending->success = success != 0;
    pending->completed = true;
  }
  pending->condition.notify_one();
}

// ─── Member function templates (ground-station call side + car receive side) ──

template<typename SrvT>
bool NetworkBridge::call_remote_service(
  const std::string & type_name,
  const std::string & remote_name,
  int timeout_ms,
  const typename SrvT::Request & request)
{
  auto serialized = serialize_ros_msg(request);

  uint64_t request_id = 0;
  auto pending = std::make_shared<PendingServiceResponse>();
  {
    std::lock_guard<std::mutex> lock(pending_service_mutex_);
    request_id = next_service_request_id_++;
    pending_service_responses_[request_id] = pending;
  }

  std::vector<uint8_t> packet;
  append_string(packet, type_name);
  append_string(packet, remote_name);
  append_pod(packet, request_id);
  append_pod(packet, static_cast<uint32_t>(serialized.size()));
  packet.insert(packet.end(), serialized.begin(), serialized.end());
  send_service_packet(kServiceRequestPacket, packet);

  std::unique_lock<std::mutex> lock(pending->mutex);
  const bool completed = pending->condition.wait_for(
    lock,
    std::chrono::milliseconds(timeout_ms),
    [&pending]() {return pending->completed;});

  {
    std::lock_guard<std::mutex> pending_lock(pending_service_mutex_);
    pending_service_responses_.erase(request_id);
  }

  if (!completed) {
    RCLCPP_WARN(
      this->get_logger(),
      "Timed out waiting for bridged service response from %s", remote_name.c_str());
    return false;
  }
  return pending->success;
}

template<typename SrvT, typename ClientBridgeT>
void NetworkBridge::do_handle_serialized_service_request(
  const std::string & type_name,
  std::span<const uint8_t> payload,
  std::vector<ClientBridgeT> & client_bridges)
{
  size_t offset = 0;
  std::string service_type, service_name;
  uint64_t request_id = 0;
  uint32_t serialized_size = 0;

  if (!read_string(payload, offset, service_type) ||
    !read_string(payload, offset, service_name) ||
    !read_pod(payload, offset, request_id) ||
    !read_pod(payload, offset, serialized_size) ||
    offset + serialized_size > payload.size())
  {
    RCLCPP_ERROR(this->get_logger(), "Malformed %s request packet", type_name.c_str());
    return;
  }

  std::span<const uint8_t> serialized_bytes(payload.data() + offset, serialized_size);
  typename SrvT::Request request;
  if (!deserialize_ros_msg(serialized_bytes, request)) {
    RCLCPP_ERROR(this->get_logger(), "Failed to deserialize %s request", type_name.c_str());
    send_failure_response(type_name, service_name, request_id);
    return;
  }

  auto bridge_it = std::find_if(
    client_bridges.begin(), client_bridges.end(),
    [&service_name](const auto & b) {
      return b.service_name == service_name || b.remote_name == service_name;
    });

  if (bridge_it == client_bridges.end()) {
    RCLCPP_WARN(
      this->get_logger(),
      "No local client bridge configured for service %s", service_name.c_str());
    send_failure_response(type_name, service_name, request_id);
    return;
  }

  if (!bridge_it->client->service_is_ready()) {
    RCLCPP_WARN(
      this->get_logger(),
      "Local service %s is not ready", bridge_it->remote_name.c_str());
    send_failure_response(type_name, service_name, request_id);
    return;
  }

  auto req = std::make_shared<typename SrvT::Request>(request);
  bridge_it->client->async_send_request(
    req,
    [this, type_name, service_name, request_id](
      typename rclcpp::Client<SrvT>::SharedFuture future) {
      bool success = false;
      try {
        success = future.get()->success;
      } catch (const std::exception & e) {
        RCLCPP_ERROR(this->get_logger(), "Bridged service call failed: %s", e.what());
      }
      std::vector<uint8_t> response_payload;
      append_string(response_payload, type_name);
      append_string(response_payload, service_name);
      append_pod(response_payload, request_id);
      append_pod(response_payload, static_cast<uint8_t>(success ? 1 : 0));
      send_service_packet(kServiceResponsePacket, response_payload);
    });
}

// ─── SetString ────────────────────────────────────────────────────────────────

void NetworkBridge::setup_set_string_server_bridge(
  const std::string & service_name, const std::string & remote_name, int timeout_ms)
{
  SetStringServerBridge bridge;
  bridge.service_name = service_name;
  bridge.remote_name = remote_name;
  bridge.timeout_ms = timeout_ms;

  bridge.server = this->create_service<planning_msgs::srv::SetString>(
    service_name,
    [this, service_name](
      const std::shared_ptr<planning_msgs::srv::SetString::Request> request,
      std::shared_ptr<planning_msgs::srv::SetString::Response> response) {
      auto it = std::find_if(
        set_string_server_bridges_.begin(), set_string_server_bridges_.end(),
        [&service_name](const auto & b) {return b.service_name == service_name;});
      if (it == set_string_server_bridges_.end()) {
        response->success = false;
        return;
      }
      bool success = false;
      response->success = call_remote_set_string(*it, *request, success) && success;
    });

  RCLCPP_INFO(
    this->get_logger(),
    "Service server bridge: %s -> %s (%s)",
    service_name.c_str(), remote_name.c_str(), kSetStringType);

  set_string_server_bridges_.push_back(bridge);
}

void NetworkBridge::setup_set_string_client_bridge(
  const std::string & service_name, const std::string & remote_name)
{
  SetStringClientBridge bridge;
  bridge.service_name = service_name;
  bridge.remote_name = remote_name;
  bridge.client = this->create_client<planning_msgs::srv::SetString>(remote_name);

  RCLCPP_INFO(
    this->get_logger(),
    "Service client bridge: %s -> local %s (%s)",
    service_name.c_str(), remote_name.c_str(), kSetStringType);

  set_string_client_bridges_.push_back(bridge);
}

bool NetworkBridge::call_remote_set_string(
  const SetStringServerBridge & bridge,
  const planning_msgs::srv::SetString::Request & request,
  bool & success)
{
  success = call_remote_service<planning_msgs::srv::SetString>(
    kSetStringType, bridge.remote_name, bridge.timeout_ms, request);
  return true;
}

void NetworkBridge::handle_set_string_request(std::span<const uint8_t> payload)
{
  do_handle_serialized_service_request<planning_msgs::srv::SetString>(
    kSetStringType, payload, set_string_client_bridges_);
}

// ─── GhostCarCommand ──────────────────────────────────────────────────────────

void NetworkBridge::setup_ghost_car_command_server_bridge(
  const std::string & service_name, const std::string & remote_name, int timeout_ms)
{
  GhostCarCommandServerBridge bridge;
  bridge.service_name = service_name;
  bridge.remote_name = remote_name;
  bridge.timeout_ms = timeout_ms;

  bridge.server = this->create_service<planning_msgs::srv::GhostCarCommand>(
    service_name,
    [this, service_name](
      const std::shared_ptr<planning_msgs::srv::GhostCarCommand::Request> request,
      std::shared_ptr<planning_msgs::srv::GhostCarCommand::Response> response) {
      auto it = std::find_if(
        ghost_car_command_server_bridges_.begin(), ghost_car_command_server_bridges_.end(),
        [&service_name](const auto & b) {return b.service_name == service_name;});
      if (it == ghost_car_command_server_bridges_.end()) {
        response->success = false;
        return;
      }
      bool success = false;
      response->success = call_remote_ghost_car_command(*it, *request, success) && success;
    });

  RCLCPP_INFO(
    this->get_logger(),
    "Service server bridge: %s -> %s (%s)",
    service_name.c_str(), remote_name.c_str(), kGhostCarCommandType);

  ghost_car_command_server_bridges_.push_back(bridge);
}

void NetworkBridge::setup_ghost_car_command_client_bridge(
  const std::string & service_name, const std::string & remote_name)
{
  GhostCarCommandClientBridge bridge;
  bridge.service_name = service_name;
  bridge.remote_name = remote_name;
  bridge.client = this->create_client<planning_msgs::srv::GhostCarCommand>(remote_name);

  RCLCPP_INFO(
    this->get_logger(),
    "Service client bridge: %s -> local %s (%s)",
    service_name.c_str(), remote_name.c_str(), kGhostCarCommandType);

  ghost_car_command_client_bridges_.push_back(bridge);
}

bool NetworkBridge::call_remote_ghost_car_command(
  const GhostCarCommandServerBridge & bridge,
  const planning_msgs::srv::GhostCarCommand::Request & request,
  bool & success)
{
  success = call_remote_service<planning_msgs::srv::GhostCarCommand>(
    kGhostCarCommandType, bridge.remote_name, bridge.timeout_ms, request);
  return true;
}

void NetworkBridge::handle_ghost_car_command_request(std::span<const uint8_t> payload)
{
  do_handle_serialized_service_request<planning_msgs::srv::GhostCarCommand>(
    kGhostCarCommandType, payload, ghost_car_command_client_bridges_);
}

// ─── OpponentStatesCommand ────────────────────────────────────────────────────

void NetworkBridge::setup_opponent_states_command_server_bridge(
  const std::string & service_name, const std::string & remote_name, int timeout_ms)
{
  OpponentStatesCommandServerBridge bridge;
  bridge.service_name = service_name;
  bridge.remote_name = remote_name;
  bridge.timeout_ms = timeout_ms;

  bridge.server = this->create_service<planning_msgs::srv::TacticalStatesCommand>(
    service_name,
    [this, service_name](
      const std::shared_ptr<planning_msgs::srv::TacticalStatesCommand::Request> request,
      std::shared_ptr<planning_msgs::srv::TacticalStatesCommand::Response> response) {
      auto it = std::find_if(
        opponent_states_command_server_bridges_.begin(),
        opponent_states_command_server_bridges_.end(),
        [&service_name](const auto & b) {return b.service_name == service_name;});
      if (it == opponent_states_command_server_bridges_.end()) {
        response->success = false;
        return;
      }
      bool success = false;
      response->success =
        call_remote_opponent_states_command(*it, *request, success) && success;
    });

  RCLCPP_INFO(
    this->get_logger(),
    "Service server bridge: %s -> %s (%s)",
    service_name.c_str(), remote_name.c_str(), kOpponentStatesCommandType);

  opponent_states_command_server_bridges_.push_back(bridge);
}

void NetworkBridge::setup_opponent_states_command_client_bridge(
  const std::string & service_name, const std::string & remote_name)
{
  OpponentStatesCommandClientBridge bridge;
  bridge.service_name = service_name;
  bridge.remote_name = remote_name;
  bridge.client = this->create_client<planning_msgs::srv::TacticalStatesCommand>(remote_name);

  RCLCPP_INFO(
    this->get_logger(),
    "Service client bridge: %s -> local %s (%s)",
    service_name.c_str(), remote_name.c_str(), kOpponentStatesCommandType);

  opponent_states_command_client_bridges_.push_back(bridge);
}

bool NetworkBridge::call_remote_opponent_states_command(
  const OpponentStatesCommandServerBridge & bridge,
  const planning_msgs::srv::TacticalStatesCommand::Request & request,
  bool & success)
{
  success = call_remote_service<planning_msgs::srv::TacticalStatesCommand>(
    kOpponentStatesCommandType, bridge.remote_name, bridge.timeout_ms, request);
  return true;
}

void NetworkBridge::handle_opponent_states_command_request(std::span<const uint8_t> payload)
{
  do_handle_serialized_service_request<planning_msgs::srv::TacticalStatesCommand>(
    kOpponentStatesCommandType, payload, opponent_states_command_client_bridges_);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  // Randomized name to avoid conflicts
  std::srand(std::time(nullptr));
  std::string node_name = "network_bridge" + std::to_string(std::rand());
  auto node = std::make_shared<NetworkBridge>(node_name);

  node->initialize();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
