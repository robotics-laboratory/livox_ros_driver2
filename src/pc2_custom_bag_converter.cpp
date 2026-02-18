#include <algorithm>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <memory>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include <builtin_interfaces/msg/time.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rcutils/allocator.h>
#include <rcutils/types/uint8_array.h>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <rosbag2_storage/topic_metadata.hpp>
#include <rosidl_runtime_cpp/traits.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

#include "include/livox_pc2_layout.h"
#include "lddc.h"

namespace {

using PointCloud2 = sensor_msgs::msg::PointCloud2;
using CustomMsg = livox_ros::CustomMsg;
using LivoxPC2Point = livox_ros::LivoxPC2Point;

struct Options {
  std::string input_uri;
  std::string output_uri;
  std::string storage_id = "mcap";
  std::string lidar_topic = "/livox/lidar";
};

const char kPc2Type[] = "sensor_msgs/msg/PointCloud2";

void PrintUsage() {
  std::cout
      << "Usage:\n"
      << "  ros2 run livox_ros_driver2 pc2_custom_bag_converter \\\n"
      << "    --input <bag_path> --output <bag_path> [options]\n\n"
      << "Options:\n"
      << "  --lidar-topic <name>   (default: /livox/lidar)\n"
      << "  --storage <id>         (default: mcap)\n";
}

Options ParseArgs(int argc, char **argv) {
  Options opts;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    auto require_value = [&](const std::string &name) {
      if (i + 1 >= argc) {
        throw std::runtime_error("Missing value for argument: " + name);
      }
      return std::string(argv[++i]);
    };

    if (arg == "--help" || arg == "-h") {
      PrintUsage();
      std::exit(0);
    } else if (arg == "--input") {
      opts.input_uri = require_value(arg);
    } else if (arg == "--output") {
      opts.output_uri = require_value(arg);
    } else if (arg == "--lidar-topic") {
      opts.lidar_topic = require_value(arg);
    } else if (arg == "--storage") {
      opts.storage_id = require_value(arg);
    } else {
      throw std::runtime_error("Unknown argument: " + arg);
    }
  }

  if (opts.input_uri.empty() || opts.output_uri.empty()) {
    throw std::runtime_error("--input and --output are required.");
  }
  if (opts.input_uri == opts.output_uri) {
    throw std::runtime_error("--input and --output must be different.");
  }
  return opts;
}

uint64_t TimeToNs(const builtin_interfaces::msg::Time &stamp) {
  return static_cast<uint64_t>(stamp.sec) * 1000000000ULL + static_cast<uint64_t>(stamp.nanosec);
}

bool IsCustomMsgType(const std::string &type_name) {
  return type_name == rosidl_generator_traits::name<CustomMsg>();
}

template <typename T>
T ReadScalar(const uint8_t *data) {
  T out{};
  std::memcpy(&out, data, sizeof(T));
  return out;
}

bool MatchNewLivoxPc2Layout(const PointCloud2 &cloud) {
  if (cloud.fields.size() != livox_ros::kLivoxPc2FieldCount) {
    return false;
  }
  for (size_t i = 0; i < livox_ros::kLivoxPc2FieldCount; ++i) {
    const auto &f = cloud.fields[i];
    const auto &exp = livox_ros::kLivoxPc2Fields[i];
    if (f.name != exp.name || f.offset != exp.offset || f.datatype != exp.datatype || f.count != exp.count) {
      return false;
    }
  }
  return true;
}

bool MatchOldLivoxPc2Layout(const PointCloud2 &cloud) {
  if (cloud.fields.size() != 7) {
    return false;
  }

  const auto &f0 = cloud.fields[0];
  const auto &f1 = cloud.fields[1];
  const auto &f2 = cloud.fields[2];
  const auto &f3 = cloud.fields[3];
  const auto &f4 = cloud.fields[4];
  const auto &f5 = cloud.fields[5];
  const auto &f6 = cloud.fields[6];

  return f0.name == "x" && f0.offset == 0 && f0.datatype == sensor_msgs::msg::PointField::FLOAT32 && f0.count == 1 &&
         f1.name == "y" && f1.offset == 4 && f1.datatype == sensor_msgs::msg::PointField::FLOAT32 && f1.count == 1 &&
         f2.name == "z" && f2.offset == 8 && f2.datatype == sensor_msgs::msg::PointField::FLOAT32 && f2.count == 1 &&
         f3.name == "intensity" && f3.offset == 12 && f3.datatype == sensor_msgs::msg::PointField::FLOAT32 && f3.count == 1 &&
         f4.name == "tag" && f4.offset == 16 && f4.datatype == sensor_msgs::msg::PointField::UINT8 && f4.count == 1 &&
         f5.name == "line" && f5.offset == 17 && f5.datatype == sensor_msgs::msg::PointField::UINT8 && f5.count == 1 &&
         f6.name == "timestamp" && f6.offset == 18 && f6.datatype == sensor_msgs::msg::PointField::FLOAT64 && f6.count == 1;
}

void ValidateNewLivoxPc2Layout(const PointCloud2 &cloud) {
  if (!MatchNewLivoxPc2Layout(cloud)) {
    throw std::runtime_error("Input PointCloud2 is not in new Livox layout");
  }
  if (cloud.point_step < sizeof(LivoxPC2Point)) {
    throw std::runtime_error("Input PointCloud2 point_step is too small for new layout");
  }
}

PointCloud2 ConvertOldLivoxPc2ToNewLivoxPc2(const PointCloud2 &input) {
  if (!MatchOldLivoxPc2Layout(input)) {
    throw std::runtime_error("Input PointCloud2 is not in old Livox layout");
  }
  if (input.point_step < 26) {
    throw std::runtime_error("Input PointCloud2 point_step is too small for old layout");
  }

  PointCloud2 out;
  out.header = input.header;
  out.height = input.height;
  out.width = input.width;
  out.is_bigendian = false;
  out.is_dense = input.is_dense;
  livox_ros::InitLivoxPc2Fields(out.fields);
  out.point_step = sizeof(LivoxPC2Point);
  out.row_step = out.point_step * out.width;
  out.data.resize(static_cast<size_t>(out.row_step) * static_cast<size_t>(std::max<uint32_t>(1U, out.height)));

  const uint64_t base_time_ns = TimeToNs(input.header.stamp);
  for (uint32_t row = 0; row < std::max<uint32_t>(1U, input.height); ++row) {
    const size_t in_row_start = static_cast<size_t>(row) * static_cast<size_t>(input.row_step);
    const size_t out_row_start = static_cast<size_t>(row) * static_cast<size_t>(out.row_step);
    for (uint32_t col = 0; col < input.width; ++col) {
      const size_t in_base = in_row_start + static_cast<size_t>(col) * static_cast<size_t>(input.point_step);
      const size_t out_base = out_row_start + static_cast<size_t>(col) * static_cast<size_t>(out.point_step);
      if (in_base + 26 > input.data.size() || out_base + sizeof(LivoxPC2Point) > out.data.size()) {
        throw std::runtime_error("PointCloud2 data buffer size mismatch");
      }

      const float x = ReadScalar<float>(&input.data[in_base + 0]);
      const float y = ReadScalar<float>(&input.data[in_base + 4]);
      const float z = ReadScalar<float>(&input.data[in_base + 8]);
      const float intensity_f = ReadScalar<float>(&input.data[in_base + 12]);
      const uint8_t tag = input.data[in_base + 16];
      const uint8_t line = input.data[in_base + 17];
      const double timestamp = ReadScalar<double>(&input.data[in_base + 18]);

      int64_t dt = static_cast<int64_t>(timestamp) - static_cast<int64_t>(base_time_ns);
      if (dt < 0) {
        dt = 0;
      }
      if (dt > static_cast<int64_t>(std::numeric_limits<uint32_t>::max())) {
        dt = static_cast<int64_t>(std::numeric_limits<uint32_t>::max());
      }

      const int intensity_i = std::max(0, std::min(255, static_cast<int>(intensity_f)));

      LivoxPC2Point p{};
      p.x = x;
      p.y = y;
      p.z = z;
      p.time = static_cast<uint32_t>(dt);
      p.intensity = static_cast<uint8_t>(intensity_i);
      p.tag = tag;
      p.line = line;
      std::memcpy(out.data.data() + out_base, &p, sizeof(LivoxPC2Point));
    }
  }

  return out;
}

PointCloud2 ConvertCustomMsgToPointCloud2(const CustomMsg &input) {
  PointCloud2 out;
  out.header = input.header;
  out.height = 1;
  out.width = static_cast<uint32_t>(input.points.size());
  out.is_bigendian = false;
  out.is_dense = true;
  livox_ros::InitLivoxPc2Fields(out.fields);
  out.point_step = sizeof(LivoxPC2Point);
  out.row_step = out.point_step * out.width;
  out.data.resize(static_cast<size_t>(out.row_step));

  auto *out_points = reinterpret_cast<LivoxPC2Point *>(out.data.data());
  for (size_t i = 0; i < input.points.size(); ++i) {
    out_points[i].x = input.points[i].x;
    out_points[i].y = input.points[i].y;
    out_points[i].z = input.points[i].z;
    out_points[i].time = input.points[i].offset_time;
    out_points[i].intensity = input.points[i].reflectivity;
    out_points[i].tag = input.points[i].tag;
    out_points[i].line = input.points[i].line;
  }

  return out;
}

CustomMsg ConvertPointCloud2ToCustomMsg(const PointCloud2 &input) {
  ValidateNewLivoxPc2Layout(input);

  CustomMsg out;
  out.header = input.header;
  out.timebase = TimeToNs(input.header.stamp);
  out.lidar_id = 0;
  out.rsvd = {0, 0, 0};

  const uint32_t height = std::max<uint32_t>(1U, input.height);
  const uint64_t total_points_u64 = static_cast<uint64_t>(height) * static_cast<uint64_t>(input.width);
  if (total_points_u64 > static_cast<uint64_t>(std::numeric_limits<uint32_t>::max())) {
    throw std::runtime_error("Too many points for CustomMsg");
  }
  const uint32_t total_points = static_cast<uint32_t>(total_points_u64);
  out.point_num = total_points;
  out.points.resize(total_points);

  uint32_t out_idx = 0;
  for (uint32_t row = 0; row < height; ++row) {
    const size_t row_start = static_cast<size_t>(row) * static_cast<size_t>(input.row_step);
    for (uint32_t col = 0; col < input.width; ++col) {
      const size_t base = row_start + static_cast<size_t>(col) * static_cast<size_t>(input.point_step);
      if (base + sizeof(LivoxPC2Point) > input.data.size()) {
        throw std::runtime_error("PointCloud2 data buffer size mismatch");
      }
      const auto *p = reinterpret_cast<const LivoxPC2Point *>(input.data.data() + base);
      out.points[out_idx].offset_time = p->time;
      out.points[out_idx].x = p->x;
      out.points[out_idx].y = p->y;
      out.points[out_idx].z = p->z;
      out.points[out_idx].reflectivity = p->intensity;
      out.points[out_idx].tag = p->tag;
      out.points[out_idx].line = p->line;
      ++out_idx;
    }
  }

  return out;
}

template <typename MsgT>
MsgT DeserializeMessage(const std::shared_ptr<rosbag2_storage::SerializedBagMessage> &bag_message) {
  MsgT out;
  rclcpp::Serialization<MsgT> serializer;
  rclcpp::SerializedMessage serialized(*bag_message->serialized_data);
  serializer.deserialize_message(&serialized, &out);
  return out;
}

template <typename MsgT>
std::shared_ptr<rosbag2_storage::SerializedBagMessage> SerializeMessage(
    const MsgT &message,
    const std::string &topic,
    rcutils_time_point_value_t recv_timestamp,
    rcutils_time_point_value_t send_timestamp) {
  rclcpp::Serialization<MsgT> serializer;
  rclcpp::SerializedMessage serialized;
  serializer.serialize_message(&message, &serialized);

  auto out = std::make_shared<rosbag2_storage::SerializedBagMessage>();
  out->topic_name = topic;
  out->recv_timestamp = recv_timestamp;
  out->send_timestamp = send_timestamp;

  auto buffer = std::shared_ptr<rcutils_uint8_array_t>(new rcutils_uint8_array_t,
      [](rcutils_uint8_array_t *ptr) {
        (void)rcutils_uint8_array_fini(ptr);
        delete ptr;
      });
  *buffer = rcutils_get_zero_initialized_uint8_array();

  const auto &src = serialized.get_rcl_serialized_message();
  auto allocator = rcutils_get_default_allocator();
  if (rcutils_uint8_array_init(buffer.get(), src.buffer_length, &allocator) != RCUTILS_RET_OK) {
    throw std::runtime_error("Failed to allocate serialized buffer");
  }

  std::memcpy(buffer->buffer, src.buffer, src.buffer_length);
  buffer->buffer_length = src.buffer_length;
  out->serialized_data = buffer;
  return out;
}

}  // namespace

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    const Options opts = ParseArgs(argc, argv);

    rosbag2_cpp::Reader reader;
    rosbag2_storage::StorageOptions in_storage;
    in_storage.uri = opts.input_uri;
    in_storage.storage_id = opts.storage_id;
    rosbag2_cpp::ConverterOptions converter_opts;
    converter_opts.input_serialization_format = "cdr";
    converter_opts.output_serialization_format = "cdr";
    reader.open(in_storage, converter_opts);

    rosbag2_cpp::Writer writer;
    rosbag2_storage::StorageOptions out_storage;
    out_storage.uri = opts.output_uri;
    out_storage.storage_id = opts.storage_id;
    writer.open(out_storage, converter_opts);

    const auto all_topics = reader.get_all_topics_and_types();
    std::string lidar_input_type;
    for (const auto &topic : all_topics) {
      if (topic.name == opts.lidar_topic) {
        lidar_input_type = topic.type;
        break;
      }
    }
    if (lidar_input_type.empty()) {
      throw std::runtime_error("Lidar topic not found in bag: " + opts.lidar_topic);
    }

    enum class ConvertMode {
      kCustomToPc2,
      kPc2ToCustom
    };

    ConvertMode mode;
    std::string lidar_output_type;
    if (IsCustomMsgType(lidar_input_type)) {
      mode = ConvertMode::kCustomToPc2;
      lidar_output_type = kPc2Type;
    } else if (lidar_input_type == kPc2Type) {
      mode = ConvertMode::kPc2ToCustom;
      lidar_output_type = rosidl_generator_traits::name<CustomMsg>();
    } else {
      throw std::runtime_error("Unsupported lidar topic type: " + lidar_input_type);
    }

    for (const auto &topic : all_topics) {
      rosbag2_storage::TopicMetadata meta = topic;
      if (meta.name == opts.lidar_topic) {
        meta.type = lidar_output_type;
      }
      writer.create_topic(meta);
    }

    uint64_t total_messages = 0;
    uint64_t converted = 0;
    while (reader.has_next()) {
      auto bag_message = reader.read_next();
      ++total_messages;

      if (bag_message->topic_name != opts.lidar_topic) {
        writer.write(bag_message);
        continue;
      }

      if (mode == ConvertMode::kCustomToPc2) {
        const auto in_custom = DeserializeMessage<CustomMsg>(bag_message);
        const auto out_pc2 = ConvertCustomMsgToPointCloud2(in_custom);
        auto out_msg = SerializeMessage(
            out_pc2, opts.lidar_topic, bag_message->recv_timestamp, bag_message->send_timestamp);
        writer.write(out_msg);
      } else {
        const auto in_pc2 = DeserializeMessage<PointCloud2>(bag_message);
        PointCloud2 normalized_pc2;
        if (MatchNewLivoxPc2Layout(in_pc2)) {
          normalized_pc2 = in_pc2;
        } else if (MatchOldLivoxPc2Layout(in_pc2)) {
          normalized_pc2 = ConvertOldLivoxPc2ToNewLivoxPc2(in_pc2);
        } else {
          throw std::runtime_error("Unsupported PointCloud2 field layout");
        }
        const auto out_custom = ConvertPointCloud2ToCustomMsg(normalized_pc2);
        auto out_msg = SerializeMessage(
            out_custom, opts.lidar_topic, bag_message->recv_timestamp, bag_message->send_timestamp);
        writer.write(out_msg);
      }
      ++converted;
    }

    std::cout << "Done. Total messages read: " << total_messages
              << ", converted " << opts.lidar_topic << " messages: " << converted
              << ", mode: " << (mode == ConvertMode::kCustomToPc2 ? "CustomMsg->PointCloud2" : "PointCloud2->CustomMsg")
              << std::endl;
  } catch (const std::exception &e) {
    std::cerr << "pc2_custom_bag_converter failed: " << e.what() << std::endl;
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
