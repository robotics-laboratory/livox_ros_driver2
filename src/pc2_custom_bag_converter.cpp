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
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

namespace {

using PointCloud2 = sensor_msgs::msg::PointCloud2;
using PointField = sensor_msgs::msg::PointField;

struct Options {
  std::string input_uri;
  std::string output_uri;
  std::string storage_id = "mcap";
  std::string lidar_topic = "/livox/lidar";
};

struct FieldSpec {
  std::string name;
  uint32_t offset;
  uint8_t datatype;
  uint32_t count;
};

struct __attribute__((packed)) OutPoint {
  float x;
  float y;
  float z;
  uint32_t time;
  uint8_t intensity;
  uint8_t tag;
  uint8_t line;
};

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

std::vector<FieldSpec> ExpectedOldFields() {
  return {
      {"x", 0, PointField::FLOAT32, 1},
      {"y", 4, PointField::FLOAT32, 1},
      {"z", 8, PointField::FLOAT32, 1},
      {"intensity", 12, PointField::FLOAT32, 1},
      {"tag", 16, PointField::UINT8, 1},
      {"line", 17, PointField::UINT8, 1},
      {"timestamp", 18, PointField::FLOAT64, 1},
  };
}

bool MatchFields(const std::vector<PointField> &fields, const std::vector<FieldSpec> &expected,
                 std::string &error) {
  if (fields.size() != expected.size()) {
    error = "field count mismatch";
    return false;
  }

  for (size_t i = 0; i < expected.size(); ++i) {
    const auto &got = fields[i];
    const auto &exp = expected[i];
    if (got.name != exp.name || got.offset != exp.offset || got.datatype != exp.datatype ||
        got.count != exp.count) {
      error = "field mismatch at index " + std::to_string(i);
      return false;
    }
  }
  return true;
}

template <typename T>
T ReadScalar(const uint8_t *data) {
  T out{};
  std::memcpy(&out, data, sizeof(T));
  return out;
}

PointCloud2 ConvertOldLidarToNewLidar(const PointCloud2 &input) {
  std::string error;
  if (!MatchFields(input.fields, ExpectedOldFields(), error)) {
    throw std::runtime_error("Input /livox/lidar layout not supported: " + error);
  }

  if (input.point_step < 26) {
    throw std::runtime_error("Input point_step is too small for old layout");
  }

  PointCloud2 out;
  out.header = input.header;
  out.height = input.height;
  out.width = input.width;
  out.is_bigendian = false;
  out.is_dense = input.is_dense;

  out.fields.resize(7);
  out.fields[0].name = "x";
  out.fields[0].offset = 0;
  out.fields[0].datatype = PointField::FLOAT32;
  out.fields[0].count = 1;
  out.fields[1].name = "y";
  out.fields[1].offset = 4;
  out.fields[1].datatype = PointField::FLOAT32;
  out.fields[1].count = 1;
  out.fields[2].name = "z";
  out.fields[2].offset = 8;
  out.fields[2].datatype = PointField::FLOAT32;
  out.fields[2].count = 1;
  out.fields[3].name = "time";
  out.fields[3].offset = 12;
  out.fields[3].datatype = PointField::UINT32;
  out.fields[3].count = 1;
  out.fields[4].name = "intensity";
  out.fields[4].offset = 16;
  out.fields[4].datatype = PointField::UINT8;
  out.fields[4].count = 1;
  out.fields[5].name = "tag";
  out.fields[5].offset = 17;
  out.fields[5].datatype = PointField::UINT8;
  out.fields[5].count = 1;
  out.fields[6].name = "line";
  out.fields[6].offset = 18;
  out.fields[6].datatype = PointField::UINT8;
  out.fields[6].count = 1;

  out.point_step = sizeof(OutPoint);
  out.row_step = out.point_step * out.width;
  out.data.resize(out.row_step * std::max<uint32_t>(1U, out.height));

  const auto base_time_ns = TimeToNs(input.header.stamp);
  for (uint32_t row = 0; row < std::max<uint32_t>(1U, input.height); ++row) {
    const size_t in_row_start = static_cast<size_t>(row) * static_cast<size_t>(input.row_step);
    const size_t out_row_start = static_cast<size_t>(row) * static_cast<size_t>(out.row_step);
    for (uint32_t col = 0; col < input.width; ++col) {
      const size_t in_base = in_row_start + static_cast<size_t>(col) * static_cast<size_t>(input.point_step);
      const size_t out_base = out_row_start + static_cast<size_t>(col) * static_cast<size_t>(out.point_step);

      if (in_base + 26 > input.data.size() || out_base + sizeof(OutPoint) > out.data.size()) {
        throw std::runtime_error("Point data buffer size mismatch");
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

      OutPoint p{};
      p.x = x;
      p.y = y;
      p.z = z;
      p.time = static_cast<uint32_t>(dt);
      p.intensity = static_cast<uint8_t>(intensity_i);
      p.tag = tag;
      p.line = line;

      std::memcpy(out.data.data() + out_base, &p, sizeof(OutPoint));
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

    for (const auto &topic : reader.get_all_topics_and_types()) {
      rosbag2_storage::TopicMetadata meta = topic;
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

      const auto old_pc2 = DeserializeMessage<PointCloud2>(bag_message);
      const auto new_pc2 = ConvertOldLidarToNewLidar(old_pc2);
      auto out = SerializeMessage(new_pc2, opts.lidar_topic, bag_message->recv_timestamp, bag_message->send_timestamp);
      writer.write(out);
      ++converted;
    }

    std::cout << "Done. Total messages read: " << total_messages
              << ", converted /livox/lidar messages: " << converted << std::endl;
  } catch (const std::exception &e) {
    std::cerr << "pc2_custom_bag_converter failed: " << e.what() << std::endl;
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
