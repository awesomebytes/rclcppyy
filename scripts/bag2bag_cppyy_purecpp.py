#!/usr/bin/env python3

import time
import argparse
import os
import glob
import cppyy
from dataclasses import dataclass
from ament_index_python.packages import get_packages_with_prefixes
from rclcppyy.bringup_rclcpp import bringup_rclcpp


def _try_load_library_candidates(lib_basenames):
    for name in lib_basenames:
        try:
            cppyy.load_library(name)
        except Exception:
            pass

    # Try absolute paths under all package prefixes
    pkgs = get_packages_with_prefixes()
    for _, prefix in pkgs.items():
        lib_dir = os.path.join(prefix, 'lib')
        if not os.path.isdir(lib_dir):
            continue
        for base in lib_basenames:
            pattern = os.path.join(lib_dir, base.replace('.so', '') + '*.so*')
            for candidate in glob.glob(pattern):
                try:
                    cppyy.load_library(candidate)
                except Exception:
                    continue


@dataclass
class Args:
    input_uri: str
    output_uri: str
    image_topic: str
    suffix: str


def main():
    # Ensure rclcpp includes are configured
    bringup_rclcpp()

    # Load likely-needed shared libs (best effort)
    _try_load_library_candidates([
        'librosbag2_cpp.so',
        'librosbag2_storage.so',
        'librosbag2_storage_default_plugins.so',
        'librosbag2_storage_mcap_plugin.so',
        'librclcpp.so',
        'librcl.so',
        'librcutils.so',
        'librcpputils.so',
        'librmw.so'
    ])

    # Includes
    cppyy.include('rosbag2_cpp/reader.hpp')
    cppyy.include('rosbag2_cpp/writer.hpp')
    cppyy.include('rosbag2_storage/serialized_bag_message.hpp')
    cppyy.include('rosbag2_storage/storage_options.hpp')
    cppyy.include('rosbag2_cpp/converter_options.hpp')
    cppyy.include('rclcpp/serialization.hpp')
    cppyy.include('rclcpp/serialized_message.hpp')
    cppyy.include('sensor_msgs/msg/image.hpp')
    # cppyy.include('rcutils/types/rcutils_uint8_array.h')

    cppyy.include('string')
    cppyy.include('vector')
    cppyy.include('memory')
    cppyy.include('unordered_map')

    cppyy.cppdef(r"""
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <rosbag2_cpp/converter_options.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <sensor_msgs/msg/image.hpp>
// #include <rcutils/types/rcutils_uint8_array.h>
#include <unordered_map>
#include <memory>
#include <string>
#include <vector>

static void bgr_or_rgb_to_gray_vec(std::vector<uint8_t>& data,
                                   uint32_t width, uint32_t height, uint32_t step,
                                   bool is_bgr,
                                   std::vector<uint8_t>& out_gray)
{
    out_gray.resize(width*height);
    for (uint32_t y = 0; y < height; ++y) {
        const uint8_t* row = data.data() + y*step;
        uint8_t* grow = out_gray.data() + y*width;
        for (uint32_t x = 0; x < width; ++x) {
            uint8_t c0 = row[3*x+0];
            uint8_t c1 = row[3*x+1];
            uint8_t c2 = row[3*x+2];
            uint8_t b = is_bgr ? c0 : c2;
            uint8_t g = c1;
            uint8_t r = is_bgr ? c2 : c0;
            float yf = 0.114f*b + 0.587f*g + 0.299f*r;
            grow[x] = (uint8_t)(yf);
        }
    }
}

extern "C" void process_bag_to_gray(const std::string& input_uri,
                                    const std::string& output_uri,
                                    const std::string& image_topic,
                                    const std::string& suffix)
{
    rosbag2_cpp::Reader reader;
    rosbag2_storage::StorageOptions inopt;
    inopt.uri = input_uri;
    inopt.storage_id = "mcap";
    rosbag2_cpp::ConverterOptions conv;
    conv.input_serialization_format = "cdr";
    conv.output_serialization_format = "cdr";
    reader.open(inopt, conv);

    rosbag2_cpp::Writer writer;
    rosbag2_storage::StorageOptions outopt;
    outopt.uri = output_uri;
    outopt.storage_id = "mcap";
    writer.open(outopt, conv);

    auto topics = reader.get_all_topics_and_types();
    std::unordered_map<std::string, std::string> topic_types;
    for (auto & md : topics) {
        topic_types[md.name] = md.type;
        if (md.name == image_topic && md.type == "sensor_msgs/msg/Image") {
            rosbag2_storage::TopicMetadata omd;
            omd.name = md.name + suffix;
            omd.type = md.type;
            omd.serialization_format = "cdr";
            writer.create_topic(omd);
        }
    }

    rclcpp::Serialization<sensor_msgs::msg::Image> ser;

    while (reader.has_next()) {
        auto in_msg = reader.read_next();
        auto it = topic_types.find(in_msg->topic_name);
        if (it == topic_types.end()) continue;

        if (in_msg->topic_name == image_topic && it->second == "sensor_msgs/msg/Image") {
            rclcpp::SerializedMessage smsg(*in_msg->serialized_data);
            sensor_msgs::msg::Image im;
            ser.deserialize_message(&smsg, &im);

            bool supported = (im.encoding == "bgr8" || im.encoding == "rgb8");
            if (supported) {
                std::vector<uint8_t> gray;
                bgr_or_rgb_to_gray_vec(im.data, im.width, im.height, im.step, im.encoding == "bgr8", gray);
                im.encoding = "mono8";
                im.step = im.width;
                im.data = std::move(gray);
            }

            rclcpp::SerializedMessage out_smsg;
            ser.serialize_message(&im, &out_smsg);

            auto out = std::make_shared<rosbag2_storage::SerializedBagMessage>();
            out->topic_name = in_msg->topic_name + suffix;
            auto arr = std::make_shared<rcutils_uint8_array_t>();
            *arr = out_smsg.release_rcl_serialized_message();
            out->serialized_data = arr;
            out->recv_timestamp = in_msg->recv_timestamp;

            writer.write(out);
        }
    }
}
""")

    ap = argparse.ArgumentParser(description='Bag-to-bag image grayscale converter (pure C++ via rosbag2_cpp).')
    ap.add_argument('input_uri')
    ap.add_argument('output_uri')
    ap.add_argument('--image-topic', default='/camera/image_raw')
    ap.add_argument('--suffix', default='_gray')
    a = ap.parse_args()

    # Time it
    start_time = time.time()
    cppyy.gbl.process_bag_to_gray(a.input_uri, a.output_uri, a.image_topic, a.suffix)
    end_time = time.time()
    print(f"Time taken: {end_time - start_time} seconds")


if __name__ == '__main__':
    main()


# After the slow upbringing...
# 0.283s
# 0.307s
# 0.338s