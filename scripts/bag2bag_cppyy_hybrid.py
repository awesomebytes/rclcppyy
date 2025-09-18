#!/usr/bin/env python3

import time
import argparse
from dataclasses import dataclass
import numpy as np
from array import array
import cppyy

from rosbag2_py import SequentialReader, SequentialWriter, StorageOptions, ConverterOptions, TopicMetadata
from rclpy.serialization import deserialize_message, serialize_message
from rosidl_runtime_py.utilities import get_message


IMG_TYPE = 'sensor_msgs/msg/Image'


cppyy.cppdef(r"""
#include <cstdint>
#include <cstddef>

extern "C" void bgr_or_rgb_to_gray(
    const uint8_t* src, uint8_t* dst,
    size_t width, size_t height, size_t step_bytes,
    bool is_bgr)
{
    for (size_t y = 0; y < height; ++y) {
        const uint8_t* row = src + y*step_bytes;
        uint8_t* grow = dst + y*width;
        for (size_t x = 0; x < width; ++x) {
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
""")


@dataclass
class Args:
    input_uri: str
    output_uri: str
    image_topic: str
    suffix: str


def to_gray_cppyy(img):
    h, w, step = img.height, img.width, img.step
    enc = img.encoding
    assert enc in ('bgr8', 'rgb8'), f'Unsupported encoding: {enc}'
    is_bgr = (enc == 'bgr8')

    src = np.frombuffer(memoryview(img.data), dtype=np.uint8)
    gray = np.empty(h * w, dtype=np.uint8)

    cppyy.gbl.bgr_or_rgb_to_gray(
        src.ctypes.data,
        gray.ctypes.data,
        w, h, step, is_bgr)
    
    gray = gray.reshape(h, w)

    img.encoding = 'mono8'
    img.step = w
    img.data = array('B', gray.tobytes())
    return img


def main():
    p = argparse.ArgumentParser(description='Bag-to-bag image grayscale converter (hybrid: Python IO + C++ pixel loop).')
    p.add_argument('input_uri')
    p.add_argument('output_uri')
    p.add_argument('--image-topic', default='/camera/image_raw')
    p.add_argument('--suffix', default='_gray')
    a = p.parse_args()
    args = Args(a.input_uri, a.output_uri, a.image_topic, a.suffix)

    reader = SequentialReader()
    reader.open(
        StorageOptions(uri=args.input_uri, storage_id='mcap'),
        ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr')
    )

    writer = SequentialWriter()
    writer.open(
        StorageOptions(uri=args.output_uri, storage_id='mcap'),
        ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr')
    )

    start_time = time.time()
    try:
        topics = reader.get_all_topics_and_types()
    except AttributeError:
        topics = reader.get_topics_and_types()
    topic_types = {t.name: t.type for t in topics}

    for t in topics:
        if t.type == IMG_TYPE and t.name == args.image_topic:
            writer.create_topic(TopicMetadata(
                id=0,
                name=t.name + args.suffix,
                type=t.type,
                serialization_format='cdr',
            ))

    Image = get_message(IMG_TYPE)

    while reader.has_next():
        topic, data, t = reader.read_next()
        if topic_types.get(topic) == IMG_TYPE and topic == args.image_topic:
            msg = deserialize_message(data, Image)
            msg = to_gray_cppyy(msg)
            writer.write(topic + args.suffix, serialize_message(msg), t)
    end_time = time.time()
    print(f"Time taken: {end_time - start_time} seconds")


if __name__ == '__main__':
    main()


