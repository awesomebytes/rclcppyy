#!/usr/bin/env python3

import time
import argparse
from dataclasses import dataclass
import numpy as np
from array import array

from rosbag2_py import SequentialReader, SequentialWriter, StorageOptions, ConverterOptions, TopicMetadata
from rclpy.serialization import deserialize_message, serialize_message
from rosidl_runtime_py.utilities import get_message


IMG_TYPE = 'sensor_msgs/msg/Image'


@dataclass
class Args:
    input_uri: str
    output_uri: str
    image_topic: str
    suffix: str


def to_gray_python(img):
    h, w, step = img.height, img.width, img.step
    enc = img.encoding
    assert enc in ('bgr8', 'rgb8'), f'Unsupported encoding: {enc}'
    buf = memoryview(img.data)
    arr = np.frombuffer(buf, dtype=np.uint8).reshape(h, step)
    rgb = arr[:, :w * 3].reshape(h, w, 3)
    if enc == 'bgr8':
        b, g, r = rgb[..., 0], rgb[..., 1], rgb[..., 2]
    else:
        r, g, b = rgb[..., 0], rgb[..., 1], rgb[..., 2]
    gray = (0.114 * b + 0.587 * g + 0.299 * r).astype(np.uint8)
    img.encoding = 'mono8'
    img.step = w
    img.data = array('B', gray.tobytes())
    return img


def main():
    p = argparse.ArgumentParser(description='Bag-to-bag image grayscale converter (pure Python baseline).')
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
                serialization_format='cdr'
            ))

    Image = get_message(IMG_TYPE)

    while reader.has_next():
        topic, data, t = reader.read_next()
        typ = topic_types.get(topic)
        if typ == IMG_TYPE and topic == args.image_topic:
            msg = deserialize_message(data, Image)
            msg = to_gray_python(msg)
            out_ser = serialize_message(msg)
            writer.write(topic + args.suffix, out_ser, t)

    end_time = time.time()
    print(f"Time taken: {end_time - start_time} seconds")

if __name__ == '__main__':
    main()

# 0.343s
# 0.369s