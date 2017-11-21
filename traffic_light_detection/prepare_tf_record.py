#!/usr/bin/env python


'''
Handy script to prepare the data for Tensorflow object detection API. 
'''


import tensorflow as tf
import yaml
import os
import sys

from object_detection.utils import dataset_util


flags = tf.app.flags

flags.DEFINE_string('output_path', 'out.record', 'Path to output TFRecord')
flags.DEFINE_string('annotation_file', '', 'Path to annotation file')
flags.DEFINE_string('data_folder', '', 'Path to data folder')
flags.DEFINE_integer('image_height', 600, 'Height of the image')
flags.DEFINE_integer('image_width', 800, 'Width of the image')

FLAGS = flags.FLAGS

LABELS =  {
    "Green" : 1,
    "Red" : 2,
    "Yellow" : 3,
    "off" : 4,
}

def create_tf_example(example):
    width = FLAGS.image_width
    height = FLAGS.image_height

    filename = example['filename'] # Filename of the image. Empty if image is not from file
    filename = filename.encode()

    with tf.gfile.GFile(example['filename'], 'rb') as fid:
        encoded_image = fid.read()

    image_format = 'jpg'.encode() 

    xmins = [] # List of normalized left x coordinates in bounding box (1 per box)
    xmaxs = [] # List of normalized right x coordinates in bounding box
                # (1 per box)
    ymins = [] # List of normalized top y coordinates in bounding box (1 per box)
    ymaxs = [] # List of normalized bottom y coordinates in bounding box
                # (1 per box)
    classes_text = [] # List of string class name of bounding box (1 per box)
    classes = [] # List of integer class id of bounding box (1 per box)

    for box in example['annotations']:
        xmins.append(float(box['xmin'] / width))
        xmaxs.append(float((box['xmin'] + box['x_width']) / width))
        ymins.append(float(box['ymin'] / height))
        ymaxs.append(float((box['ymin']+ box['y_height']) / height))
        classes_text.append(box['class'].encode())
        classes.append(int(LABELS[box['class']]))


    tf_example = tf.train.Example(features=tf.train.Features(feature={
        'image/height': dataset_util.int64_feature(height),
        'image/width': dataset_util.int64_feature(width),
        'image/filename': dataset_util.bytes_feature(filename),
        'image/source_id': dataset_util.bytes_feature(filename),
        'image/encoded': dataset_util.bytes_feature(encoded_image),
        'image/format': dataset_util.bytes_feature(image_format),
        'image/object/bbox/xmin': dataset_util.float_list_feature(xmins),
        'image/object/bbox/xmax': dataset_util.float_list_feature(xmaxs),
        'image/object/bbox/ymin': dataset_util.float_list_feature(ymins),
        'image/object/bbox/ymax': dataset_util.float_list_feature(ymaxs),
        'image/object/class/text': dataset_util.bytes_list_feature(classes_text),
        'image/object/class/label': dataset_util.int64_list_feature(classes),
    }))

    return tf_example


def main(_):
    
    writer = tf.python_io.TFRecordWriter(FLAGS.output_path)
  
    # Udacity
    yaml_file = FLAGS.annotation_file
    data_folder = FLAGS.data_folder


    samples = yaml.load(open(yaml_file, 'rb').read())
    
    for sample in samples:
        sample['filename'] = '%s%s'%(data_folder, sample['filename'])
        tf_example = create_tf_example(sample)
        writer.write(tf_example.SerializeToString())

    writer.close()



if __name__ == '__main__':
    tf.app.run()