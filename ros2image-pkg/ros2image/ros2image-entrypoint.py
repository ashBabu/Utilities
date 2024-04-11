import os
import gc
import sys
import yaml
import argparse
sys.path.insert(0, '../../api/')
from ros2image.utils import process_images


def is_valid_file(parser, arg):
    if not os.path.exists(arg):
        parser.error("This file %s does not exist" % arg)
    else:
        return open(arg, 'r')

def main():
    try:
        parser = argparse.ArgumentParser(description="provide yaml config file to class")
        requiredNamed = parser.add_argument_group('required named arguments')
        requiredNamed.add_argument("-p", "--path", dest="path",
                                   type=lambda x: is_valid_file(parser, x),
                                   help="full path to config file for producing the report and plots",
                                   required=True)
        args = parser.parse_args()
        config_path = args.path.name
        with open(config_path, 'r') as config_loader:
            ros2image_config = yaml.load(config_loader, Loader=yaml.FullLoader)
        # Process images across all cores
        process_images.main(ros2image_config)
        gc.collect()

    except Exception as e:
        print(e)


if __name__ == '__main__':
    main()
