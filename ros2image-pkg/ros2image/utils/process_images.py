# import utils.bag.convert as bag2hdf
import ros2image.utils.bag.convert as bag2hdf

def main(config):
    bag_path = config['bag_path']
    print(bag_path)
    bag2hdf.convert_to_bag()

if __name__ == "__main__":
    
    import yaml

    with open('ros2image_config.yaml') as config_loader:
        ros2image_config = yaml.load(config_loader, Loader=yaml.FullLoader)

    main(ros2image_config)


    
