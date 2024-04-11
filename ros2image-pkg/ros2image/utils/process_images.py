import os
import shutil
import cv2
import glob
import numpy as np
import pandas as pd
from multiprocessing import Queue, Process, cpu_count
import ros2image.utils.bag.convert as bag2hdf


# Create a queue for multithreading 
def create_queue(bag_path):
    
    # Return a list of bagfiles from the specified bag_path
    def get_bagfile_list(bag_path):
        if bag_path[-1] != '/':
            bag_path += '/'
        return [bag_path + bagfile for bagfile in os.listdir(bag_path)  if bagfile.endswith('.bag')]

    files_to_process = Queue()

    print('Creating rosbag queue...')
    for bagfile in get_bagfile_list(bag_path):
        files_to_process.put(bagfile)

    return files_to_process


def execute(tasks_to_accomplish, path_out, asset_ID, process_ID, extract_images, extract_hdf):
    
    cpu_df = pd.DataFrame()
    while not tasks_to_accomplish.empty():
        
        print('Queue size: %s' %(str(tasks_to_accomplish.qsize())))
        try:
            '''
                try to get task from the queue.get_nowait() function will
                raise queue.Empty exception if the queue is empty.
                queue(False) function would do the same task also.
            '''
            rosbag_file = tasks_to_accomplish.get()
            
            
            try:
                
                # Process images
                if(extract_images):
                    rosbag_df = ros2image.utils.ros2image.rosbag_2_image(rosbag_file, path_out, asset_ID)
                    cpu_df = cpu_df.append(rosbag_df)
                
                # Convert bag to hdf
                if(extract_hdf):
                    bag2hdf.rosbag_2_hdf5(rosbag_file, path_out + 'hdf/')
                                
            except Exception as e:
                print(e)

        except Queue.Empty:
            print('still going')
            break
            
            
    cpu_df.to_csv('/home/ros2image/df_tmp/%s.csv'%(process_ID), index=False)
    
    return True


# Merge all tmp dfs into master df and save to output_path. remove_tmp_dfs will remove the tmp df folder
def merge_dfs(df_path, output_path, remove_tmp_dfs=True):
    merged_df = pd.concat([pd.read_csv(df_path + file )for file in os.listdir(df_path) if file.endswith('.csv')], axis=0)
    merged_df.to_csv(output_path, index=False)
    
    if remove_tmp_dfs:
        os.system('rm -r %s' %(df_path))
        
    

# Main controller fn that creates the processes and queue. Each item is taken from the queue with dfs saved for each
# process (having a shared resource would be alot slower (tonnes of idling)). dfs are then merged when all processes
# are complete
def main(config):

    # config['path_out'] = 'images/processed/'
    if not os.path.isdir(config['path_out']):
        os.mkdir(config['path_out'])

    if config['extract_images']:
        for sdir in config['subdirs']:
            if not os.path.isdir(config['path_out'] + sdir):
                os.mkdir(config['path_out'] + sdir)


    # Create temp df folder (will be deleted at the end of main)
    if not os.path.isdir('/home/ros2image/df_tmp/'):
        os.mkdir('/home/ros2image/df_tmp/')
        
    processes = []
    # tasks_to_accomplish = create_queue('rosbags/')
    tasks_to_accomplish = create_queue(config['bag_path'])

    # creating processes
    number_of_processes = cpu_count() - 1
    print('Number of CPUs: %s' %(str(number_of_processes)))
    for process_ID in range(number_of_processes):
        proc = Process(target=execute, args=(tasks_to_accomplish, config['path_out'],
                                             config['asset_ID'], str(process_ID), config['extract_images'], config['extract_hdf']))
        processes.append(proc)
        proc.start()
    print("process started")

    # completing process
    for p in processes:
        p.join()
    print('Process finished')
    
    # merge_dfs('/home/ros2image/df_tmp/', 'images/log_file.csv')
    merge_dfs('/home/ros2image/df_tmp/', config['path_out'] + 'log_file.csv')
    print('DF merged')
    
    
# Generate stitched mp4
def generate_mp4(save_path, fps):
    
    
    # Stitch images into a single image
    def stitch_images(left, right, top, forward, ID, video_shape):

        img = np.zeros(video_shape)

        
        img[512:, 384:] = cv2.resize(right, (512, 384))
        image = cv2.putText(img, 'Right', (750, 420), 2, 0.75, (255, 255, 255))


        # Bottom left
        img[512:, :384] = cv2.resize(left, (512, 384))
        image = cv2.putText(img, 'Left', (250, 420), 2, 0.75, (255, 255, 255))


        # Top left
        img[:512, :384] = cv2.resize(top, (512, 384))
        image = cv2.putText(img, 'Top', (250, 38), 2, 0.75, (255, 255, 255))


        # Top right
        img[:512, 384:] = cv2.resize(forward, (512, 384))
        image = cv2.putText(img, 'Forward', (750, 38), 2, 0.75, (255, 255, 255))


        image = cv2.putText(img, ID, (800, 800), 2, 0.75, (255, 255, 255))

        return img.astype(np.uint8)

    # left_paths = sorted(glob.glob('images/processed/left_camera/*.jpeg'))
    # right_paths = sorted(glob.glob('images/processed/right_camera/*.jpeg'))
    # top_paths = sorted(glob.glob('images/processed/top_camera/*.jpeg'))
    # forward_paths = sorted(glob.glob('images/processed/forward_camera/*.jpeg'))
    left_paths = sorted(glob.glob('/home/ujjar/Data/Suez/Telesto_Data/1st_run//left_camera/*.jpeg'))
    right_paths = sorted(glob.glob('/home/ujjar/Data/Suez/Telesto_Data/1st_run/right_camera/*.jpeg'))
    top_paths = sorted(glob.glob('/home/ujjar/Data/Suez/Telesto_Data/1st_run/top_camera/*.jpeg'))
    forward_paths = sorted(glob.glob('/home/ujjar/Data/Suez/Telesto_Data/1st_run/forward_camera/*.jpeg'))
    
    
    video_shape = (1024, 768, 3)
#     imgs = np.ones((len(left_paths), video_shape[0], video_shape[1], video_shape[2]))
    for idx, (left, right, top, forward) in enumerate(zip(left_paths, right_paths, top_paths, forward_paths)):

        
        ID = left.split('/')[-1].split('.')[0]
        left = cv2.imread(left)
        right = cv2.imread(right)
        top = cv2.imread(top)
        forward = cv2.imread(forward)

        fname = '/home/ujjar/Data/Suez/Telesto_Data/1st_run/tmp_stitched_dir/' + ID + '.jpeg'
        img = stitch_images(left, right, top, forward, ID, video_shape)
        cv2.imwrite(fname, img)
        
        
    ffmpeg_arg = 'ffmpeg -r %s -pattern_type glob -i \"/home/ujjar/Data/Suez/Telesto_Data/1st_run/tmp_stitched_dir/*.jpeg\" %s' %(fps, save_path)
    os.system(ffmpeg_arg)
        
#         imgs[idx] = stitch_images(left, right, top, forward, ID, video_shape)
        
#     imageio.mimwrite(save_path, imgs.astype(np.uint8) , fps = fps)

    
def generate_singleview_mp4(save_path, fps, camera_view, flip=''):
    if not os.path.isdir(save_path + 'tmp_singleview_dir/'):
        os.mkdir(save_path + 'tmp_singleview_dir/')
    paths = sorted(glob.glob(save_path + camera_view + '_camera/*.jpeg'))
    video_shape = (1024, 768, 3)
    for idx, cam in enumerate(paths):
        ID = cam.split('/')[-1].split('.')[0]
        cam_img = cv2.imread(cam)
        if flip == 'v':
            cam_img = cv2.flip(cam_img, 0)
        if flip == 'h':
            cam_img = cv2.flip(cam_img, 1)
        if flip == 'hv' or flip == 'vh':
            cam_img = cv2.flip(cam_img, -1)
        fname = save_path + 'tmp_singleview_dir/' + ID + '.jpeg'
        print(fname)
        img = cv2.resize(cam_img, (1024, 768))
        image = cv2.putText(img, camera_view, (900, 38), 2, 0.75, (255, 255, 255))
        image = cv2.putText(img, ID, (900, 750), 2, 0.75, (255, 255, 255))
        img = img.astype(np.uint8)
        cv2.imwrite(fname, img)
    ffmpeg_arg = 'ffmpeg -r %s -pattern_type glob -i \"%stmp_singleview_dir/*.jpeg\" %s' %(fps, save_path, save_path + camera_view + '_video.mp4')
    os.system(ffmpeg_arg)
    shutil.rmtree(save_path + 'tmp_singleview_dir/')


if __name__ == "__main__":
    
    import yaml

    with open('ros2image_config.yaml') as config_loader:
        ros2image_config = yaml.load(config_loader, Loader=yaml.FullLoader)

    main(ros2image_config)


    
