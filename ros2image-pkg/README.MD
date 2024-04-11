### Description
The 'ros2image' package multithreads the image augmentation over multiple cores (cpu_count() -1) of a machine. This will be run on server with alot of cpu cores allowing for efficient multithreading and read/write currently using the Dropbox API.

### Installation
* `sudo apt install python3-pip`
* `pip3 install --no-cache-dir -r requirements.txt`
* `python3 -m pip install -e .`  # `-m` and `-e` respectively stands for module and editable. `e` useful for development and can be skipped when fully tested

### Explanation
* A good video tutorial available [here](https://www.youtube.com/watch?v=v6tALyc4C10&ab_channel=RealPython)
* The [pyproject.toml](pyproject.toml) has the following line
  * [project.scripts]
  * `ros2image = "ros2image.ros2image-entrypoint:main"`
  where `ros2image =` can be any name that you want to call the python executable, 
  the next `ros2image` is the name of the python package as mentioned in [pyproject.toml](pyproject.toml), 
  [ros2image-entrypoint](ros2image/ros2image-entrypoint.py) is the module that contain the function `main`
* Subdirectories can be imported as `from ros2image.utils import process_images` 
* Sub-subdirectories can be imported as `from ros2image.utils.bag.convert import convert_to_bag` 

### Usage
Once installed `ros2image` is available globally and it can be run in the terminal as `ros2image -p /path/to/the/config.yaml`