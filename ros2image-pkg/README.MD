### Description
This is an example python package created to learn python packaging

### Installation
* `sudo apt install python3-pip`
* `pip3 install --no-cache-dir -r requirements.txt`
* `python3 -m pip install -e .`  # `-m` and `-e` respectively stands for module and editable. `e` useful for development and can be skipped when fully tested

### Explanation
* A good video tutorial available [here](https://www.youtube.com/watch?v=v6tALyc4C10&ab_channel=RealPython)
* The [pyproject.toml](pyproject.toml) has the following line
  * [project.scripts]
  * `ros2image = "ros2image.ros2image_entrypoint:main"`
  where `ros2image =` can be any name that you want to call the python executable, 
  the next `ros2image` is the `name` of the python package as mentioned in [pyproject.toml](https://github.com/ashBabu/Utilities/blob/master/ros2image-pkg/pyproject.toml#L6), 
  [ros2image_entrypoint](ros2image/ros2image_entrypoint.py) is the module that contain the function `main()`
* Subdirectories can be imported as `from ros2image.utils import process_images` 
* Sub-subdirectories can be imported as `from ros2image.utils.bag.convert import convert_to_bag` 

### Usage
* Once installed `ros2image` is available globally and it can be run in the terminal as `ros2image -c /path/to/the/config.yaml`
* A `test.yaml` is also provided
* `cd ~/ros2image-pkg` and run `ros2image -c test.yaml`. This will print out `/home/ash/Ash/data-processing/
Converting to bag`
