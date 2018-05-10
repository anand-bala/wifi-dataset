# RGB-D/WiFi Dataset

This repository provides a ROS workspace to work with the RGB-D/WiFi Dataset.

## Directory Description

- `carto_ws`:
  - This workspace contains all Cartographer related packages that need to be
      built with `catkin_make_isolated --use-ninja --install`
- `catkin_ws`:
  - This contains all non-Cartographer packages (including the `dataset_ros`
      workspace)
  - Build this workspace with `catkin_make`
- `scripts`:
  - Contains scripts to clean up the data, do initial setup/installation,
      extract data, etc.
- `env`:
  - Source the `env/setup.sh` for basic shell setup (environment variables and
      such)
