# Solution up to level 3
This is the complete solution to the 2024 AMR coursework

## Dependancies
To get this solution to work properly you will need to run these commands
`sudo apt-install ros-humble-navigation2`
`sudo apt-install ros-humble-nav2-bringup`
`sudo apt-install ros-humble-twist-mux`
`pip install pathfinding`

## Correct setup
To get this to work as im not sure how to make dynamic paths make sure that these packages are located in your dev container at `where ever stored\AMR_Workshops\AMR-Assessment` unless you are cloning this repo `https://github.com/Just-A-Tea-Drinker/AMR-Assessment.git`, then install the dependacies and continue


## Usage
### Access the embedded lite Desktop

1. Click on the "Port" in VSCode, find the "novnc" port, right click on it to open the menu, and then choose either "Open in Browser" to open it outside of VSCode or "Preview in Editor" to have it open within VSCode:

   <img width="735" alt="image" src="https://github.com/LCAS/ros2-teaching-ws/assets/1153084/2b0bdfa9-07ea-4238-a0b9-dd2dc8f4c111">

2. (recommended) Set the dekstop scaling by clicking on the settings cog and choose scaling mode "Remote Resizing" if it's not set

   <img width="292" alt="image" src="https://github.com/LCAS/ros2-teaching-ws/assets/1153084/2d9bc88e-7319-4723-968a-0aa08db026ef">

3. click on "Connect" and enter the password `vscode` when prompted:

   <img width="455" alt="image" src="https://github.com/LCAS/ros2-teaching-ws/assets/1153084/ddc224eb-5980-4d9a-994e-b05aa1e9fc1d">

Before using these packages please use `colcon build`
if you are wanting to test this on an environment not already tested or to be certain run `ros2 launch tidy_mapping_mode mapping.launch.py` this solution will work will obstacles, no matter the placement as long as the robot has a line of sight to the walls and boxes


wait for the terminal to shutdown, once shutdown you can run `ros2 launch tidy_cleaning_mode tidy_cleaning.launch.py`
this will automatically begin the tidying behaviour the robot will finish once the message "All moveable boxes have been moved"
`crtl c` to close the program.

## Known Problems
- Even if the box is far away enough to be moved to us, the image processing and global path finding sometimes will not, meaning sometimes a viable path is missed
- Topic names are different in real life compared to the topics used in this simulation
  



