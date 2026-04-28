# System-usage instructions
## Connecting to Orin
- The system is run via SSH and devcontainers. This means that your computer is acting more or less as an interface for NVIDIA Thor.
- To start, make sure you have the following extensions installed
    - Dev Containers
    - Remote - SSH
    - Remote Explorer
- If you want to make a new connection:
    1. Open the Remote Explorer -tab on the ribbon
    2. Switch to remotes on the drop-down -menu, if not already chosen
    3. Add a new remote by pressing the + next to SSH
    4. Type in fortis-agv-2@192.168.2.10 and type in the password when prompted
    5. Press File on the top-left corner and open a new folder in /mnt/nvme/fortis_ws/fortis_pilot     (our system has a different path, so the names here might be inaccurate. Just in case, cd one step at a time and check the next step with ls)
    6. Press f1 and click "reopen in container". This will create the image and the container
- If you already have an SSH-connection and a dev-container set-up:
    1. Open the Remote Explorer -tab on the ribbon
    2. Switch to remotes on the drop-down -menu, if not already chosen
    3. Open the drop-down menu under the remote and connect to desired folder
    4. Once connected, press f1 and click "reopen in container". Refrain from using "rebuild and reopen in container", as this will rebuild the entire image, which will take long
    5. Now you are inside the container

## Building after changes
- Whenever changes are made to any file (for example, after pulling new version), the project needs to be built and sourced again. This ensures that ROS can see the new binaries
    1. Make sure you're at the base of the project (fortis_ws or if the name has changed, the directory containing src)
    2. Run colcon build
    3. Run source install/setup.bash
- Now the changes are incorporated into the project

## Issues with building
- Sometimes the build-process fails. This is usually due to some programming error and colcon will tell exactly what broke. However, in the case that the problem is not with programming, but with mismatched build-versions, clearing the build usually helps. This can be done by manually deleting the /build-, /log-, and /install-directories from the workspace, and then running colcon build again.
- Be sure to **NOT** delete /src. You will have to pull again

## Launching the system
- Running the code is done via launch-files. These are python scripts that tell ROS what packages to launch.
- In this pilot, all packages are launched via a master launch-package: fortis_uc1_tau_bringup
- To launch the system, run: ros2 launch fortis_uc1_tau_bringup fortis.launch.py   (you can also tab through this command)

## Changing launched packages
- Currently fortis_uc1_tau_bringup is configured to only launch:
    1. Realsense
    2. Mic-publisher
    3. Laser scan combiner (Package name: Proximity)
    4. Front-lidar publisher
    5. Back-lidar publisher
    6. Roboteq-controller
    7. SLAM
    8. Static transform publisher
    9. Foxglove
- To add or remove launched packages, comment or uncomment the corresponding ld.add_action() from fortis.launch.py
- pilot3_agv.launch.py can be found in fortis_uc1_tau_bringup/launch/fortis.launch.py
- Remember to build if you've made changes

## Checking if topics are receiving messages
- Sometimes topics do not receive messages, for one reason or another. To help debug this, or just to check if a topic is active before recording a ROS-bag, you can echo the topic
    1. Run ros2 topic list  (can skip this if you already know the name of the topic)
    2. Run ros2 topic echo $NAME OF THE TOPIC$  (for example: ros2 topic echo /f_scan would show all lidar messages being published to /f_scan)
    3. ctrl+c when you want to stop reading messages

## Recording ROS-bags
- ROS-bags are the primary way of recording data in a ROS-system. What it does, is record all messages in specified topics (in our case, all topics), and saves them in bag-files. These bags can then be played back later, to resend the messages to their corresponding topics at the same pace they were recorded. This emulates the exact sequence of events the AGV was in during recording
- To record bags:
    1. cd src/bags
    2. Run ros2 bag record -a --max-bag-duration 60     (-a [all topics], --max-bag-duration 60 [automatically starts a new bag after 60s])
    3. ctrl+c when you want to stop recording

## Visualization
- The current system uses Lichtblick to visualize the map, lidars, transforms, etc. This helps with keeping track of progress, checking if topics have gone inactive, or just remote operation.
- To launch the system:
    1. In a new cmd, run ssh -L 8765:localhost:8765 -L 8080:localhost:8080 fortis-agv-2@192.168.2.10 and type in the password of Orin
    2. cd /mnt/nvme/fortis_ws/lichtblick
    3. yarn run web:serve
    4. Wait until done
    5. Make sure that foxglove is launched (refer to Launching the system and Changing launched packages)
    6. In your browser, type in localhost:8080
    7. In the connections menu, choose port 8765
- If you have not used lichtblick/foxglove before, the view might be empty. You can find tutorials on how to populate it online. I would personally recommend opening a 3D-panel and setting /f_scan, /b_scan, /scan_composite, and all the various maps visible.