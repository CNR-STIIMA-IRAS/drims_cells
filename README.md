### drims_cells

This repository contains the robot descriptions, MoveIt configurations, and controller settings for the UR5e and PAL Tiago Pro robots used in the DRIMS3 Summer School 2026.

To launch the robotic cells, use the following commands.

For the UR5e with hand-e gripper robots:

```bash
ros2 launch drims_description ur5e_1_start.launch.py fake:=true
```

> ⚠️  Use `ur5e_1_start```,  ```ur5e_2_start`, or ```ur5e_3_start` according to the actual robot you're using.

For the Tiago Pro robot:
```bash
ros2 launch drims_description tiago_pro_start.launch.py fake:=true
```

> ⚠️  Use `fake:=true` to run in simulation mode, and `fake:=false` to connect to the real hardware.


## Newtwork setup to connect to the robot

### UR robot

To connect to the UR robot, first plug the **Ethernet cable** from the robot to your laptop.  

1. On Ubuntu, go to **Settings → Network → Wired → Add**.  
2. Create a new wired network with the configuration shown in the image below:  

![Network Settings](media/network_settings.png)


To verify that the connection is working, open a terminal and run:

- For robot 1:
    ```bash
    ping 192.164.254.101
    ```
- For robot 2:
    ```bash
    ping 192.164.254.102
    ```
- For robot 3:
    ```bash
    ping 192.164.254.103
    ```
  
If the network is correctly configured, you should see replies from the robot as shown below:

![Network Settings](media/ping.png)

### Tiago Pro

1. **Network setup**  
   - Connect your laptop to the robot using a **wired DHCP connection**.  

2. **Start the Docker environment**  
   - Once inside the container, run:  
     ```bash
     source setup_robot_connection.sh
     ```
   - Ensure that all messages are displayed without errors.

3. **Verification**  
   - Check that the topics from Tiago are visible:  
     ```bash
     ros2 topic list
     ```
   - When opening new terminals, use:  
     ```bash
     ./connect.sh
     ```
     and verify that the same topics are still visible.

