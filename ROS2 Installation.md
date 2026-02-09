### **Phase 1: Install ROS 2 Humble**

*This sets up the core libraries and communication middleware.*

**1. Set Locale & Enable Universe Repository**

```bash
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

sudo apt install software-properties-common
sudo add-apt-repository universe

```

**2. Add the ROS 2 GPG Key**

```bash
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

```

**3. Add the Repository to Sources**

```bash
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

```

**4. Install ROS 2 Humble**

```bash
sudo apt update
sudo apt upgrade
sudo apt install ros-humble-desktop

```

---

### **Phase 2: Install Build Tools**

*You need these to compile code and manage dependencies. This fixes the "command not found" errors you encountered earlier.*

**1. Install Colcon & Rosdep**

```bash
sudo apt install python3-colcon-common-extensions python3-rosdep

```

**2. Initialize Rosdep**
*Crucial step. If you skip this, dependency checks will fail.*

```bash
sudo rosdep init
rosdep update

```

---

### **Phase 3: Set Up Your Workspace**

*This is where your actual code will live.*

**1. Create the Directory Structure**

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws

```

**2. Build the Workspace (First Run)**
Even though it's empty, this creates the necessary `build/`, `install/`, and `log/` folders.

```bash
colcon build

```

---

### **Phase 4: Automate Your Environment**

*This ensures you don't have to manually "source" setup files every time you open a terminal.*

Run these commands **once** to add them to your `.bashrc` file:

```bash
# 1. Source the main ROS 2 installation (The Underlay)
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# 2. Source your specific workspace (The Overlay)
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc

# 3. Add Colcon auto-complete (Optional but recommended)
echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> ~/.bashrc

# 4. Apply changes immediately
source ~/.bashrc

```

### **Phase 5: Verification**

To confirm everything is working, close your terminal, open a new one, and run:

```bash
ros2 run demo_nodes_cpp talker

```

If you see **"Publishing: Hello World..."**, your environment is perfect.
