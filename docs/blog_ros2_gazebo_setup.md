# How to Match ROS 2, Gazebo, and PyTorch CUDA Versions - For Any Ubuntu and GPU

**TL;DR: ROS 2 distro, Gazebo version, CUDA toolkit, and PyTorch build must all match your Ubuntu version and GPU architecture. This guide gives you the decision process - not just commands to copy-paste - so it works regardless of when you're reading it.**

---

I had an RTX 5090, a fresh Ubuntu 24.04 install, and three "definitive" ROS 2 tutorials open in browser tabs. Eight hours later, I had a broken Python environment, a Gazebo window rendering on CPU, and a PyTorch install that silently refused to touch the GPU.

The tutorials weren't wrong - they were written for Ubuntu 22.04, Humble, and RTX 30-series cards. I was one Ubuntu version, one ROS distro, and two GPU generations ahead of every guide on the internet.

Here's everything I figured out, structured so you can get the same stack running in 30 minutes instead of 8 hours.

---

## The Version Chain (This Is the Whole Post in Two Lines)

```
Ubuntu version → ROS 2 distro → Gazebo version
GPU model → compute capability → PyTorch CUDA build
```

Every problem in this guide traces back to getting one of these arrows wrong. The rest of the post teaches you how to resolve each arrow correctly.

---

## Before You Start: Know Your System

Run these commands and write down the results. You'll need them for every decision that follows.

```bash
# Ubuntu version and codename
lsb_release -a
# GPU model, driver version, CUDA version (top-right corner of output)
nvidia-smi
# Python version
python3 --version
# Display server (matters for Gazebo rendering)
echo $XDG_SESSION_TYPE
```

For my machine, this gave me:
- Ubuntu 24.04 LTS (Noble)
- RTX 5090, Driver 580.126, CUDA 13.0
- Python 3.12.3
- X11

Your numbers will be different, but the process is the same.

---

## Part 1: Picking the Right ROS 2 Distro (~2 minutes)

This is the decision everything else depends on. **ROS 2 distros are locked to specific Ubuntu versions.** There's no way around this - the binary packages literally don't exist for other Ubuntu releases.

### How to Find Your Match

Check the [ROS 2 Distributions page](https://docs.ros.org/en/rolling/Releases.html):

- **Ubuntu 22.04 Jammy** → **Humble** (LTS, supported until 2027)
- **Ubuntu 24.04 Noble** → **Jazzy** (LTS, supported until 2029)

The rule is simple: **use the LTS distro that targets your Ubuntu version.**

I made the mistake of initially planning for Humble because every tutorial uses it. On Ubuntu 24.04, `apt install ros-humble-desktop` returns "package not found." There's no workaround that's worth the trouble - no PPA, no manual build, no Docker hack that doesn't add layers of complexity. Just use Jazzy. The API is 95% identical to Humble.

> **"Should I just use Docker?"** If you only need ROS 2 (no GPU, no Gazebo GUI), Docker is fine. But if you need GPU-accelerated PyTorch AND Gazebo rendering, Docker adds real complexity: NVIDIA Container Toolkit, X11 socket forwarding, GPU passthrough config. For a development machine where you'll be iterating daily, native install is less friction. Save Docker for CI/CD and deployment.

---

## Part 2: Picking the Right Gazebo Version (~1 minute)

Gazebo versions are paired with ROS 2 distros. The integration packages (`ros-<distro>-ros-gz`) are compiled against a specific Gazebo release. Mix them and you get ABI errors.

### How to Find Your Match

Check the [official pairing table](https://gazebosim.org/docs/harmonic/ros_installation/):

- **Humble** → Fortress (Gz Sim 6) → `ros-humble-ros-gz`
- **Jazzy** → Harmonic (Gz Sim 8) → `ros-jazzy-ros-gz`

If you're on Jazzy, you use Harmonic. Period.

**Naming confusion alert**: "Gazebo" has been rebranded. The old simulator (Gazebo 9, 11) is now "Gazebo Classic" and is deprecated. The new one - formerly "Ignition Gazebo" - is just "Gazebo." You'll use the `gz` command, not the old `gazebo` command. If a tutorial tells you to run `gazebo`, it's outdated.

---

## Part 3: The Installation (~15 minutes)

Now that we know what versions to install, let's do it.

### Prep Work

```bash
# Fix locale (prevents build warnings down the line)
sudo locale-gen en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
# Update everything first - ROS 2 depends on system libraries like libc6.
# Stale versions cause mysterious conflicts.
sudo apt update && sudo apt upgrade -y
# Prerequisites
sudo apt install -y software-properties-common curl mesa-utils
```

Before going further, verify your GPU is actually rendering:

```bash
glxinfo | grep "OpenGL renderer"
```

You should see your NVIDIA GPU name here. If it says `llvmpipe`, your NVIDIA drivers aren't set up correctly - fix that first, or Gazebo will either crash or render on CPU.

> **Wayland users**: Gazebo's rendering engine (Ogre2) works best on X11. If you're on a Wayland session and Gazebo shows a black screen, log out and select "GNOME on Xorg" at the login screen.

### Install ROS 2

Replace `jazzy` and `noble` with your target distro and Ubuntu codename if different.

```bash
# Add the ROS 2 GPG key
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
 -o /usr/share/keyrings/ros-archive-keyring.gpg
# Add the repository
echo "deb [arch=amd64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu noble main" | \
  sudo tee /etc/apt/sources.list.d/ros2.list
sudo apt update
# Install ROS 2 desktop (includes RViz, rclpy, demo nodes)
sudo apt install -y ros-jazzy-desktop
# Build tools
sudo apt install -y python3-colcon-common-extensions python3-rosdep
# Initialize rosdep
sudo rosdep init 2>/dev/null || true
rosdep update
```

Verify it works:

```bash
source /opt/ros/jazzy/setup.bash  # use setup.zsh if you're a zsh user
ros2 topic list
```

You should see `/parameter_events` and `/rosout`. If you do, ROS 2 is alive.

### Install Gazebo + ROS Integration

```bash
# This pulls in Gazebo Harmonic automatically
sudo apt install -y ros-jazzy-ros-gz
# For robot simulation with ros2_control
sudo apt install -y ros-jazzy-gz-ros2-control
```

Verify:

```bash
gz sim --version
# Expected: Gazebo Sim, version 8.x.x
```

Now the real test - run this on your desktop (not over SSH):

```bash
gz sim shapes.sdf
```

A 3D window should open with colored shapes. If it renders smoothly, your GPU + Gazebo pipeline is working.

> **Running on a headless server or via SSH?** Gazebo needs a display for rendering. If your GPU machine is headless, use [RustDesk](https://rustdesk.com/) (free, open-source) or VNC for remote desktop access. RustDesk handles GPU-accelerated 3D rendering over the network with minimal setup.

### Install Common Robotics Packages

Most robotics projects need navigation, robot control, and description tools:

```bash
# Navigation stack
sudo apt install -y ros-jazzy-navigation2 ros-jazzy-nav2-bringup
# Robot control framework
sudo apt install -y ros-jazzy-ros2-control ros-jazzy-ros2-controllers
# URDF/Xacro tools
sudo apt install -y ros-jazzy-xacro \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-joint-state-publisher-gui
```

Quick sanity check:

```bash
# Should show ~34 nav2 packages
ros2 pkg list | grep -c nav2
# Should show controller_manager, ros2_control, etc.
ros2 pkg list | grep ros2_control
```

---

## Part 4: The Python Virtual Environment Trap (~5 minutes)

This is the section that breaks most ML-robotics setups - and the fix is one flag.

If you're doing any ML/AI work alongside ROS 2 - running YOLO for object detection, using LangChain for an LLM agent - you need pip packages. And pip packages should live in a virtual environment.

**The trap: a standard Python venv can't see ROS 2 packages.** You'll get `ModuleNotFoundError: No module named 'rclpy'` and wonder what went wrong.

ROS 2 installs `rclpy` and other Python bindings under `/opt/ros/jazzy/lib/python3.12/site-packages/`. A normal venv isolates itself from system packages, which means `import rclpy` fails inside the venv.

### The Fix

Create the venv with `--system-site-packages`:

```bash
cd ~/your_project && python3 -m venv --system-site-packages venv
source venv/bin/activate
pip install --upgrade pip
```

This gives you the best of both worlds:
- ROS 2 packages (`rclpy`, etc.) are visible through the system site-packages
- Anything you `pip install` goes into the venv, isolated from the system

### The Source Order Gotcha

When setting up your terminal, **always source ROS first, then activate the venv**:

```bash
source /opt/ros/jazzy/setup.bash   # FIRST
source ~/your_project/venv/bin/activate  # SECOND
```

Why? The ROS setup script adds its Python paths to `PYTHONPATH`. Then venv activation prepends the venv's `site-packages`. This layering means your pip packages take priority, with ROS packages as a fallback.

Reverse the order and `PYTHONPATH` gets mangled - your venv packages won't take priority, and you'll get version conflicts that are impossible to debug.

### Packages That Will Bite You

Some Python packages exist in both the ROS 2 apt world and the pip world. Installing both leads to version conflicts:

- **PyYAML** - ROS 2 provides it (`python3-yaml`). **Do NOT pip install.** This is the #1 source of cryptic import errors.
- **NumPy** - Usually provided by system apt. Pip installing a newer version is usually fine.
- **OpenCV** - Not in base ROS 2. Safe to pip install.

Rule of thumb: run `apt list --installed | grep python3-<package>` first. If it's there, don't pip install it.

---

## Part 5: PyTorch with GPU Support (~10 minutes)

This section is where I lost the most time. Getting PyTorch to actually use your GPU requires matching three things:

1. **Your GPU's compute capability** (hardware)
2. **Your CUDA driver version** (system)
3. **The PyTorch CUDA build** (pip package)

### Step 1: Find Your GPU's Compute Capability

Every NVIDIA GPU has a "compute capability" version that determines what CUDA features it supports. Find yours at [NVIDIA's CUDA GPUs page](https://developer.nvidia.com/cuda-gpus):

- **RTX 20xx** - Turing - sm_75
- **RTX 30xx** - Ampere - sm_86
- **RTX 40xx** - Ada Lovelace - sm_89
- **RTX 50xx** - Blackwell - sm_120

### Step 2: Check Your CUDA Driver

```bash
nvidia-smi
```

Look at the top-right of the output for `CUDA Version: XX.X`. This is the maximum CUDA toolkit version your driver supports. Your PyTorch CUDA build must be at or below this version.

### Step 3: Find a Compatible PyTorch Build

PyTorch publishes separate builds for different CUDA versions. Here's how to check what's available:

```bash
pip index versions torch --index-url https://download.pytorch.org/whl/cu124
pip index versions torch --index-url https://download.pytorch.org/whl/cu126
pip index versions torch --index-url https://download.pytorch.org/whl/cu128
```

### The Compatibility Rules

**CUDA driver backward compatibility**: The CUDA driver version (shown by `nvidia-smi`) must be >= the CUDA toolkit version in the PyTorch wheel. So a `cu124` build works fine on a system with CUDA driver 12.8 or 13.0 - the driver is newer than the toolkit, which is fine.

**GPU architecture support**: This is where it gets tricky. Each PyTorch build only includes pre-compiled kernels for certain GPU architectures. If your GPU isn't included, PyTorch will either:
- Show a compatibility warning and attempt PTX fallback (may work with degraded performance)
- Throw a hard runtime error ("no kernel image is available for execution on the device")
- In some cases, silently fall back to CPU

Here's a reference for which CUDA builds support which GPUs:

- **RTX 20xx** (Turing, sm_75) - cu102+ - Generally works with most builds
- **RTX 30xx** (Ampere, sm_86) - cu113+ - "sm_86 not compatible" warning with older
- **RTX 40xx** (Ada, sm_89) - cu118+ - "sm_89 not compatible" warning with older
- **RTX 50xx** (Blackwell, sm_120) - cu128 - Warnings or runtime errors with older builds

I learned this the hard way. My RTX 5090 (Blackwell, sm_120) showed compatibility warnings with both `cu124` and `cu126` builds. While basic tensor operations seemed to work via PTX fallback, this isn't reliable - some operations can fail at runtime. Only `cu128` (specifically PyTorch 2.10.0+cu128, which I tested) included native sm_120 kernels and ran completely warning-free.

### The Install (Order Matters!)

```bash
source venv/bin/activate
# Install PyTorch FIRST with the correct CUDA version
# I tested with 2.10.0+cu128 on an RTX 5090 - adjust the index URL for your GPU
pip install torch torchvision --index-url https://download.pytorch.org/whl/cu128
```

**Critical**: Install PyTorch before any package that depends on it - especially `ultralytics` (YOLO). If you `pip install ultralytics` first, it auto-installs CPU-only PyTorch as a dependency. Then you have to uninstall and reinstall torch with the CUDA version, which can leave broken state.

The correct order:
```bash
# 1. GPU PyTorch first
pip install torch torchvision --index-url https://download.pytorch.org/whl/cu128
# 2. THEN packages that depend on it
pip install ultralytics  # Will use the GPU torch you just installed
```

> For reference: I tested with PyTorch **2.10.0+cu128** on an RTX 5090 with CUDA 13.0 driver, and everything worked without warnings. If you're on a different GPU, adjust the CUDA index URL accordingly.

### Verify Everything Works

```python
import torch
print(f"PyTorch: {torch.__version__}")
print(f"CUDA available: {torch.cuda.is_available()}")
print(f"GPU: {torch.cuda.get_device_name(0)}")
print(f"Compute capability: {torch.cuda.get_device_capability(0)}")
# Actually test GPU computation
x = torch.randn(1000, 1000).cuda()
y = x @ x.T
print(f"GPU computation: OK")
```

Expected output (your specifics will differ):
```
PyTorch: 2.10.0+cu128
CUDA available: True
GPU: NVIDIA GeForce RTX 5090
Compute capability: (12, 0)
GPU computation: OK
```

**If you see `CUDA available: False`**, you installed CPU-only PyTorch. Go back and reinstall with `--index-url`.
**If you see a "sm_XXX is not compatible" warning**, your CUDA build is too old for your GPU. Step up (cu124 → cu126 → cu128).

---

## Part 6: Make It Permanent (~1 minute)

Add ROS 2 sourcing to your shell config so you don't have to type it every time:

```bash
# For bash users - add to ~/.bashrc
echo 'source /opt/ros/jazzy/setup.bash' >> ~/.bashrc
# For zsh users - add to ~/.zshrc
echo 'source /opt/ros/jazzy/setup.zsh' >> ~/.zshrc
```

---

## Reference (Bookmark This Part)

Everything below is designed to be referenced after your first read-through. Bookmark and come back when you need a quick lookup.

### Quick Reference Card

### Version Pairing Matrix

- **22.04** → Humble → Fortress (Gz 6) → Python 3.10
- **24.04** → Jazzy → Harmonic (Gz 8) → Python 3.12

### GPU + PyTorch Matrix

- **RTX 20xx** - Turing (sm_75) - cu102+
- **RTX 30xx** - Ampere (sm_86) - cu113+
- **RTX 40xx** - Ada (sm_89) - cu118+
- **RTX 50xx** - Blackwell (sm_120) - cu128

### Common Commands

```bash
# Source ROS 2 (do this in every terminal)
source /opt/ros/jazzy/setup.bash
# Launch Gazebo with a world file
gz sim your_world.sdf
# Check ROS 2 is running
ros2 topic list
# Build your ROS 2 package
colcon build --packages-select your_package
source install/setup.bash
# Check PyTorch GPU
python3 -c "import torch; print(torch.cuda.is_available(), torch.cuda.get_device_name(0))"
```

---

## Troubleshooting Cheat Sheet

**"Package ros-humble-desktop not found"**
You're on Ubuntu 24.04 trying to install Humble. Use Jazzy instead. Check your Ubuntu codename with `lsb_release -cs`.

**Gazebo black screen or crash**
Check `glxinfo | grep "OpenGL renderer"`. If it doesn't show your NVIDIA GPU, fix your drivers. If you're on Wayland, switch to X11 at the login screen.

**`import rclpy` fails in venv**
You created the venv without `--system-site-packages`. Delete it and recreate:
```bash
rm -rf venv && python3 -m venv --system-site-packages venv
```

**PyTorch CUDA not available**
Check `python3 -c "import torch; print(torch.__version__)"`. If the version doesn't end with `+cuXXX`, you installed CPU-only PyTorch. Reinstall with `--index-url`.

**"sm_XXX is not compatible" warning**
Your CUDA build doesn't include kernels for your GPU. Move to a newer build (cu124 → cu126 → cu128).

**PyYAML import errors after pip install**
Uninstall the pip version: `pip uninstall pyyaml`. ROS 2 already provides it.

---

## What This Gets You

Once everything above is working, you have a full robotics + ML development stack:

- **ROS 2 Jazzy** for robot middleware (topics, services, actions, launch files)
- **Gazebo Harmonic** for 3D physics simulation with GPU-accelerated rendering
- **Nav2** for autonomous navigation (path planning, obstacle avoidance, SLAM)
- **ros2_control** for robot joint control (arm manipulation, differential drive)
- **PyTorch + CUDA** for GPU-accelerated deep learning (YOLO object detection, LLM inference)
- **A clean Python venv** that sees both ROS 2 and pip packages without conflicts

This is the foundation for projects like autonomous mobile manipulation, sim-to-real transfer, or LLM-powered robot task planning.

---

## The Bottom Line

Version matching is the entire game. The actual install commands are trivial - it's knowing *which* versions to install that eats your day.

One more time:

```
Ubuntu version → ROS 2 distro → Gazebo version
GPU model → compute capability → PyTorch CUDA build
```

Get those two chains right and you're up in 30 minutes. Get them wrong and you're in dependency hell until 2 AM.

The approach here - checking official compatibility tables, using `pip index versions` to discover builds, verifying with smoke tests - works regardless of when you're reading this. New Ubuntu versions, new ROS distros, new GPUs will keep coming. The version-matching process stays the same.

---

**If this saved you time**, I'm writing a follow-up series on building an LLM-powered robot task planner with this stack - natural language commands to robot actions using ROS 2, Gazebo, YOLO, and GPT-4. Follow me to catch it.

**Hit a problem not covered here?** Drop a comment with your Ubuntu version, GPU, and the error message. I'll update the troubleshooting section.
