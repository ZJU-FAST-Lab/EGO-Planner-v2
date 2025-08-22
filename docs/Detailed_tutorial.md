
# Beginner-Friendly Reproduction Tutorial for EGO-Planner-v2

**Target audience:** absolute beginners who want a quick, minimal-effort way to run the EGO-Planner-v2 demo.  
**Estimated time (if all goes well):** ~10 minutes.

> **Note:** This is a simplified reproduction guide. It complements (but does not replace) the project's technical introduction: see [tutorial PDF file](docs/[README]_Brief_Documentation_for_Swarm_Playground.pdf).

---

## 0. Quick checklist (what you need)

- A PC with enough disk space and internet access.  
- VirtualBox (or any VM manager) and an Ubuntu **20.04** VM.  
- Basic comfort with opening a terminal and copy/paste.  
  - Linux terminal copy/paste: `Ctrl+Shift+C` / `Ctrl+Shift+V`.

---

## 1. Install Ubuntu (use 20.04)

- Create a VM (e.g., VirtualBox) and install **Ubuntu 20.04** (or 18.04 if you prefer).  
- **Important:** do **not** use Ubuntu 22.04 / 24.x or newer for this tutorial — ROS1 compatibility issues commonly occur on newer releases.

If you need an installation walkthrough, follow any beginner-friendly Ubuntu 20.04 tutorial (video or written).

---

## 2. Basic Ubuntu / terminal knowledge (very short)

You only need to know how to:

- Open a terminal.
- Change directories (`cd`).
- Copy and paste commands.(`ctrl+shift+c/v`)

- Run commands with `sudo` (it will ask for your password).

If you're totally new to Linux, watch a short terminal basics video or read a short intro.
<https://www.bilibili.com/video/BV1AP41157kT?spm_id_from=333.788.videopod.sections&vd_source=27a5e3cb94616aace08aaa99c932edfc> 

You do **not** need to be an expert.

---

## 3. Install ROS (ROS1)

This guide assumes using a helper script to install ROS1.

Open a terminal and run:

```bash
wget http://fishros.com/install -O fishros && . fishros
````

* Follow the script prompts to install **ROS1** (choose the ROS1 option when prompted).
* To verify ROS installation, run:

```bash
roscore
```

If `roscore` starts and prints version/info, ROS is installed successfully. Press `Ctrl+C` to stop it.

---

## 4. Install one dependency

Run:

```bash
sudo apt-get update
sudo apt-get install libarmadillo-dev
```

This installs the `libarmadillo-dev` library required by the project.

---

## 5. Clone the repository

In your terminal:

```bash
git clone https://github.com/ZJU-FAST-Lab/EGO-Planner-v2.git
```

If cloning fails due to network restrictions, you can download the repository ZIP from GitHub via the web UI and extract it locally.

---

## 6. Build the workspace

Navigate to the workspace used in this guide:

```bash
cd EGO-Planner-v2/swarm-playground/main_ws
```

> If your repository structure differs slightly, locate the folder that contains `src/` and open a terminal there.

Build with catkin:

```bash
catkin_make -j1
```

After the build completes, run:

```bash
source devel/setup.bash
```

---

## 7. Open RViz (visualization)

Start RViz to see the visualization environment:

```bash
roslaunch ego_planner rviz.launch
```

You should see the RViz GUI open. If nothing appears, check the terminal output for errors and confirm `source devel/setup.bash` was executed in the same terminal session.

---

## 8. Launch the swarm demo

Open a **new terminal** (you can keep RViz running):

```bash
cd EGO-Planner-v2/swarm-playground/main_ws
source devel/setup.bash
roslaunch ego_planner swarm.launch
```

`swarm.launch` should start the demo nodes and you will see simulated drones/paths in RViz.

---

## 9. Try other demos / launch files

There are multiple `.launch` files inside:

```
src/planner/plan_manage/launch/
```

To run another demo, replace `swarm.launch` with any other `.launch` file name, e.g.:

```bash
roslaunch ego_planner single_drone.launch
```

---

## 10. Troubleshooting tips

* **ROS commands not found:** ensure you ran `source devel/setup.bash` from your workspace. If you open a new terminal, source it again.
* **Build errors:** read the terminal log carefully. Missing packages often appear as error messages — install them with `apt` or `rosdep` if needed.
* **Network/git errors:** if `git clone` fails, try from a browser download or use a proxy network.
* **Different repo structure:** if the folder paths above don't exist in your cloned copy, search for `src/` and run `catkin_make` from its parent workspace.

---

## 11. Final notes

* This guide is intentionally short and pragmatic — it focuses on getting a working reproduction quickly.
* For deeper understanding of the code, algorithm, and design decisions, please refer to [tutorial PDF file](docs/[README]_Brief_Documentation_for_Swarm_Playground.pdf) and the source code.

Happy experimenting! 🚁



