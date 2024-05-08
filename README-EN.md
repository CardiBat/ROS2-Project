# ROS-on-RISCV

## Intro

ROS (acronym for Robot Operating System) represents a collection of software libraries and tools aimed at the creation of robotic applications. This system includes a wide range of components, from drivers to cutting-edge algorithms, to powerful development tools, providing everything needed for the development of sophisticated robotic projects. ROS 2 has been designed as the evolution of ROS 1, with the goal of addressing the continuously evolving needs in the field of robotics and automation, leveraging the strengths of ROS 1 and improving aspects that had limitations.


## First Steps

To begin understanding how ROS and its main components work, it's possible to use various terminal windows to later have them interact with each other. Before this, however, it's important to understand the various components that make up the core of this Operating System.

### Nodes

Nodes in ROS are fundamental components that allow the application to function correctly. Each node is independent from the others and performs a specific task assigned to it (via PYTHON code). To understand how these nodes operate, a demo package called 'demo_nodes_*' is available that contains both simple and more complex nodes.  
For example, there is a node named 'talker' that sends a 'Hello World' message every second; there is also a node called 'listener' that listens for incoming messages. There are other more complex examples, like 'turtlesim', which represents a static turtle in a space, and the corresponding node 'turtle_teleop_key' that sends movement commands to the turtle from keyboard input.  
This set of nodes, of course, can be custom programmed as needed, perhaps starting from a demo node and modifying it to one's liking. It also follows that these nodes cannot communicate with each other unless a certain communication policy is adopted, which, as we will see later, is called 'Topic'.

### Topics

Knowing then that each node is responsible for a particular action, it's important to clarify how topics, which enable communication between them, work. When terminal windows are opened and a node is assigned to each one, their interaction typically follows the 'pub-sub' message exchange paradigm where one or more nodes act as publishers and an equal number as subscribers. Therefore, topics are merely the representation of this communication.

<p>&nbsp;</p>

![Turtlesim-topic](/turtlesim.png)

<p>&nbsp;</p>

As can be observed, in this chart (generated via rqt_graph with group 0 and Nodes/Topics view), there are the two nodes already mentioned (the turtle and its controller) and between them are the various communication topics. In particular, the controller communicates keyboard inputs through the 'cmd_vel' topic, while the turtle object communicates feedback and its current state through two distinct channels.  
More specifically, the control node acts as a publisher of keyboard inputs, while the turtlesim node subscribes to this topic. Similarly, turtlesim operates by publishing feedback and status updates while the controller subscribes to these.

### Services

Services in ROS meet the need of the client-server architecture, which is not native to topics as they were created. Indeed, a node through a classic topic sends information every x time but does not expect a response from the other side (as already mentioned, it follows the pub-sub paradigm).  
Through services, however, it is possible to implement what typically characterizes web interaction, where there are requests and responses, servers that expose services, and clients that use them.

<p>&nbsp;</p>
<div align="center">
  <img src="/services.png" alt="services.png">
</div>
<p>&nbsp;</p>

In the above image, there is a representation of the client-server architecture where one client (but there could be more) sends requests and receives responses through the service.  
An example of how this can be implemented can be observed again through the demo package using on one terminal add_two_ints_server (Server node) and in another window the keyword 'call' adding the parameters a and b. By doing so, a request is sent to the server which will respond with the result.


## Compilation and Installation

Normally, installation (on Ubuntu) involves a simple terminal command and the installation is automatically completed. In our case, however, we will attempt to manually compile the set of files to streamline the installation as much as possible and facilitate its porting to a RISC-V architecture (despite ROS being designed for Intel architectures)  
We will start with an inspection on a virtual environment that allows (in case of irreversible errors) to not damage the system and consequently to try various compilation combinations. This step is essential to understand how to move agilely when we then shift to RISC-V.

### Installing a virtual environment

Before proceeding, check that your machine is compatible with integrated UNIX virtualization. Then run the following command:

```sh
$ egrep -c '(vmx|svm)' /proc/cpuinfo
```

If the command returns a number greater than 0, it means that the processor supports virtualization. If it returns 0, you may need to enable virtualization in the BIOS.  
At this point, you can proceed with the installation of KVM:

```sh
sudo apt update
```
```sh
sudo apt install qemu-kvm libvirt-daemon-system libvirt-clients bridge-utils virt-manager
```

It's also necessary to add oneself to the appropriate groups to avoid issues:

```sh
sudo adduser `id -un` libvirtrosinstall_generator osrf_testing_tools_cpp --rosdistro foxy --deps --tar >

 foxy-custom.rosinstall

```
```sh
sudo adduser `id -un` kvm
```

After rebooting the machine to apply the changes, you can proceed to launch KVM and create an Ubuntu 20.04 VM necessary for ROS2 Foxy, with 4GB of RAM and 4 Cores.  
Then run the following command:

```sh
virt-manager
```

and install the operating system from an ISO file downloaded from the official Ubuntu site.

### Installation of Libraries and Dependencies Required by ROS2 Foxy

It is advisable to install the compilers and Python libraries required, as well as Python itself if it is not already installed on your machine. Execute the following commands:

```sh
sudo apt update
```
```sh
sudo apt install build-essential cmake git python3-rosdep python3-rosinstall-generator python3-wstool python3-rosinstall
```

In case Python is not installed, execute:

```sh
sudo apt update
```
```sh
sudo apt install python3 python3-pip
```

Finally, install the colcon compiler:
```sh
sudo apt update
```
```sh
pip3 install -U colcon-common-extensions
```

Then permanently set the environment variable. Open the shell:
```sh
nano ~/.bashrc
```
Add the following line at the end of the file, save it, and reopen the terminal:
```sh
export PATH="$PATH:$HOME/.local/bin"
```


### Downloading ROS Source Code

Initialize `rosdep` which helps to install system dependencies for the sources to be compiled.

```sh
sudo rosdep init
```
```sh
rosdep update
```

### Creation of the Catkin Workspace
To start configuring the ROS environment, it is necessary to create a Catkin workspace. This workspace will serve as a container for ROS sources and packages. Creation can be done with the following commands:

```sh
mkdir -p ~/ros2_foxy_ws/src
```
```sh
cd ~/ros2_foxy_ws
```

### Selection of Packages
Using `rosinstall_generator` allows you to select specific parts of ROS to put in the .rosinstall configuration file. You can opt to include essential components like `ros_comm` or a more complete version with `desktop-full`. In any case, you need to replace PKGNAME with the packages you want. Generally:

```sh
rosinstall_generator [PKGNAME] --rosdistro foxy --deps --tar > foxy-desktop.rosinstall
```
An example might be to consider packages like rclcpp for programming nodes in C++, rclpy for programming nodes in Python, and example_interfaces which provides examples of service interfaces and messages that could be used for experiments and learning.

```sh
rosinstall_generator rclcpp rclpy example_interfaces --rosdistro foxy --deps --tar > foxy-desktop.rosinstall
```

Next, you will download the packages listed in the newly created configuration file:

```sh
wstool init -j8 src foxy-desktop.rosinstall
```

### Compilation of ROS
It is crucial to ensure that all dependencies of the selected packages are met before compilation. This is possible through the use of `rosdep`:

```sh
rosdep install --from-paths src --ignore-src --rosdistro foxy -y
```

Using `colcon`, the recommended build tool for ROS 2, compile the downloaded packages with their dependencies:

```sh
colcon build --symlink-install
```


### Uninstallation and Restoration Process

To remove ROS 2 components, simply remove the compilation workspace, which typically includes the `build`, `install`, and `log` directories. Assuming that the workspace is located in `~/ros2_foxy_ws`, execute the following commands:

```sh
cd ~/ros2_foxy_ws
```
```sh
rm -rf build install log
```
This will remove all directories related to the compilation and installation of ROS 2 packages.

To reconfigure the development environment from a clean state, it is also advisable to remove any residual ROS 2 references in the shell configuration file, such as `.bashrc` or `.zshrc`, depending on the shell in use. These references might include adding the ROS 2 installation path to the `PATH`, as well as sourcing setup scripts.

Open the shell configuration file with a text editor:

```sh
nano ~/.bashrc  # or use ~/.zshrc for zsh
```

Search for and remove lines associated with the ROS 2 environment, which might appear as:

```sh
source /opt/ros/foxy/setup.bash
```
```sh
source ~/ros2_foxy_ws/install/local_setup.bash
```

After making changes, save the file and restart the terminal to ensure the changes take effect. This step ensures that the terminal environment is free of any ROS-related configuration, allowing for a new installation or the setup of a different environment cleanly and without conflicts.


## Toward Porting on RISC-V: Using the Monte Cimone Cluster

After understanding the workings of ROS on an Ubuntu machine, we will now attempt to adapt these steps to the target machine, in this case based on RISC-V. We will encounter several issues since it is not natively supported, requiring various compilation-level interventions. Therefore, it will be necessary to skip non-essential steps and delve into low-level operations if commands like rosdep or colcon might not function.

Before starting, it's useful to learn how to navigate the RISC-V machine named Monte-Cimone easily. The next steps will clarify the main operations as much as possible.

### Connection and Login

To start practicing with a RISC-type processor, a cluster with various nodes named Monte Cimone is available. This machine is accessible from your PC (after registration via a form) using the SSH command:

```sh
ssh username@beta.dei.unibo.it -p 2223
```
> output: username@mcimone-login:~$ 

After entering the password provided by the instructor, your username on the terminal will change and the `nodeinfo` command (which can also be called later for real-time information from the login) will be displayed. The screen will then show all nodes (available _if and only if_ they are in the `IDLE` state):  

<p>&nbsp;</p>
<div align="center">
  <img src="/nodes.png" alt="nodes.png">
</div>
<p>&nbsp;</p>

IMPORTANT NOTE: The login is based on Intel architecture (and therefore CISC). It will then be appropriate to proceed by connecting to a node to actually reach the RISC-V architecture.

### Brief Description of Nodes and Their Use

The available nodes are of two types depending on the partition. There are `sifive-nodes`, each consisting of 4 cores, while there are `milkv-nodes` which have up to 64 cores each. The entire system is installed with `Slurm`, an open-source job scheduler capable of organizing the start of processes simply.

To quickly enter a node and start a job, simply type the following command (with nx = number of cores):

```sh
srun -n4 --pty bash
```

This command is useful for using the first available node, but if you want to try using a `milkv`, then you must follow the complete process, namely allocation, SSH, potential use, and finally deallocation.
To allocate a node, use the `salloc` command followed by the number of cores and the partition selector with its name.

```sh
salloc -n64 -p mcimone-milkvs
```

If you don't know the names of the partitions, it is sufficient to type the `sinfo` command to get the list. In any case, after this operation, you can find out which node of the partition has been allocated with `nodeinfo` and then execute the command:

```sh
ssh fsimoni@mcimone-milkv-1
```

Thus managing to enter via Secure Shell into the node. From here, it is possible to reuse `srun` to start a job. To deallocate, instead, type the `squeue` command to find out the PID of the allocated node and then the cancellation command followed by the latter.

```sh
scancel [PID]
```

For completeness, here is the general usage of these two commands:

```sh
salloc -n <> -t <hours:minutes:seconds> [-p <>] [-w <>] [--exclusive]
```

```sh
srun -n <n_task_to_allocate> [-N <n_nodes_to_run> ] [-p <partitions>] [-w <specific_node>] [-t <hours:minutes:seconds>] [--pty] command
```

We will use node-4 and therefore run the same task on 4 cores:
```sh
srun -n4 -c1 -w mcimone-node-4 --pty bash
```

For a more detailed guide, look at the [specific CIMONE guide](https://gitlab.com/ecs-lab/courses/lab-of-big-data/riscv-hpc-perf/-/blob/main/2_slurm.md?ref_type=heads) or the [official documentation](https://slurm.schedmd.com/overview.html)

### Inspection of Basic Dependencies on the RISC-V Machine

Summarizing, the dependencies necessary for ROS2 are:

- build-essential
- cmake
- git
- python3
- python3-pip  
- python3-rosdep
- python3-rosinstall-generator
- python3-wstool
- python3-rosinstall  

You must therefore check for their presence on the RISC-V machine, possibly install them, and at this point, you can proceed with the porting.
_Important note_: You

 do not have root privileges to install programs on the machine you are using, so any programs must be installed in your own Desktop directory from _Source_ and you must then update the PATH environment variable for each of them (or you can ask the administrator in some cases).

Run the following command:

```sh
dpkg -l | grep <package_name>
```

Replacing `<package_name>` with python3, git, cmake, and build-essential. Noting the absence of rosdep, rosinstall-generator, wstool, and rosinstall, these will need to be installed in the current directory and added to the PATH.

Installation:

```sh
pip3 install --user rosdep rosinstall-generator wstool rosinstall
```
Where `--user` specifies the local directory and not the system

Setting PATH:

From HOME (~) run `nano .bashrc` and add the following line at the bottom of the file, which looks in the local directories for packages:

```sh
export PATH="$HOME/.local/bin:$PATH"
```

Save, close nano and reload bashrc with the following command:

```sh
source ~/.bashrc
```
After a general check with `--version` options, you will notice that everything has been fully installed.

## ROS Compilation Tests on RISC-V

We have reached the heart of the project: the porting of an operating system not compatible with RISC. It will be necessary to move cautiously, paying attention to installations, conflicts, and dependencies. Additionally, we must be able to find alternatives if some solutions do not work.

Firstly, for the compilation of ROS, `colcon` is normally used although it is not certain that it will work on RISC-V. We will still try to use it by installing it with this command (always in the user's directory):

```sh
pip3 install --user -U colcon-common-extensions
```

At this point, create the workspace as already seen virtually on CISC:

```sh
mkdir -p ~/ros2_foxy_ws/src
```
```sh
cd ~/ros2_foxy_ws
```

### [FAIL] Test Compilation on Main Packages and Exposure of Problems

We will then try to install two test packages for simple example nodes and customization only in Python3.

Configuration file:

```sh
rosinstall_generator rclpy example_interfaces --rosdistro foxy --deps --tar > foxy-custom.rosinstall
```

Downloading packages:

```sh
wstool init -j8 src foxy-custom.rosinstall
```

Now you must proceed with installing dependencies which may cause problems without root privileges. Although rosdep provides the `--as-root=FALSE` flag, it will not work in our case. We must therefore manually search and install using pip --user (as done previously) to solve the root problems. Move to the src of the workspace and run the following commands:

```sh
cat rclpy/package.xml
cat example_interfaces/package.xml
```

and look for `depend` in each of them to identify the dependencies.

Dependencies of rclpy:

- ament_cmake
- python_cmake_module
- rcutils
- rmw_implementation_cmake
- ament_index_python
- builtin_interfaces
- rcl
- rcl_action
- rcl_interfaces
- rcl_yaml_param_parser
- rosgraph_msgs
- rpyutils
- rmw_implementation
- unique_identifier_msgs

Dependencies of example_interfaces:

- ament_cmake
- rosidl_default_generators
- action_msgs
- rosidl_default_runtime

To understand exactly which dependencies are missing, use rosdep which however only works with root privileges since it searches all folders, including admin ones. To prevent it from searching folders it does not have permission for, use this series of commands:

```sh
mkdir -p $HOME/.ros/rosdep/sources.list.d
echo 'export ROSDEP_SOURCE_PATH="$HOME/.ros/rosdep/sources.list.d"' >> ~/.bashrc
source ~/.bashrc
```

and then start rosdep:
```sh
rosdep init
rosdep update
```
at this point, it can be used to resolve dependencies:

```sh
rosdep install --from-paths src --ignore-src --rosdistro foxy -y
```

But now there will be a problem with RISC-V, namely attempting compilation:
```
colcon build --symlink-install
```

The output indeed tells us:

```
18 packages finished [1min 37s]: 
- 1 package failed: osrf_testing_tools_cpp
- 3 packages aborted: ament_cmake_export_include_directories ament_lint_cmake ament_xmllint
- 10 packages had stderr output: ament_cmake_test ament_copyright ament_cppcheck ament_flake8 ament_lint ament_lint_cmake ament_package ament_pep257

 fastcdr osrf_testing_tools_cpp
- 86 packages not processed
```

The same situation also arises when manually cloning each dependency through the git command (with token).

```sh
git clone https://<token>@github.com/ros2/<package>.git
```

```sh
cd rmw_implementation_cmake
colcon build --packages-select rmw_implementation_cmake
cd ..
```

We thus remain stuck in this situation because even downloading only rlcpy (which always has more than 100 dependencies) the compilation stops and manually compiling the packages one by one does not resolve the dependencies either. Any test node depends on rlcpp or rlcpy so it is not possible to lighten the load of packages which always hovers around 100.

### Possible Solutions and Choosing the Most Effective

There are three possible solutions to these problems:

1. **Use of Precompiled Packages**: This option is generally less engaging because it doesn't allow us to learn about and solve the compilation issues firsthand.
   
2. **Avoid rlcpy/rlcpp by using simple message exchange packages like HTTP or SOCKET, thus IPC (Internal Process Communication)**: This could sidestep the problem but doesn't address our main goal of working directly with ROS core functionalities.
   
3. **Manually changing the compiler's approach for each package that fails**: This option allows us to deeply understand and possibly correct specific issues related to the architecture or compiler settings. This is the most informative approach and aligns with our objective of understanding how to compile any package, avoiding intrinsic architectural errors that might just be simple checks and not actual malfunctions.

We will proceed with the third option, going low-level by modifying the CMAKE files to force compilation wherever possible.

## Modifying CMAKE Behavior at Compile Time

We will attempt to install a fundamental ROS package: `rclcpp`, which enables the creation of C++ nodes and is a critical part of this OS.

To install this package, execute:

```sh
rosinstall_generator rclcpp --rosdistro foxy --deps --tar > foxy-custom.rosinstall
```
```sh
wstool init -j8 src foxy-custom.rosinstall
```
```sh
rosdep install --from-paths src --ignore-src --rosdistro foxy -y
```
```sh
colcon build --symlink-install
```

This will result in an error as demonstrated in the previous [FAIL] test, at the 18th package.

### Ad Hoc Compilation of Each Package with Possible Modifications

We will try to compile `osrf_testing_tools_cpp`, one of the packages that fails and causes others to abort in cascade. Running:

```sh
colcon build --packages-select osrf_testing_tools_cpp
```  

Encounters an error like this:

```
Starting >>> osrf_testing_tools_cpp
[Processing: osrf_testing_tools_cpp]                              
--- stderr: osrf_testing_tools_cpp                               
In file included from /home/fsimoni/ros2_foxy_ws/src/osrf_testing_tools_cpp/src/memory_tools/./print_backtrace.hpp:26,
                 from /home/fsimoni/ros2_foxy_ws/src/osrf_testing_tools_cpp/src/memory_tools/custom_memory_functions.cpp:30:
/home/fsimoni/ros2_foxy_ws/src/osrf_testing_tools_cpp/src/memory_tools/././vendor/bombela/backward-cpp/backward.hpp:3944:2: error: #warning is a GCC extension [-Werror]
 3944 | #warning ":/ sorry, ain't know no nothing none not of your architecture!"
      |  ^~~~~~~
/home/fsimoni/ros2_foxy_ws/src/osrf_testing_tools_cpp/src/memory_tools/././vendor/bombela/backward-cpp/backward.hpp:3944:2: error: #warning ":/ sorry, ain't know no nothing none not of your architecture!" [-Werror=cpp]
In file included from /home/fsimoni/ros2_foxy_ws/src/osrf_testing_tools_cpp/src/memory_tools/./stack_trace_impl.hpp:31,
                 from /home/fsimoni/ros2_foxy_ws/src/osrf_testing_tools_cpp/src/memory_tools/memory_tools_service.cpp:20:
/home/fsimoni/ros2_foxy_ws/src/osrf_testing_tools_cpp/src/memory_tools/././vendor/bombela/backward-cpp/backward.hpp:3944:2: error: #warning is a GCC extension [-Werror]
 3944 | #warning ":/ sorry, ain't know no nothing none not of your architecture!"
      |  ^~~~~~~
/home/fsimoni/ros2_foxy_ws/src/osrf_testing_tools_cpp/src/memory_tools/././vendor/bombela/backward-cpp/backward.hpp:3944:2: error: #warning ":/ sorry, ain't know no nothing none not of your architecture!" [-Werror=cpp]
/home/fsimoni/ros2_foxy_ws/src/osrf_testing_tools_cpp/src/memory_tools/././vendor/bombela/backward-cpp/backward.hpp: In static member function ‘static void backward::SignalHandling::handleSignal(int, siginfo_t*, void*)’:
/home/fsimoni/ros2_foxy_ws/src/osrf_testing_tools_cpp/src/memory_tools/././vendor/bombela/backward-cpp/backward.hpp:3920:17: error: unused variable ‘uctx’ [-Werror=unused-variable]
 3920 |     ucontext_t *uctx = static_cast<ucontext_t *>(_ctx);
      |                 ^~~~
cc1plus: all warnings being treated as errors
gmake[2]: *** [src/memory_tools/CMakeFiles/memory_tools.dir/build.make:76: src/memory_tools/C

MakeFiles/memory_tools.dir/custom_memory_functions.cpp.o] Error 1
gmake[2]: *** Waiting for unfinished jobs....
/home/fsimoni/ros2_foxy_ws/src/osrf_testing_tools_cpp/src/memory_tools/././vendor/bombela/backward-cpp/backward.hpp: In static member function ‘static void backward::SignalHandling::handleSignal(int, siginfo_t*, void*)’:
/home/fsimoni/ros2_foxy_ws/src/osrf_testing_tools_cpp/src/memory_tools/././vendor/bombela/backward-cpp/backward.hpp:3920:17: error: unused variable ‘uctx’ [-Werror=unused-variable]
 3920 |     ucontext_t *uctx = static_cast<ucontext_t *>(_ctx);
      |                 ^~~~
In file included from /home/fsimoni/ros2_foxy_ws/build/osrf_testing_tools_cpp/googletest-1.10.0.1-extracted/googletest-1.10.0.1-src/googletest/src/gtest-all.cc:42:
/home/fsimoni/ros2_foxy_ws/build/osrf_testing_tools_cpp/googletest-1.10.0.1-extracted/googletest-1.10.0.1-src/googletest/src/gtest-death-test.cc: In function ‘bool testing::internal::StackGrowsDown()’:
/home/fsimoni/ros2_foxy_ws/build/osrf_testing_tools_cpp/googletest-1.10.0.1-extracted/googletest-1.10.0.1-src/googletest/src/gtest-death-test.cc:1301:24: error: ‘dummy’ may be used uninitialized [-Werror=maybe-uninitialized]
 1301 |   StackLowerThanAddress(&dummy, &result);
      |   ~~~~~~~~~~~~~~~~~~~~~^~~~~~~~~~~~~~~~~
/home/fsimoni/ros2_foxy_ws/build/osrf_testing_tools_cpp/googletest-1.10.0.1-extracted/googletest-1.10.0.1-src/googletest/src/gtest-death-test.cc:1290:13: note: by argument 1 of type ‘const void*’ to ‘void testing::internal::StackLowerThanAddress(const void*, bool*)’ declared here
 1290 | static void StackLowerThanAddress(const void* ptr, bool* result) {
      |             ^~~~~~~~~~~~~~~~~~~~~
/home/fsimoni/ros2_foxy_ws/build/osrf_testing_tools_cpp/googletest-1.10.0.1-extracted/googletest-1.10.0.1-src/googletest/src/gtest-death-test.cc:1299:7: note: ‘dummy’ declared here
 1299 |   int dummy;
      |       ^~~~~
cc1plus: all warnings being treated as errors
gmake[2]: *** [src/memory_tools/CMakeFiles/memory_tools.dir/build.make:132: src/memory_tools/CMakeFiles/memory_tools.dir/memory_tools_service.cpp.o] Error 1
gmake[1]: *** [CMakeFiles/Makefile2:1012: src/memory_tools/CMakeFiles/memory_tools.dir/all] Error 2
gmake[1]: *** Waiting for unfinished jobs....
cc1plus: all warnings being treated as errors
gmake[2]: *** [googletest-1.10.0.1-extracted/googletest-1.10.0.1-build/googletest/CMakeFiles/gtest.dir/build.make:76: googletest-1.10.0.1-extracted/googletest-1.10.0.1-build/googletest/CMakeFiles/gtest.dir/src/gtest-all.cc.o] Error 1
gmake[1]: *** [CMakeFiles/Makefile2:1143: googletest-1.10.0.1-extracted/googletest-1.10.0.1-build/googletest/CMakeFiles/gtest.dir/all] Error 2
gmake: *** [Makefile:146: all] Error 2
---
Failed   <<< osrf_testing_tools_cpp [47.4s, exited with code 2]

Summary: 0 packages finished [50.0s]
  1 package failed: osrf_testing_tools_cpp
  1 package had stderr output: osrf_testing_tools_cpp
```

We then proceed to ignore warnings since they are not of interest and suppress architecture-related warnings from the text. Note that these errors are not fatal but merely warnings treated as errors on RISC-V. _We will do the same for the following packages, being careful not to compromise them_:

```sh
nano src/osrf_testing_tools_cpp/src/memory_tools/vendor/bombela/backward-cpp/backward.hpp
```

Remove `#warning ":/ sorry, ain't know no nothing none not of your architecture!"` and run:



```sh
colcon build --cmake-args -DCMAKE_CXX_FLAGS="-Wno-error -Wno-unused-variable -Wno-maybe-uninitialized -Wno-error=cpp -Wno-error=pedantic"
```

Moving forward, unfortunately, there are other types of errors for many packages, all due to the lack of a 'benchmark' package.

```
CMake Error at CMakeLists.txt:20 (find_package):
  By not providing "Findbenchmark.cmake" in CMAKE_MODULE_PATH this project
  has asked CMake to find a package configuration file provided by
  "benchmark", but CMake did not find one.

  Could not find a package configuration file provided by "benchmark" with
  any of the following names:

    benchmarkConfig.cmake
    benchmark-config.cmake

  Add the installation prefix of "benchmark" to CMAKE_PREFIX_PATH or set
  "benchmark_DIR" to a directory containing one of the above files.  If
  "benchmark" provides a separate development package or SDK, be sure it has
  been installed.


---
Failed   <<< performance_test_fixture [6.56s, exited with code 1]
Aborted  <<< rosidl_generator_dds_idl [7.96s]                                                                                                                                      
Aborted  <<< rosidl_typesupport_interface [1min 50s]                                                                                         
Aborted  <<< fastrtps [16min 1s]                                             

Summary: 60 packages finished [20min 13s]
  1 package failed: performance_test_fixture
  3 packages aborted: fastrtps rosidl_generator_dds_idl rosidl_typesupport_interface
  23 packages had stderr output: ament_cmake ament_cmake_cppcheck ament_cmake_cpplint ament_cmake_flake8 ament_cmake_gmock ament_cmake_google_benchmark ament_cmake_gtest ament_cmake_pep257 ament_cmake_pytest ament_cmake_ros ament_cmake_uncrustify ament_cmake_xmllint ament_lint_auto ament_lint_common fastrtps foonathan_memory_vendor performance_test_fixture python_cmake_module rosidl_adapter rosidl_cmake rosidl_generator_dds_idl rosidl_parser test_interface_files
  44 packages not processed
```

So we follow the suggestion to "Add the installation prefix of 'benchmark' to CMAKE_PREFIX_PATH", remembering that we do not have root privileges.

Clone the repository locally to avoid using the `sudo` command:
```sh
git clone https://github.com/google/benchmark.git
```
```sh
cd benchmark
```
```sh
mkdir build
```
```sh
cd build
```

Finally, disable Google's tests to avoid unexpected subsequent errors:
```sh
cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=$HOME/local -DBENCHMARK_ENABLE_TESTING=OFF ..
```
And proceed with the installation of `benchmark`:
```sh
make
```
```sh
make install
```

However, at this point, a significant error occurs during the compilation (caused by the `mimick` package):
```
CMake Error at CMakeLists.txt:88 (message):
  Architecture 'riscv64' is not supported.
```

Fortunately, this package is not essential for the construction of nodes but rather in the realm of simulation, and thus, it can be ignored by modifying the compilation command (used henceforth for non-essential packages that give this type of error):

```sh
colcon build --packages-skip mimick_vendor
```

Naturally, there are other packages that depend on `mimick_vendor`, and for each, one must enter its configuration file CMakeLists.txt and ignore the dependency. Here is an example for rcutils:

```sh
cd src/rcutils && nano CMakeLists.txt
```

And comment out the line `find_package(mimick_vendor REQUIRED)` using `#`.  

Unfortunately, however, the dependencies of this package appear in many files that use it for testing. After careful research, the command to avoid this becomes:

```sh
colcon build --packages-skip mimick_vendor --cmake-args -DBUILD_TESTING=OFF
```

Indeed, this command allows us to avoid all the other test packages that are not very interesting in our case and could lead to a significant waste of time.

At this point, the compilation proceeds to the final installation package rclcpp but then fails in this way:

```
[Processing: rclcpp]                                             
--- stderr: rclcpp                                              
/home/fsimoni/ros2_foxy_ws/src/rclcpp/rclcpp/src/rclcpp/executor.cpp: In member function ‘bool rclcpp::Executor::get_next_ready_executable(rclcpp::AnyExecutable&)’:
/home/fsimoni/ros2_foxy_ws/src/rclcpp/rclcpp/src/rclcpp/executor.cpp:615:67: warning: ‘using CallbackGroupType = enum class rclcpp::CallbackGroupType’ is deprecated: use rclcpp::CallbackGroupType instead [-Wdeprecated-declarations]
  615 |       any_executable.callback_group->type() == CallbackGroupType::MutuallyExclusive)
      |                                                                   ^~~~~~~~~~~~~~~~~
In file included from /home/fsimoni/ros2_foxy_ws/src/rclcpp/rclcpp/include/rclcpp/any_executable.hpp:20,
                 from /home/fsimoni/ros2_foxy_ws/src/rclcpp/rclcpp/include/rclcpp/memory_strategy.hpp:24,
                 from /home/fsimoni/ros2_foxy_ws/src/rclcpp/rclcpp/include/rclcpp/memory_strategies.hpp:18,
                 from /home/fsimoni/ros2_foxy_ws/src/rclcpp/rclcpp/include/rclcpp/executor_options.hpp:20,
                 from /home/fsimoni/ros2_foxy_ws/src/rclcpp/rclcpp/include/rclcpp/executor.hpp:33,
                 from /home/fsimoni/ros2_foxy_ws/src/rclcpp/rclcpp/src/rclcpp/executor.cpp:24:
/home/fsimoni/ros2_foxy_ws/src/rclcpp/rclcpp/include/rclcpp/callback_group.hpp:165:7: note: declared here
  165 | using CallbackGroupType [[deprecated("use rclcpp::CallbackGroupType instead")]] = CallbackGroupType;
      |       ^~~~~~~~~~~~~~~~~
/home/fsimoni/ros2_foxy_ws/src/rclcpp/rclcpp/src/rclcpp/parameter_value.cpp: In instantiation of ‘std::string array_to_string(const std::vector<ValType>&, std::ios_base::fmtflags) [with ValType = std::__cxx11::basic_string<char>; PrintType = std::__cxx11::basic_string<char>; std::string = std::__cxx11::basic_string<char>; std::ios_base::fmtflags = std::ios_base::fmtflags]’:
/home/fsimoni/ros2_foxy_ws/src/rclcpp/rclcpp/src/rclcpp/parameter_value.cpp:105:29:   required from here
/home/fsimoni/ros2_foxy_ws/src/rclcpp/rclcpp/src/rclcpp/parameter_value.cpp:70:22: warning: loop variable ‘value’ creates a copy from type ‘const std::__cxx11::basic_string<char>’ [-Wrange-loop-construct]
   70 |   for (const ValType value : array) {
      |                      ^~~~~
/home/fsimoni/ros2_foxy_ws/src/rclcpp/rclcpp/src/rclcpp/parameter_value.cpp:70:22: note: use reference type to prevent copying
   70 |   for (const ValType value : array) {
      |                      ^~~~~
      |                      &
---
Finished <<< rclcpp [5min 5s]
                               
Summary: 107 packages finished [14min 2s]
  1 package had stderr output: rclcpp


```

Examining the stderr, we notice that we have two warnings:
- A warning related to a deprecated declaration that should be updated, but it is not a critical problem;
- A warning about a while loop that decreases performance, but this is also not of interest to us.
As a result, the package has been completely compiled and we can proceed with the demo on the nodes.

## Creating Nodes through ROS Installed on RISC-V

We have thus managed to install the `rclcpp` library almost entirely along with all its dependencies. We must now try to create working C++ nodes to demonstrate its functionality.

### [Optional] Recursive Addition of Variables to the PATH for Each Package

Since easy-to-use commands like `ros2` are not available and we must proceed manually, it is possible to simplify the work by adding all the environmental variables to avoid specifying paths during the compilation of nodes that will be created with `rclcpp`:

```sh
export CPLUS_INCLUDE_PATH=$(find /home/fsimoni/ros2_foxy_ws/install -type d -name include | paste -sd ":" -):${CPLUS_INCLUDE_PATH}

export LD_LIBRARY_PATH=$(find /home/fsimoni/ros2_foxy_ws/install -type d -name lib | paste -sd ":" -):${LD_LIBRARY_PATH}
```

### Creation of Example Nodes

To create nodes in C++, it is necessary to create a package that contains the file to be compiled, which calls the libraries dependent on `rclcpp` as needed, already installed in the previous chapter. We will do two tests by creating two different packages, one simple and one more complex. For each, a node will be created and executed.

Let's move to src and create our package:

```sh
cd /path/to/ros2_foxy_ws/src
mkdir my_package
cd my_package
```

Create a main.cpp file from which we will launch the node:
```sh
nano main.cpp
```

Write a simple test one, made in this way:


```
#include "rclcpp/rclcpp.hpp"

class MyNode : public rclcpp::Node
{
public:
  MyNode() : Node("my_node") {
    RCLCPP_INFO(this->get_logger(), "Hello from my_node!");
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MyNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
```

Save it and compile it individually, from the root of the project:

```sh
g++ -o my_node src/my_package/main.cpp \
-I/home/fsimoni/ros2_foxy_ws/install/rclcpp/include \
-I/home/fsimoni/ros2_foxy_ws/install/rcutils/include \
/home/fsimoni/ros2_foxy_ws/install/rclcpp/lib/librclcpp.so \
/home/fsimoni/ros2_foxy_ws/install/rcutils/lib/librcutils.so \
-lstdc++fs -pthread
```

Note that the libraries not added to the PATH were specified at compile time to avoid confusion. Indeed, without these specifications, there would be a series of errors. After extensive research, we have thus understood how to specify them.

An executable named my_node will then be created. Run it in the traditional way:

```sh
./my_node
```
The node works and prints to the terminal:

![image](https://github.com/CardiBat/ROS2-Project/assets/102695322/10902e5a-8931-4e3d-a73d-ae099687105f)

For safety, let's perform a backup:

```sh
scp -r fsimoni@mcimone-node-4:/home/fsimoni/ros2_foxy_ws /home/cardigun/Desktop/backup-riscv
```

Let's create another example node. So we create a `my_package_2` in `src` and inside I create another node composed as follows:
```
#include "rclcpp/rclcpp.hpp"

class MyNode2 : public rclcpp::Node
{
public:
  MyNode2() : Node("my_node_2") {
    // Do something different here
    timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&MyNode2::timerCallback, this));
    RCLCPP_INFO(this->get_logger(), "Hello from my_node_2!");
  }

private:
  void timerCallback() {
    // Perform some action periodically
    RCLCPP_INFO(this->get_logger(), "Doing something different...");
  }

  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MyNode2>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

```

We then compile in a similar way but adding the necessary libraries:

```sh
g++ -o my_node_2 src/my_package_2/main.cpp \
-I/home/fsimoni/ros2_foxy_ws/install/rclcpp/include \
-I/home/fsimoni/ros2_foxy_ws/install/rcutils/include \
/home/fsimoni/ros2_foxy_ws/install/rclcpp/lib/librclcpp.so \
/home/fsimoni/ros2_foxy_ws/install/rcutils/lib/librcutils.so \
/home/fsimoni/ros2_foxy_ws/install/rcl/lib/librcl.so \
/home/fsimoni/ros2_foxy_ws/install/tracetools/lib/libtracetools.so \
-lstdc++fs -pthread
```

and run:

```sh
./my_node_2
```

The output then displays a node capable of listening for potential sensor signals coming from outside. It could therefore be able to execute a `handler` if an `interrupt` occurs:
![image](https://github.com/CardiBat/ROS2-Project/assets/102695322/7c671563-f673-40ef-970f-37fd4a219abd)

[NOTE]:
The reason we add various paths at each compilation is because we have not installed ROS2 on CISC, and thus classic commands such as:

```sh
ros2 run my_package_2 my_node_2
```

are not available. In fact, `ros2` would not be found. Therefore, I specify at compilation where to find the compiled libraries and other necessary external ones so that I encounter no issues if I use `./exec` for running, which is native for g++. 


## Conclusions

### Porting Objective Achieved

During this project, we delved into and navigated through the complex experience of porting ROS2 onto a RISC-V architecture, uncovering technical challenges and exploring various possible solutions. Throughout this journey, we managed to overcome several obstacles by adapting ROS to an unsupported environment, demonstrating its feasibility.

After explaining how ROS operates and analyzing how it is used on Ubuntu, we dived into RISC-V and attempted to compile a package for node creation. The outcome was successful, and we were able to create fully functional example nodes.

Our experience highlights the importance of a solid understanding of the fundamentals of ROS 2 and low-level programming practices for successfully navigating its ecosystem, especially when facing challenges related to specific hardware and architectures.

We hope that this document will serve not only as a tutorial on how to install ROS on RISC-V but also on how to approach any CISC -> RISC porting. Indeed, here one can observe all the strategies that can be applied, such as avoiding non-essential portions during installation, making ad hoc compiler modifications, resolving dependencies step-by-step, and so on.

### Future Developments

The project thus showcases some examples of node execution. In the future, the functionalities of these nodes could obviously be enhanced by, for example, adding other libraries making it usable in a real automated hardware system such as robots or by connecting to physical sensors on boards.

Furthermore, these steps could be applied to install other libraries like `rclpy`, the Python equivalent of `rclcpp`. By doing so, one could create much more complex and modern nodes for a much more extensive control over various hardware components.

Finally, another development could be to focus on the operation of the test packages, which were ignored in this guide for time reasons. Their operation would allow for specific testing of the created nodes.
