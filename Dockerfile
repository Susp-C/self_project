FROM osrf/ros:noetic-desktop-full
<launch>
  <arg name="is_sim"        default="0"/>
  <arg name="is_physical"   default="1"/>
  <arg name="use_joystick"  default="1"/>
  <arg name="use_keyboard"  default="0"/>
  <arg name="serial_port"   default="/dev/serial0"/>
  <arg name="use_imu"       default="0"/>

  <!-- 新增 -->
  <arg name="use_ai_camera" default="0"/>
  <arg name="rosbridge_port" default="9090"/>

  <group if="$(arg is_physical)">
    <node pkg="rosserial_python" type="serial_node.py"
          name="dingo_rosserial" args="$(arg serial_port)" output="screen"/>

    <node pkg="dingo_peripheral_interfacing"
          type="dingo_lcd_interfacing.py"
          name="dingo_LCD_node" output="screen"/>
  </group>

  <group if="$(arg use_joystick)">
    <node pkg="joy" type="joy_node" name="JOYSTICK">
      <param name="autorepeat_rate" value="30"/>
    </node>
  </group>

  <group if="$(arg use_keyboard)">
    <node pkg="dingo_input_interfacing" type="Keyboard.py"
          name="keyboard_input_listener" output="screen"/>
  </group>

  <node pkg="dingo" type="dingo_driver.py"
        name="dingo"
        args="$(arg is_sim) $(arg is_physical) $(arg use_imu)"
        output="screen"/>

  <!-- ===== AI 接管部分(需要 rosbridge + ai_bridge) ===== -->
  <group if="$(arg use_ai_camera)">
    <include file="$(find rosbridge_server)/launch/rosbridge_websocket.launch">
      <arg name="port" value="$(arg rosbridge_port)"/>
    </include>

    <node pkg="dingo" type="dingo_ai_bridge.py"
          name="dingo_ai_bridge" output="screen">
      <param name="enabled"  value="true"/>
      <param name="max_x"    value="0.4"/>
      <param name="max_y"    value="0.3"/>
      <param name="max_yaw"  value="1.0"/>
      <param name="cmd_rate" value="30.0"/>
    </node>
  </group>
</launch>

RUN sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    gdb \
    apt-utils \
    python3-rosdep \
    python3-pip \
    python3-vcstool \
    python3-pymodbus \
    build-essential \
    ros-noetic-catkin \
    python3-catkin-tools \
    ros-noetic-ros-controllers \
    nano \
    ros-noetic-soem \
    libvlccore-dev \
    libvlc-dev \
    ros-noetic-joy \
    ros-noetic-rosserial \
    ros-noetic-rosserial-arduino \
    ros-noetic-rosbridge-server
    ros-noetic-rosbridge-suite
    git \
&& rm -rf /var/lib/apt/lists/*

RUN pip3 install \
    #Following are from pupper code
    transforms3d \
    UDPComms \
    serial \
    pyserial \
    pigpio \
    regex \
    matplotlib \
    #Following are Nathan/Alex additions
    pynput \
    spidev \
    #adafruit-circuitpython-pca9685 \
    adafruit-circuitpython-servokit

# Make the prompt a little nicer
RUN echo "PS1='${debian_chroot:+($debian_chroot)}\u@:\w\$ '" >> /etc/bash.bashrc 

WORKDIR /dingo_ws
COPY /dingo_ws/src /dingo_ws/src
RUN rosdep update
RUN rosdep install --from-paths src --ignore-src -r -y

RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /etc/bash.bashrc

RUN /bin/bash -c 'source /opt/ros/$ROS_DISTRO/setup.bash &&\
catkin_make --directory /dingo_ws -DCMAKE_BUILD_TYPE=Debug'

RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /etc/bash.bashrc
RUN echo "source /dingo_ws/devel/setup.bash" >> /etc/bash.bashrc

ENTRYPOINT ["./ros_entrypoint.sh"]
CMD [ "bash" ]



