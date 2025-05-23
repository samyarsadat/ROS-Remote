#  The ROS remote project - MicroROS Library Build Workspace Dockerfile
#  Copyright 2024-2025 Samyar Sadat Akhavi
#  Prepared by Samyar Sadat Akhavi, 2024-2025.
#
#  This program is free software: you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation, either version 3 of the License, or
#  (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#
#  You should have received a copy of the GNU General Public License
#  along with this program.  If not, see <https: www.gnu.org/licenses/>.
#  
#  Bits of code taken from:
#  https://github.com/joshnewans/dockerfile_example/blob/main/Dockerfile
#  https://micro.ros.org/docs/tutorials/core/first_application_linux/
#  https://circuitdigest.com/microcontroller-projects/how-to-program-raspberry-pi-pico-using-c
#  https://github.com/BrunoB81HK/ros2-workspace-devcontainer-template

FROM samyarsadat/custom_base_images:base-ros-jazzy



# Install pip
RUN apt-get update \
    && apt-get install -y python3-pip \
    && rm -rf /var/lib/apt/lists/* && apt-get autoremove && apt-get autoclean

# Install OpenSSH Server
#RUN apt-get update \
#    && apt-get install -y python3-pip openssh-server \
#    && rm -rf /var/lib/apt/lists/* && apt-get autoremove && apt-get autoclean

# Set the SSH server port to 44
#RUN sed -i "s/#Port 22/Port 44/" /etc/ssh/sshd_config

# Set a password for the nonroot user
#RUN echo "${USERNAME}:nonroot" | chpasswd

# Pico setup
COPY install_pico.sh /pico_setup.sh
RUN export NONROOT_USERNAME=${USERNAME} \
    && bash /pico_setup.sh \
    && rm -rf /var/lib/apt/lists/* && apt-get autoremove && apt-get autoclean

# MicroROS agent helpers
USER ${USERNAME}
COPY uros_agent/* /home/$USERNAME/pico_ws/uros_agent/

# Copy configuration files
USER ${USERNAME}
COPY vscode_config/* /home/$USERNAME/pico_ws/.vscode/

# OpenOCD Debug Probe permission fix
USER root
RUN usermod -aG plugdev ${USERNAME}

# Raspberry Pi Pico TTY permission fix
USER root
RUN usermod -aG dialout ${USERNAME}
    
# Post create and start scripts
COPY post_create.sh /devcon_post_create.sh
COPY post_start.sh /devcon_post_start.sh
USER root
RUN chmod +x /devcon_post_start.sh && chmod +x /devcon_post_create.sh
USER ${USERNAME}