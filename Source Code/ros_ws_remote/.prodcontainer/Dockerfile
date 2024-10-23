#  The ROS remote project - ROS 2 Deployment Container
#  Copyright 2024 Samyar Sadat Akhavi
#  Prepared by Samyar Sadat Akhavi, 2024.
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

FROM samyarsadat/custom_base_images:base-ros-humble
ENV USERNAME=nonroot


# Install pip, python3-opencv, libxcb-cursor0, freeglut3-dev, libxcb-shape0, 
# libxkbcommon-x11-0, libxcb-icccm4, libxcb-keysyms1, ros-humble-cv-bridge
RUN apt-get update \
    && apt-get install -y python3-pip python3-opencv libxcb-cursor0 freeglut3-dev \
                          libxcb-shape0 libxkbcommon-x11-0 libxcb-icccm4 libxcb-keysyms1 ros-humble-cv-bridge \
    && rm -rf /var/lib/apt/lists/* && apt-get autoremove && apt-get autoclean

# Install PySide6
RUN pip install --no-cache-dir PySide6

# Raspberry Pi Pico TTY permission fix
USER root
RUN usermod -aG dialout ${USERNAME}
    
# Entrypoint
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
USER ${USERNAME}
CMD ["ros2", "run", "ros_remote_gui", "remote_gui_node"]