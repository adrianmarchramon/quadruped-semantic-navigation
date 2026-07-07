FROM osrf/ros:noetic-desktop-full

# Evitar prompts interactivos durante la instalación
ENV DEBIAN_FRONTEND=noninteractive

# 1. Instalar dependencias del sistema (Originales + Nuevas)
RUN apt-get update && apt-get install -y \
    git \
    python3-rosdep \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-wstool \
    python3-pip \
    build-essential \
    # Controladores de robot y Gazebo
    ros-noetic-controller-interface \
    ros-noetic-gazebo-ros-control \
    ros-noetic-joint-state-controller \
    ros-noetic-effort-controllers \
    ros-noetic-joint-trajectory-controller \
    # Librerías matemáticas y gráficas
    freeglut3-dev \
    libxmu-dev \
    libxi-dev \
    libfftw3-dev \
    # Dependencias de comunicación y navegación (CRÍTICAS)
    liblcm-dev \
    ros-noetic-move-base-msgs \
    ros-noetic-navigation \
    ros-noetic-cv-bridge \
    ros-noetic-image-transport \
    ros-noetic-sensor-msgs \
    && rm -rf /var/lib/apt/lists/*

# 2. Inicializar rosdep
RUN rosdep update

# 3. Instalar dependencias de Python (Versiones compatibles con Noetic)
# -> AÑADIDO: ultralytics para la detección con YOLO
RUN pip3 install --no-cache-dir \
    numpy==1.23.5 \
    opencv-python==4.8.0.74 \
    protobuf==3.20.3 \
    attrs==23.1.0 \
    ultralytics

# Usamos la 0.10.5 que el log confirmó como disponible
RUN pip3 install --no-cache-dir mediapipe==0.10.5 --no-deps && \
    pip3 install --no-cache-dir sentencepiece

# 4. Configurar el workspace
WORKDIR /catkin_ws

# 5. Configurar el entorno (Bashrc)
# Añadimos la fuente de ROS y del workspace automáticamente
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc && \
    echo "source /catkin_ws/devel/setup.bash" >> ~/.bashrc

# 6. Marcar el directorio como seguro para Git (Evita errores en Docker)
RUN git config --global --add safe.directory /catkin_ws

# Comando por defecto
CMD ["bash"]
