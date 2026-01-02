# 1. Use the official Jupyter 3.10 image
FROM jupyter/base-notebook:python-3.10

USER root

# 2. Install system dependencies (Logic + Graphics)
RUN apt-get update && apt-get install -y \
    swi-prolog \
    libgl1-mesa-dev \
    python3-opengl \
    xvfb \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

USER ${NB_UID}

# 3. Install Python libraries
# (Note: Using a single RUN command is good for reducing image layers)
RUN pip install --no-cache-dir \
    pybullet \
    pyswip \
    numpy \
    pyvirtualdisplay

# 4. Set the Working Directory 
# We use /home/jovyan/work as it is the standard 'persistent' folder
WORKDIR /home/jovyan/work

# 5. Copy your project files into the container
# This ensures that when Binder launches, the code is already there.
COPY --chown=${NB_UID}:${NB_GID} . /home/jovyan/work
