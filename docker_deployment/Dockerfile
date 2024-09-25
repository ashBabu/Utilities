FROM ros:humble-ros-base

ARG DEBIAN_FRONTEND=noninteractive

# Install basic packages
RUN apt-get update && \
    apt-get install -y \
        locales \
        curl \
        build-essential \
        gettext-base \
        software-properties-common && \
    rm -rf /var/lib/apt/lists/*

# Install apt packages
COPY docker_images/deploy-image/apt-packages /tmp/

RUN apt-get update && \
    apt-get install -y \
        $(cut -d# -f1 </tmp/apt-packages | envsubst) && \
    rm -rf /var/lib/apt/lists/* /tmp/apt-packages

ARG USERNAME=headlightai
RUN useradd -ms /bin/bash $USERNAME && echo "$USERNAME ALL=(ALL) NOPASSWD: ALL" >> /etc/sudoers

# Switch to the new user and clone the private repo into their home directory
USER $USERNAME
WORKDIR /home/$USERNAME

# Create an SSH config directory to avoid GitHub host key checking issues
RUN mkdir -p ~/.ssh && ssh-keyscan github.com >> ~/.ssh/known_hosts

# Clone the private repo (repository URL and token will be injected via the workflow)
ARG GIT_USERNAME 
ARG GIT_REPO_NAME
ARG GIT_TOKEN

# Clone the private repository into the user's home directory
RUN git clone https://$GIT_TOKEN@github.com/$GIT_USERNAME/$GIT_REPO_NAME.git /home/$USERNAME/$GIT_REPO_NAME
# RUN git clone -b $GIT_BRANCH https://$GIT_TOKEN@$GIT_REPO /home/$USERNAME/$GIT_REPO

# Build the project (example with colcon)
WORKDIR /home/$USERNAME/$GIT_REPO_NAME
# Delete all except ros2_ws and cbuild.sh
# RUN find . -maxdepth 1 ! -name 'ros2_ws' ! -name 'cbuild.sh' ! -name '.' -exec rm -rf {} + 
RUN rm -rf docker_images/ .adehome .aderc .gitignore start-ade .github
# RUN chmod +x save_map_call.sh
USER root
RUN chmod -R a+rwx docker_startup.sh
USER $USERNAME
WORKDIR /home/$USERNAME/$GIT_REPO_NAME/ros2_ws/
RUN bash cbuild.sh
# RUN rm -rf src

RUN bash install/setup.bash
RUN echo "source /home/$USERNAME/$GIT_REPO_NAME/ros2_ws/install/setup.bash" >> ~/.bashrc
WORKDIR /home/$USERNAME/$GIT_REPO_NAME/
CMD ["./docker_startup.sh"]