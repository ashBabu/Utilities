FROM ghcr.io/headlightai/telesto-ros2-deploy-libs:latest

ARG DEBIAN_FRONTEND=noninteractive
RUN userdel -r ubuntu

# Create an SSH config directory to avoid GitHub host key checking issues
RUN mkdir -p ~/.ssh && ssh-keyscan github.com >> ~/.ssh/known_hosts
ARG HOST_UID=1000
ARG HOST_GID=1000

ENV GIT_USERNAME=headlightai 
ENV GIT_REPO_NAME=telesto-ros2
ARG GIT_TOKEN

RUN apt-get update && apt-get install -y sudo \
    && groupadd --gid $HOST_GID $GIT_USERNAME \
    && useradd --uid  $HOST_UID --gid $HOST_GID -m -s /bin/bash $GIT_USERNAME \
    && echo "$GIT_USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

ENV HOME=/home/$GIT_USERNAME

USER $GIT_USERNAME

WORKDIR $HOME
# Clone the private repository into the user's home directory
RUN git clone --recurse-submodules https://$GIT_TOKEN@github.com/$GIT_USERNAME/$GIT_REPO_NAME.git 

# Build the project (example with colcon)
WORKDIR $HOME/$GIT_REPO_NAME
# Delete all except ros2_ws and cbuild.sh
# RUN find . -maxdepth 1 ! -name 'ros2_ws' ! -name 'cbuild.sh' ! -name '.' -exec rm -rf {} + 
RUN rm -rf docker_images/ .adehome .aderc .gitignore start-ade .github .bashrc
# RUN chmod +x save_map_call.sh
RUN chmod -R a+rwx docker_startup.sh
RUN cd ros2_ws && bash release_build.sh && rm -rf src

CMD ["./docker_startup.sh"]