FROM wpilib/gazebo-ubuntu:22.04

RUN apt update -y
RUN apt install sudo -y
#
## Install needed packages
RUN sudo apt install -y python3 python3-pip
RUN sudo apt install -y curl wget vim
RUN sudo apt install -y zip unzip
RUN sudo apt install -y apt-transport-https gnupg
#
## Install sdkman
RUN curl -s "https://get.sdkman.io?rcupdate=true" | bash
#
## Install java
RUN bash -c "source /root/.sdkman/bin/sdkman-init.sh"
#
## Install gradle
RUN bash -c "source /root/.sdkman/bin/sdkman-init.sh && sdk install gradle && sdk install java 11.0.12-open"
#
RUN echo "source /root/.sdkman/bin/sdkman-init.sh" >> /root/.bashrc

#
## Install Bazel
#RUN curl -fsSL https://bazel.build/bazel-release.pub.gpg | gpg --dearmor >bazel-archive-keyring.gpg
#RUN sudo mv bazel-archive-keyring.gpg /usr/share/keyrings
#RUN echo "deb [arch=amd64 signed-by=/usr/share/keyrings/bazel-archive-keyring.gpg] https://storage.googleapis.com/bazel-apt stable jdk1.8" | sudo tee /etc/apt/sources.list.d/bazel.list
#RUN sudo apt update && sudo apt install bazel
#sudo apt update && sudo apt full-upgrade
