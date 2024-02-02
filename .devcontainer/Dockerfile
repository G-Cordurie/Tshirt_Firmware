FROM ubuntu:20.04

USER root
WORKDIR /tmp
ARG PWD

ARG NRF5_TOOLCHAIN_URL=https://developer.arm.com/-/media/Files/downloads/gnu-rm/9-2020q2/gcc-arm-none-eabi-9-2020-q2-update-x86_64-linux.tar.bz2
ARG NRF5_CLI_URL=https://www.nordicsemi.com/-/media/Software-and-other-downloads/Desktop-software/nRF-command-line-tools/sw/Versions-10-x-x/10-15-4/nrf-command-line-tools-10.15.4_amd64.zip
ARG SES_URL=https://www.segger.com/downloads/embedded-studio/Setup_EmbeddedStudio_ARM_v570a_linux_x64.tar.gz

# Download tools and prerequisites
ARG DEBIAN_FRONTEND=noninteractive
ENV TZ=Europe/Paris
RUN \
    apt-get update && apt-get upgrade -y && \
    apt-get -y install vim git git-gui git-lfs wget unzip bzip2 python build-essential gcc-multilib pkg-config make srecord libusb-1.0.0 curl python3-pip udev libsm6 && \
    apt-get -y install python-protobuf protobuf-compiler && \
    apt-get clean all

# Install the Toolchain
RUN \
    wget -P /tmp -q --show-progress $NRF5_TOOLCHAIN_URL && \
    tar -xvjf /tmp/gcc-arm-none-eabi-*.tar.bz2 -C /usr/local && \
    rm /tmp/gcc-arm-none-eabi-*.tar.bz2

# Install nRF CLI
RUN \
    wget -P /tmp -q --show-progress $NRF5_CLI_URL && \
    unzip /tmp/nrf-command-line-tools-10.15.4_amd64.zip -d /tmp && \
    dpkg -i /tmp/JLink_Linux_V758b_x86_64.deb && \
    dpkg -i --force-overwrite /tmp/nrf-command-line-tools_10.15.4_amd64.deb && \
    rm -r /tmp/*

# Install nRF pynrfjprog & nRF Util
RUN \
    pip3 install pip --upgrade && \
    pip3 install pynrfjprog && \
    pip3 install nrfutil

# Install Segger Embedded Studio
RUN \
    apt-get -y install libfreetype6 libxrender1 libfontconfig1 libsm6 libxrandr2 libxfixes3 libxcursor1 && \
    wget -P /tmp -q --show-progress $SES_URL && \
    tar -zxvf /tmp/Setup_EmbeddedStudio_ARM_*_linux_x64.tar.gz && \
    ./arm_segger_embedded_studio_570a_linux_x64/install_segger_embedded_studio --copy-files-to /usr/local/segger_embedded_studio --accept-license

# SDK Env Vars
ENV SDK_ROOT=$PWD/sdk/nRF5_SDK_17.1.0_ddde560
ENV BUILD_PROJECT=kSenseQi
ENV DEBUG=1
ENV QT_X11_NO_MITSHM=1

# Run a bash shell
CMD ["/bin/bash"]
