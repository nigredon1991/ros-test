FROM osrf/ros:jazzy-desktop-full
ENV DEBIAN_FRONTEND=noninteractive

# Update the package lists, install necessary packages, and clean up
RUN apt-get update && \
    apt-get install -y --no-install-recommends qtwayland5 && \
    rm -rf /var/lib/apt/lists/*

# Example of a command to run when the container starts
CMD ["/bin/bash"]
