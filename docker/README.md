# 🚀 Run vS-Graphs using Docker

## ✅ I. Set Up NVIDIA Container Toolkit (Ubuntu)

You might need to install `Nvidia`'s container toolkit to

```bash
# Detect your Ubuntu distribution
distribution=$(. /etc/os-release;echo $ID$VERSION_ID)

# Add the NVIDIA GPG key
curl -s -L https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg

# Add the NVIDIA container repository
curl -s -L https://nvidia.github.io/libnvidia-container/$distribution/nvidia-container-toolkit.list | sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list

# Install the toolkit
sudo apt update
sudo apt install -y nvidia-container-toolkit

# Configure the Docker runtime
sudo nvidia-ctk runtime configure --runtime=docker

# Restart Docker
sudo systemctl restart docker
```

## ⚙️ II. Build

To build the Docker image, run the following command within this directory:

```bash
docker build \
  --build-arg ssh_prv_key="$(cat ~/.ssh/id_rsa)" \
  --build-arg ssh_pub_key="$(cat ~/.ssh/id_rsa.pub)" \
  -t vsgraphs .
```

--build-arg ssh_pub_key="$(cat ~/.ssh/id_ed25519.pub)" -t vsgraphs .

> 🛎️ Tip: Please note that your Github authentication keys might be named differently depending on the encryption algorithm. If above does not work, try replacing `id_rsa` with `id_ed25519`, in the above command.

## 🚀 III. Run

### III-A. Run the Docker Image

Use one of the below options:

```bash
# [Option I] using Docker
docker run -it -d --privileged --name vsgraphs -e DISPLAY=$DISPLAY -e XAUTHORITY=$XAUTHORITY -v /tmp/.X11-unix:/tmp/.X11-unix -v $XAUTHORITY:$XAUTHORITY vsgraphs

# [Option II] using Docker Compose
docker compose up -d
```

> 🛎️ Tip: If you use **Docker Compose**, do not forget to set `"[dataset/path]:/root/datasets"` to the path in which your `rosbags` exist.

### III-B. Run the Container

```bash
docker exec -it vsgraphs bash

# Inside the container
source /opt/ros/noetic/setup.bash && source devel/setup.bash && roslaunch orb_slam3_ros vs_graphs.launch
```

You can use any other launch files instead of `vs_graphs.launch`.
