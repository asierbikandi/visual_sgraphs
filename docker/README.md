# 🚀 Run vS-Graphs using Docker
---

To build the Docker image, run the following command within this directory:

```bash
docker build \
  --build-arg ssh_prv_key="$(cat ~/.ssh/id_rsa)" \
  --build-arg ssh_pub_key="$(cat ~/.ssh/id_rsa.pub)" \
  -t vsgraphs .
```

Please note that your Github authentication keys might be named differently depending on the encryption algorithm. If above does not work, try replacing `id_rsa` with `ed25519`, in the above command.   
