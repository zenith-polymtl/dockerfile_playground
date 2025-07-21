## 1. Initial Setup

1. Clone this repository and navigate into it.

2. Make the setup script executable and run it:

   ```bash
   chmod +x setup.sh
   ./setup.sh
   ```

   This will install any required dependencies on the host.


 ## 2. Running the Container

### Launch Docker

Start the container in detached mode:

```bash
docker compose up
```

Start the container in detached mode:

```bash
docker compose up -d
```

### Stop Docker

Shut down and remove the container:

```bash
docker compose down
```

### Access the Container Shell

Open an interactive bash session inside the running container:

```bash
docker exec -it dockerfile_playground-zenith-1 bash