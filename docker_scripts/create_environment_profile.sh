DIR="$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

readonly DEST_DIR="${DIR}"

profile_name="dev"

mkdir -p $DEST_DIR

# env used in the guest .bashrc
if [[ ! -f ${DEST_DIR}/.guest_env_${profile_name} ]]; then
    tee -a ${DEST_DIR}/.guest_env_${profile_name} > /dev/null <<'EOF'
#!/usr/bin/env bash

DIR="$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
source $DIR/.guest_env_default

EOF
    echo "Created environment file at ${DEST_DIR}/.guest_env_${profile_name}"
else
    echo "Environment file already exists"
fi

volume_name="ros1_noetic_dev_volume"
docker volume create ${volume_name}
docker run \
        -it --rm \
        --env="UID=$(id -u)" \
        --env="GID=$(id -g)" \
        -v ${volume_name}:/data \
        busybox:latest \
        sh -c 'chown -R $UID:$GID /data'