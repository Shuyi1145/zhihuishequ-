#!/usr/bin/env bash
# Restore backed-up references; unused appearance meshes can stay in place.
set -euo pipefail
backup_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if [[ ! -f "$backup_dir/target_package.txt" ]]; then
    printf '%s\n' 'Run the restore.sh located inside the backup directory printed during installation.' >&2
    exit 1
fi
IFS= read -r model_pkg < "$backup_dir/target_package.txt"
if [[ ! -d "$model_pkg" || ! -w "$model_pkg" ]]; then
    printf '%s\n' "Original package is unavailable or not writable: $model_pkg" >&2
    exit 1
fi
for relative in \
    urdf/robot_description.urdf.xacro \
    urdf/robot_description.urdf \
    urdf/robot_description.gazebo.xacro \
    meshes/base_chamfered_bluegrey.dae \
    meshes/body_guards_silver.dae; do
    if [[ -f "$backup_dir/$relative" ]]; then
        cp -p -- "$backup_dir/$relative" "$model_pkg/$relative"
    fi
done
printf '%s\n' "Restored backed-up files to: $model_pkg"
printf '%s\n' 'Restart the simulation to apply the restored model.'
