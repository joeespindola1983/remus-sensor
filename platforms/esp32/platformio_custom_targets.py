Import("env")

import os
import subprocess


def monitor_after_upload(source, target, build_env):
    project_dir = build_env.subst("$PROJECT_DIR")
    helper = os.path.join(
        project_dir,
        "scripts",
        "platformio_install_and_monitor.sh",
    )
    command = [
        helper,
        build_env.subst("$PIOENV"),
        "REMUS device",
        "PlatformIO Custom task",
        "--monitor-only",
    ]

    upload_port = build_env.subst("$UPLOAD_PORT").strip()
    if upload_port and upload_port.lower() not in {"none", "auto"}:
        command.extend(["--port", upload_port])

    return subprocess.call(command, cwd=project_dir)


env.AddCustomTarget(
    name="install_and_monitor",
    dependencies=["upload"],
    actions=[monitor_after_upload],
    title="Install & Monitor",
    description="Build, upload and open the REMUS serial monitor at 115200 baud",
    always_build=True,
)

