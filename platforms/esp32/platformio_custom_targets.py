try:
    Import("env")
except NameError:
    env = None

import os
import subprocess


def monitor_after_upload(target, source, env, **kwargs):
    project_dir = env.subst("$PROJECT_DIR")
    helper = os.path.join(
        project_dir,
        "scripts",
        "platformio_install_and_monitor.sh",
    )
    pio_env = env.subst("$PIOENV")
    device_name = (
        "Remus Blade" if pio_env == "remus-blade-dev" else "Remus Computer"
    )
    command = [
        helper,
        pio_env,
        device_name,
        "PlatformIO Custom task",
        "--monitor-only",
    ]

    upload_port = env.subst("$UPLOAD_PORT").strip()
    if upload_port and upload_port.lower() not in {"none", "auto"}:
        command.extend(["--port", upload_port])

    stdin_target = None
    try:
        if hasattr(os, "isatty") and os.isatty(0):
            stdin_target = None
        elif os.path.exists("/dev/tty"):
            stdin_target = open("/dev/tty", "r")
    except Exception:
        stdin_target = None

    try:
        return subprocess.call(command, cwd=project_dir, stdin=stdin_target)
    finally:
        if stdin_target is not None:
            try:
                stdin_target.close()
            except Exception:
                pass


if env is not None:
    env.AddCustomTarget(
        name="install_and_monitor",
        dependencies=["upload"],
        actions=[monitor_after_upload],
        title="Install & Monitor",
        description="Build, upload and open the REMUS serial monitor at 115200 baud",
        always_build=True,
    )

