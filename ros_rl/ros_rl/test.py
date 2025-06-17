import subprocess


result = subprocess.run(
    [
        "gz", "service", "-s", "/world/empty/control",
        "--reqtype", "gz.msgs.WorldControl",
        "--reptype", "gz.msgs.Boolean",
        "--timeout", "3000",
        "--req", "reset: {all: true}"
    ],
    capture_output=True,
    text=True
)
print("Reset result:", result.stdout)
if result.returncode != 0:
    print("Reset error:", result.stderr)