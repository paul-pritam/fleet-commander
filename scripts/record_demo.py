#!/usr/bin/env python3
"""
Autonomous Headless Demo Recorder for Fleet Commander.
Records a 1080p 60 FPS demo with authentic mouse click interaction
on an isolated Hyprland virtual display without touching user workspaces.
Outputs a VLC-compatible MP4 (H.264 / yuv420p).
"""

import os
import sys
import time
import signal
import subprocess
import json

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, ".."))
RECORDINGS_DIR = os.path.expanduser("~/Videos/recordings")
OUTPUT_MP4 = os.path.join(RECORDINGS_DIR, "fleet_commander_demo.mp4")
YDOTOOL = os.path.expanduser("~/.local/bin/ydotool")
YDOTOOLD = os.path.expanduser("~/.local/bin/ydotoold")
WF_RECORDER = os.path.expanduser("~/.local/bin/wf-recorder")
SOCKET_PATH = "/tmp/.ydotool_socket"


def run_cmd(cmd, check=True, capture=True):
    if capture:
        res = subprocess.run(cmd, shell=True, text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    else:
        res = subprocess.run(cmd, shell=True)
    if check and res.returncode != 0:
        print(f"[ERROR] Command failed ({res.returncode}): {cmd}\nStderr: {res.stderr}")
    return res


def get_hyprland_display_info(output_name="demo_display"):
    res = run_cmd("hyprctl monitors -j", check=True)
    try:
        monitors = json.loads(res.stdout)
        for m in monitors:
            if m.get("name") == output_name:
                return m.get("x", 5760), m.get("y", 0), m.get("width", 1920), m.get("height", 1080)
    except Exception as e:
        print(f"[WARN] Failed parsing monitors: {e}")
    return 5760, 0, 1920, 1080


def smooth_move(target_x, target_y, steps=20, duration=0.6):
    res = run_cmd("hyprctl cursorpos", check=False)
    curr_x, curr_y = 5760, 500
    if res.stdout and "," in res.stdout:
        parts = res.stdout.strip().split(",")
        try:
            curr_x = float(parts[0].strip())
            curr_y = float(parts[1].strip())
        except ValueError:
            pass

    delay = duration / max(steps, 1)
    for i in range(1, steps + 1):
        alpha = i / steps
        # smoothstep interpolation
        t = alpha * alpha * (3.0 - 2.0 * alpha)
        interp_x = curr_x + (target_x - curr_x) * t
        interp_y = curr_y + (target_y - curr_y) * t
        subprocess.run(
            ["hyprctl", "dispatch", "movecursor", str(int(interp_x)), str(int(interp_y))],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        time.sleep(delay)


def click_left():
    env = os.environ.copy()
    env["YDOTOOL_SOCKET"] = SOCKET_PATH
    subprocess.run([YDOTOOL, "click", "1"], env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def smooth_move_and_click(target_x, target_y, duration=0.7):
    smooth_move(target_x, target_y, steps=25, duration=duration)
    time.sleep(0.15)
    click_left()
    time.sleep(0.2)


def main():
    os.makedirs(RECORDINGS_DIR, exist_ok=True)
    if os.path.exists(OUTPUT_MP4):
        try:
            os.remove(OUTPUT_MP4)
        except OSError:
            pass

    print("[1/7] Initializing virtual mouse daemon (ydotoold)...")
    run_cmd(f"pkill -f {YDOTOOLD} || true", check=False)
    ydo_proc = subprocess.Popen(
        [YDOTOOLD, f"--socket-path={SOCKET_PATH}", "--socket-perm=0666"],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    time.sleep(0.6)

    print("[2/7] Creating headless virtual display 'demo_display' on Workspace 4...")
    run_cmd('hyprctl keyword monitor "demo_display,1920x1080@60,auto,1"', check=False)
    run_cmd("hyprctl output create headless demo_display", check=False)
    time.sleep(1.0)

    base_x, base_y, disp_w, disp_h = get_hyprland_display_info("demo_display")
    print(f"      Virtual monitor mapped at ({base_x}, {base_y}), resolution: {disp_w}x{disp_h}")

    print("[3/7] Setting Hyprland window placement rules for Workspace 4...")
    run_cmd('hyprctl keyword windowrulev2 "workspace 4, class:(fleet_commander)"', check=False)

    print("[4/7] Launching wf-recorder with NVIDIA NVENC (H.264 / yuv420p for VLC)...")
    wf_cmd = [
        WF_RECORDER,
        "-o", "demo_display",
        "-c", "h264_nvenc",
        "-x", "yuv420p",
        "-r", "60",
        "-f", OUTPUT_MP4,
    ]
    rec_proc = subprocess.Popen(wf_cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(1.2)

    print("[5/7] Starting Fleet Commander on Workspace 4...")
    fc_script = os.path.join(PROJECT_DIR, "start_fleet_commander.sh")
    fc_proc = subprocess.Popen(["bash", fc_script], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(4.5)

    try:
        print("[6/7] Executing choreographed mouse click showcase...")

        # Act 1: Live Fleet Map View
        print("      - Act 1: Viewing live warehouse map and idle robot telemetry...")
        time.sleep(3.0)

        # Move to warehouse aisle and click to dispatch Goal 1
        g1_x = base_x + 580
        g1_y = base_y + 460
        print(f"      - Moving cursor to Inbound Corridor ({g1_x}, {g1_y}) and clicking...")
        smooth_move_and_click(g1_x, g1_y, duration=0.9)
        time.sleep(1.5)

        # Move to central buffer and click to queue Goal 2
        g2_x = base_x + 720
        g2_y = base_y + 540
        print(f"      - Moving cursor to Central Buffer ({g2_x}, {g2_y}) and clicking Goal 2...")
        smooth_move_and_click(g2_x, g2_y, duration=0.8)

        # Let viewer watch AMR real-time tracking
        print("      - Tracking AMR trajectory navigation in real-time (12 seconds)...")
        time.sleep(12.0)

        # Act 2: Switch to Benchmark Page via Navigation Bar
        comp_btn_x = base_x + 520
        comp_btn_y = base_y + 22
        print("      - Act 2: Moving cursor to navigation bar '[Run Comparison & Benchmark]' and clicking...")
        smooth_move_and_click(comp_btn_x, comp_btn_y, duration=0.8)
        time.sleep(2.0)

        # Click [ Run Benchmark Comparison ] button if needed
        bench_btn_x = base_x + 1550
        bench_btn_y = base_y + 220
        print("      - Clicking '[Run Benchmark Comparison]'...")
        smooth_move_and_click(bench_btn_x, bench_btn_y, duration=0.7)

        # Move cursor to trajectory canvas to showcase the overlay
        canvas_x = base_x + 600
        canvas_y = base_y + 500
        smooth_move(canvas_x, canvas_y, steps=20, duration=0.8)

        # Showcase the full benchmark metrics for 12 seconds
        print("      - Showcasing dual trajectory overlay and performance benchmark (12 seconds)...")
        time.sleep(12.0)

    finally:
        print("[7/7] Stopping recorder and cleaning up virtual display...")
        if rec_proc.poll() is None:
            rec_proc.send_signal(signal.SIGINT)
            try:
                rec_proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                rec_proc.kill()

        if fc_proc.poll() is None:
            fc_proc.terminate()
            try:
                fc_proc.wait(timeout=3)
            except subprocess.TimeoutExpired:
                fc_proc.kill()

        run_cmd("pkill -f fleet_commander || true", check=False)
        run_cmd("hyprctl output remove demo_display || true", check=False)

        if ydo_proc.poll() is None:
            ydo_proc.terminate()
            try:
                ydo_proc.wait(timeout=2)
            except subprocess.TimeoutExpired:
                ydo_proc.kill()
        run_cmd(f"pkill -f {YDOTOOLD} || true", check=False)

    if os.path.exists(OUTPUT_MP4):
        size_mb = os.path.getsize(OUTPUT_MP4) / (1024 * 1024)
        print(f"\n[SUCCESS] Flawless demo recorded successfully!")
        print(f"File: {OUTPUT_MP4} ({size_mb:.2f} MB)")
        # Verify with ffprobe
        run_cmd(f"ffprobe -v error -show_entries stream=width,height,r_frame_rate,codec_name,pix_fmt -of default=noprint_wrappers=1 {OUTPUT_MP4}", check=False, capture=False)
        print("\nReady to view in VLC: vlc ~/Videos/recordings/fleet_commander_demo.mp4")
    else:
        print("[FAILED] Output video was not created.")
        sys.exit(1)


if __name__ == "__main__":
    main()
