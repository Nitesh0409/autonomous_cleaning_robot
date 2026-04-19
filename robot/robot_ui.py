import tkinter as tk
from tkinter import ttk, messagebox
import subprocess
import threading
import os

# ── World catalogue ────────────────────────────────────────────────────────────
WORLDS = {
    "patrol_arena":      ("Patrol Arena — Main World 🆕",          True,  "patrol_launch.py"),
    "arm_test":          ("Arm Test — Pick & Place",                False, "sim_gazebo.launch.py"),
    "static_obstacles":  ("Static Obstacles — Nav Test",            False, "sim_gazebo.launch.py"),
    "dynamic_obstacles": ("Dynamic Obstacles — Avoidance Test",     False, "sim_gazebo.launch.py"),
    "mini_proving_ground":("Mini Proving Ground — Free Roam",       False, "sim_gazebo.launch.py"),
    "benchmark_obstacle": ("Static Wall Pass — Benchmark",          False, "sim_gazebo.launch.py"),
    "slalom_gauntlet":   ("The Gauntlet — Slalom",                  False, "sim_gazebo.launch.py"),
}


class RobotMissionUI:
    def __init__(self, root):
        self.root = root
        self.root.title("🚀 Micro-Sentinel Mission Control")
        self.root.geometry("490x560")
        self.root.attributes('-topmost', 1)
        self.root.resizable(False, False)
        self.root.configure(bg="#1e1e2e", padx=18, pady=14)

        self.world_key        = tk.StringVar(value="patrol_arena")
        self.planner_local_var = tk.StringVar(value="apf")
        self.planner_global_var= tk.StringVar(value="dstar")
        self.face_path_var     = tk.BooleanVar(value=True)
        self.robot_count_var   = tk.IntVar(value=1)

        self.sim_process = None
        self._apply_style()
        self._build_ui()

    # ── Styling ────────────────────────────────────────────────────────────────
    def _apply_style(self):
        s = ttk.Style()
        s.theme_use("clam")
        bg, fg, acc = "#1e1e2e", "#cdd6f4", "#89b4fa"
        s.configure(".",                background=bg, foreground=fg)
        s.configure("TFrame",           background=bg)
        s.configure("TLabel",           background=bg, foreground=fg)
        s.configure("TRadiobutton",     background=bg, foreground=fg)
        s.configure("TCheckbutton",     background=bg, foreground=fg)
        s.configure("TCombobox",
                    fieldbackground="#313244", foreground=fg,
                    background="#313244", selectbackground="#45475a")
        s.configure("TSpinbox",
                    fieldbackground="#313244", foreground=fg, background="#313244")
        s.configure("TLabelframe",
                    background=bg, foreground=acc, relief="groove")
        s.configure("TLabelframe.Label",
                    background=bg, foreground=acc, font=("Consolas", 9, "bold"))

    def _section(self, title):
        f = ttk.LabelFrame(self.root, text=f"  {title}  ", padding=(10, 5))
        f.pack(fill="x", pady=4)
        return f

    def _btn(self, parent, text, color, cmd, side="left", w=None):
        kw = dict(bg=color, fg="white", font=("Consolas", 9, "bold"),
                  relief="flat", cursor="hand2",
                  activebackground=color, activeforeground="white",
                  command=cmd, pady=7)
        if w:
            kw["width"] = w
        b = tk.Button(parent, text=text, **kw)
        b.pack(side=side, expand=(w is None), fill="x", padx=2)
        return b

    # ── UI Build ───────────────────────────────────────────────────────────────
    def _build_ui(self):
        tk.Label(self.root, text="MICRO-SENTINEL CONTROL", bg="#1e1e2e",
                 fg="#cba6f7", font=("Consolas", 13, "bold")).pack(pady=(0, 10))

        # 1. World Selection
        wf = self._section("1 ▸ World")
        labels = [v[0] for v in WORLDS.values()]
        self._world_display_map = {v[0]: k for k, v in WORLDS.items()}

        self.world_cb = ttk.Combobox(wf, values=labels, state="readonly",
                                     font=("Consolas", 9))
        self.world_cb.set(WORLDS["patrol_arena"][0])
        self.world_cb.pack(fill="x")
        self.world_cb.bind("<<ComboboxSelected>>", self._on_world_change)

        # 2a. Global Planner
        gf = self._section("2a ▸ Global Planner")
        ttk.Radiobutton(gf, text="D* Lite  — incremental replan (fast updates)",
                        variable=self.planner_global_var, value="dstar").pack(anchor="w")
        ttk.Radiobutton(gf, text="A*       — grid search + LOS smoothing",
                        variable=self.planner_global_var, value="astar").pack(anchor="w")

        # 2b. Local Planner
        af = self._section("2b ▸ Local Planner")
        ttk.Radiobutton(af, text="APF — Artificial Potential Fields",
                        variable=self.planner_local_var, value="apf").pack(anchor="w")
        ttk.Radiobutton(af, text="DWA — Dynamic Window Approach",
                        variable=self.planner_local_var, value="dwa").pack(anchor="w")

        # 3. Robot Count (only enabled for patrol_arena)
        rc_frame = self._section("3 ▸ Robot Count  (Patrol Arena only)")
        rc_row = ttk.Frame(rc_frame)
        rc_row.pack(fill="x")
        ttk.Label(rc_row, text="Bots:").pack(side="left")
        self.bot_spin = ttk.Spinbox(rc_row, from_=1, to=6,
                                    textvariable=self.robot_count_var,
                                    width=4, font=("Consolas", 10))
        self.bot_spin.pack(side="left", padx=8)
        ttk.Label(rc_row, text="(1–6 — only applies to Patrol Arena)",
                  foreground="#6c7086", font=("Consolas", 8)).pack(side="left")

        # 4. Heading Mode
        hf = self._section("4 ▸ Heading Mode")
        self.face_chk = ttk.Checkbutton(
            hf, text="Face the Path  (rotate first, then translate)",
            variable=self.face_path_var,
            command=lambda: threading.Thread(
                target=self._send_toggle, daemon=True).start())
        self.face_chk.pack(anchor="w")
        ttk.Label(hf, text="OFF → holonomic strafe + hold 2D pose arrow orientation",
                  foreground="#6c7086", font=("Consolas", 8),
                  wraplength=430, justify="left").pack(anchor="w")

        # 5. POV Switcher
        pf = self._section("5 ▸ Camera POV  (Patrol Arena)")
        pov_row = ttk.Frame(pf)
        pov_row.pack(fill="x")
        self._btn(pov_row, "⬛ Free",        "#313244", lambda: self._set_pov(None), w=8)
        for i in range(6):
            self._btn(pov_row, f"Bot {i}", "#45475a",
                      lambda idx=i: self._set_pov(f"robot_{idx}"), w=6)

        # 6. Launch / Kill
        lf = ttk.Frame(self.root)
        lf.pack(fill="x", pady=12)
        self.btn_launch = self._btn(lf, "▶  LAUNCH", "#40a02b", self._launch)
        self.btn_kill   = self._btn(lf, "■  KILL",   "#d20f39", self._kill, "right")

        self.log_var = tk.StringVar(value="Ready.")
        tk.Label(self.root, textvariable=self.log_var, bg="#1e1e2e",
                 fg="#6c7086", font=("Consolas", 8)).pack()

        # Initial state
        self._on_world_change()

    # ── Callbacks ──────────────────────────────────────────────────────────────
    def _on_world_change(self, _=None):
        label = self.world_cb.get()
        key   = self._world_display_map.get(label, "mini_proving_ground")
        self.world_key.set(key)

        # Enable bot count spinbox ONLY for patrol_arena
        is_patrol = WORLDS[key][1]
        self.bot_spin.config(state="normal" if is_patrol else "disabled")

    def _log(self, msg):
        self.root.after(0, lambda: self.log_var.set(msg))

    def _wsl(self):
        base = os.path.dirname(os.path.abspath(__file__))
        return base.replace("C:", "/mnt/c").replace("c:", "/mnt/c").replace("\\", "/")

    def _run_wsl(self, cmd):
        return subprocess.run(
            ["wsl", "-d", "Ubuntu-24.04", "bash", "-c", cmd],
            capture_output=True, text=True)

    def _send_toggle(self):
        val = "true" if self.face_path_var.get() else "false"
        self._run_wsl(
            f"source /opt/ros/jazzy/setup.bash && "
            f"ros2 topic pub --once /planner/face_movement "
            f"std_msgs/msg/Bool '{{data: {val}}}'"
        )
        label = "Face the Path" if val == "true" else "Face 2D Goal Pose"
        self._log(f"Heading → {label}")

    def _set_pov(self, model):
        name = model or ""
        self._run_wsl(
            f"gz service -s /gui/follow "
            f"--reqtype gz.msgs.StringMsg --reptype gz.msgs.Boolean "
            f"--req 'data: \"{name}\"' --timeout 1000"
        )
        self._log(f"POV: {'Free camera' if not model else model}")

    def _launch(self):
        if self.sim_process:
            messagebox.showwarning("Warning", "Already running! Kill first.")
            return

        key     = self.world_key.get()
        _, is_patrol, launch_file = WORLDS[key]
        local   = self.planner_local_var.get()
        globpl  = self.planner_global_var.get()
        # Encode as combined string; launch files decode it
        # astar global always pairs with dwa local
        if globpl == 'astar':
            planner = 'astar'
        else:
            planner = local
        n_bots  = self.robot_count_var.get()
        wp      = self._wsl()

        if is_patrol:
            extra = f"robot_count:={n_bots} planner:={planner}"
        else:
            extra = f"world_name:={key} planner:={planner} gui:=true"

        cmd = (
            f"export PYTHONUNBUFFERED=1 && "
            f"source /opt/ros/jazzy/setup.bash && "
            f"cd '{wp}' && "
            f"colcon build --symlink-install && "
            f"source './install/setup.bash' && "
            f"ros2 launch robot {launch_file} {extra}"
        )

        self.btn_launch.config(text="RUNNING…", state="disabled", bg="#6c7086")
        self._log(f"Launching '{key}' ({n_bots if is_patrol else 1} bot)…")
        self.sim_process = subprocess.Popen(
            ["wsl", "-d", "Ubuntu-24.04", "bash", "-c", cmd])
        threading.Timer(12.0, self._send_toggle).start()

    def _kill(self):
        self._log("Stopping…")
        kill_patterns = (
            "ruby|gz|rviz|planner|manager|spawner|tracker|"
            "patrol_mission|obstacle_tracker|garbage_spawner"
        )
        self._run_wsl(f"pkill -9 -f '{kill_patterns}'")
        self._run_wsl("pkill -9 -f 'python3'")
        if self.sim_process:
            try: self.sim_process.kill()
            except: pass
            self.sim_process = None
        self.btn_launch.config(text="▶  LAUNCH", state="normal", bg="#40a02b")
        self._log("Killed. Ready.")


def main():
    root = tk.Tk()
    app  = RobotMissionUI(root)
    root.protocol("WM_DELETE_WINDOW", lambda: (app._kill(), root.destroy()))
    root.mainloop()


if __name__ == "__main__":
    main()
