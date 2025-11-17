import os
import cv2
import json
import time
import datetime
import threading
import numpy as np
import queue


class AsyncEpisodeWriter:
    def __init__(
        self, data_root, camera_config, freq=30, gripper_open=0.07, gripper_close=0.0, max_queue=200, make_preview=False
    ):  # ⭐ 你可以选择是否生成视频预览
        self.data_root = data_root
        self.camera_config = camera_config

        self.freq = freq
        self.gripper_open = gripper_open
        self.gripper_close = gripper_close
        self.make_preview = make_preview  # ⭐ 控制是否生成视频

        self.queue = queue.Queue(maxsize=max_queue)

        self.robot_states = []
        self.timestamps = []

        self.running = False
        self.frame_id = 0

        self.stop_event = threading.Event()
        self.writer_thread = None

        os.makedirs(self.data_root, exist_ok=True)

    # ----------------------------- START EPISODE ----------------------------- #
    def start(self):
        timestamp = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        self.episode_dir = os.path.join(self.data_root, f"episode_{timestamp}")
        os.makedirs(self.episode_dir, exist_ok=True)

        for cfg in self.camera_config:
            cam = cfg["name"]
            if cfg.get("has_rgb", False):
                os.makedirs(os.path.join(self.episode_dir, f"rgb_{cam}"), exist_ok=True)
            if cfg.get("has_depth", False):
                os.makedirs(os.path.join(self.episode_dir, f"depth_{cam}"), exist_ok=True)

        self.timestamps.clear()
        self.robot_states.clear()
        self.frame_id = 0

        self.running = True
        self.stop_event.clear()
        self.writer_thread = threading.Thread(target=self._writer_loop, daemon=True)
        self.writer_thread.start()

        print(f"[AsyncWriter] ▶️ Episode started: {self.episode_dir}")

    # ----------------------------- ADD ONE FRAME ----------------------------- #
    def add_item(self, camera_data, robot_state):
        if not self.running:
            return

        ts = time.time()

        robot_state["frame_id"] = self.frame_id
        self.timestamps.append(ts)
        self.robot_states.append(robot_state)

        try:
            self.queue.put_nowait((self.frame_id, camera_data))
        except queue.Full:
            print("⚠️ WARNING: writer queue FULL — frame dropped!")

        self.frame_id += 1

    # ----------------------------- ASYNC WRITER ----------------------------- #
    def _writer_loop(self):
        """深度图改为 PNG16 保存"""
        while not self.stop_event.is_set() or not self.queue.empty():
            try:
                frame_id, camera_data = self.queue.get(timeout=0.1)
            except queue.Empty:
                continue

            for cfg in self.camera_config:
                cam = cfg["name"]
                sub = camera_data.get(cam, {})

                if "rgb" in sub:
                    path = os.path.join(self.episode_dir, f"rgb_{cam}", f"{frame_id}.jpg")
                    cv2.imwrite(path, sub["rgb"])

                if "depth" in sub:
                    # ⭐ PNG (16-bit) 保存深度
                    path = os.path.join(self.episode_dir, f"depth_{cam}", f"{frame_id}.png")
                    cv2.imwrite(path, sub["depth"])  # depth dtype should be uint16

    # --------------------------- VIDEO STITCHING --------------------------- #
    def _make_preview_video(self):
        """多相机预览视频，深度读取改为 PNG"""
        if not self.make_preview:
            print("[AsyncWriter] ⏭ Skip preview video.")
            return

        print("[AsyncWriter] 🎬 Building multi-camera preview video...")

        rgb_cams = [c["name"] for c in self.camera_config if c.get("has_rgb", False)]
        depth_cams = [c["name"] for c in self.camera_config if c.get("has_depth", False)]

        if len(rgb_cams) == 0 and len(depth_cams) == 0:
            print("[AsyncWriter] ⚠️ No RGB or Depth for preview.")
            return

        # First frame
        first_cam = rgb_cams[0] if rgb_cams else depth_cams[0]
        first_img_path = os.path.join(self.episode_dir, f"rgb_{first_cam}", "0.jpg")
        if not os.path.exists(first_img_path):
            print("[AsyncWriter] ⚠️ Cannot find first frame.")
            return

        first_img = cv2.imread(first_img_path)
        if first_img is None:
            print("[AsyncWriter] ❌ Cannot read preview frame.")
            return

        H, W = first_img.shape[:2]
        pad = 10

        total_rgb_width = len(rgb_cams) * (W + pad) - pad
        total_depth_width = len(depth_cams) * (W + pad) - pad

        final_width = max(total_rgb_width, total_depth_width)
        final_height = H + (pad if len(depth_cams) > 0 else 0) + (H if len(depth_cams) > 0 else 0)

        output_path = os.path.join(self.episode_dir, "preview.avi")
        fourcc = cv2.VideoWriter_fourcc(*"MJPG")  # type: ignore
        writer = cv2.VideoWriter(output_path, fourcc, float(self.freq), (final_width, final_height))

        if not writer.isOpened():
            print("[AsyncWriter] ❌ VideoWriter failed.")
            return

        # 拼接视频
        for frame_id in range(self.frame_id):

            # ---- RGB row ----
            rgb_row = np.zeros((H, total_rgb_width, 3), dtype=np.uint8)
            x = 0
            for cam in rgb_cams:
                img_path = os.path.join(self.episode_dir, f"rgb_{cam}", f"{frame_id}.jpg")
                img = cv2.imread(img_path) if os.path.exists(img_path) else None
                if img is None:
                    img = np.zeros((H, W, 3), dtype=np.uint8)
                rgb_row[:, x : x + W] = img
                x += W + pad

            # ---- Depth row ----
            if depth_cams:
                depth_row = np.zeros((H, total_depth_width, 3), dtype=np.uint8)
                x = 0
                for cam in depth_cams:
                    depth_path = os.path.join(self.episode_dir, f"depth_{cam}", f"{frame_id}.png")
                    if os.path.exists(depth_path):
                        # ⭐ PNG16 读取
                        depth = cv2.imread(depth_path, cv2.IMREAD_UNCHANGED)
                        depth_vis = cv2.convertScaleAbs(depth, alpha=0.03)  # type: ignore
                        depth_vis = cv2.applyColorMap(depth_vis, cv2.COLORMAP_JET)
                    else:
                        depth_vis = np.zeros((H, W, 3), dtype=np.uint8)

                    depth_row[:, x : x + W] = depth_vis
                    x += W + pad

                final = np.zeros((final_height, final_width, 3), dtype=np.uint8)
                final[0:H, 0:total_rgb_width] = rgb_row
                final[H + pad : H + pad + H, 0:total_depth_width] = depth_row
            else:
                final = rgb_row

            writer.write(final)

        writer.release()
        print(f"[AsyncWriter] 🎉 Preview saved: {output_path}")

    # --------------------------- MISSING FRAME CHECK --------------------------- #
    def _check_missing_frames(self):
        print("[AsyncWriter] 🔍 Checking missing frames...")

        missing_report = {}
        for cfg in self.camera_config:
            cam = cfg["name"]
            if not cfg.get("has_rgb", False):
                continue

            folder = os.path.join(self.episode_dir, f"rgb_{cam}")
            files = sorted(os.listdir(folder))
            expected = list(range(self.frame_id))
            found = [int(f.split(".")[0]) for f in files]
            missing = sorted(set(expected) - set(found))
            if missing:
                missing_report[cam] = missing

        if missing_report:
            print("❌ Missing frames:")
            print(json.dumps(missing_report, indent=2))
        else:
            print("✅ RGB frames complete.")

    # ------------------------------- STOP EPISODE ------------------------------- #
    def stop(self):
        if not self.running:
            return

        self.running = False
        self.stop_event.set()
        if self.writer_thread:
            self.writer_thread.join()

        # robot_state.json
        robot_state_dict = {
            "timestamps": self.timestamps,
            "states": self.robot_states,
        }
        with open(os.path.join(self.episode_dir, "robot_state.json"), "w", encoding="utf-8") as f:
            json.dump(robot_state_dict, f, indent=2)

        # meta.json
        meta = {
            "episode_dir": self.episode_dir,
            "timestamp": datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
            "frequency": self.freq,
            "total_frames": self.frame_id,
            "gripper_open_width": self.gripper_open,
            "gripper_close_width": self.gripper_close,
            "camera_config": self.camera_config,
            "robot_state_keys": list(self.robot_states[0].keys()) if self.robot_states else [],
            "rgb_format": "jpg",
            "depth_format": "png16",  # ⭐ 记录 16bit PNG
        }

        with open(os.path.join(self.episode_dir, "meta.json"), "w", encoding="utf-8") as f:
            json.dump(meta, f, indent=2)

        print(f"[AsyncWriter] 💾 Episode saved: {self.episode_dir}")

        # 预览视频（可禁用）
        self._make_preview_video()

        # 帧检查
        self._check_missing_frames()

        print("[AsyncWriter] ✅ All done.")
