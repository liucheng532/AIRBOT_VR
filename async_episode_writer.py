import os
import cv2
import json
import time
import datetime
import threading
import numpy as np
import queue
import mediapy


class AsyncEpisodeWriter:
    def __init__(
        self, data_root, camera_config,
        # task="pick up the carton",
        # task="pick up the banana peel",
        # task="pick up the bottle",
        # task="throw the carton into the blue trash can",
        # task="throw the banana into the green trash can",
        task="throw the bottle into the blue trash can",
        # task="pick up the kettle",
        # task="place the kettle under the water tap",
        # task="turn on the water tap",
        # task="turn off the water tap",
        # task="pick up the kettle with water",
        # task="water the flower",
        # task="place the kettle on the table",
        freq=30, gripper_open=0.07, gripper_close=0.0, max_queue=200, make_preview=True, preview_downsample_rate=3
    ):  # ⭐ 你可以选择是否生成视频预览
        self.data_root = data_root
        self.camera_config = camera_config

        self.freq = freq
        self.gripper_open = gripper_open
        self.gripper_close = gripper_close
        self.make_preview = make_preview  # ⭐ 控制是否生成视频
        self.preview_downsample_rate = preview_downsample_rate  # ⭐ 预览视频降采样率

        self.task = task

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
        # 统计 data_root 中现有的 episode 文件夹数量
        existing_episodes = 0
        if os.path.exists(self.data_root):
            for item in os.listdir(self.data_root):
                item_path = os.path.join(self.data_root, item)
                if os.path.isdir(item_path) and item.startswith("episode_"):
                    existing_episodes += 1
        
        # 使用现有 episode 数量来命名新文件夹
        self.episode_dir = os.path.join(self.data_root, f"episode_{existing_episodes}")
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
        """多相机预览视频，仅使用rgb_cams，垂直拼接，以最宽图像宽度为帧宽度"""
        if not self.make_preview:
            print("[AsyncWriter] ⏭ Skip preview video.")
            return

        print(f"[AsyncWriter] 🎬 Building multi-camera preview video (downsample rate: {self.preview_downsample_rate})...")

        rgb_cams = [c["name"] for c in self.camera_config if c.get("has_rgb", False)]

        if len(rgb_cams) == 0:
            print("[AsyncWriter] ⚠️ No RGB cameras for preview.")
            return

        # 找到第一帧，确定每个相机的分辨率，并找到最大宽度
        max_width = 0
        cam_shapes = {}
        for cam in rgb_cams:
            img_path = os.path.join(self.episode_dir, f"rgb_{cam}", "0.jpg")
            if os.path.exists(img_path):
                img = cv2.imread(img_path)
                if img is not None:
                    h, w = img.shape[:2]
                    cam_shapes[cam] = (h, w)
                    max_width = max(max_width, w)
        
        if max_width == 0:
            print("[AsyncWriter] ⚠️ Cannot find first frame or determine image sizes.")
            return

        # MP4格式，低质量设置
        episode_name = os.path.basename(self.episode_dir)
        output_path = os.path.join(self.episode_dir, f"{episode_name}_preview.mp4")
        # 降采样后的帧率
        preview_fps = float(self.freq) / self.preview_downsample_rate

        # 拼接视频（降采样），收集所有帧
        frames = []
        for frame_id in range(0, self.frame_id, self.preview_downsample_rate):
            # 收集所有rgb图像，不resize较小宽度的图像，而是放在max_width的画布上
            processed_imgs = []
            for cam in rgb_cams:
                img_path = os.path.join(self.episode_dir, f"rgb_{cam}", f"{frame_id}.jpg")
                if os.path.exists(img_path):
                    img = cv2.imread(img_path)
                    if img is not None:
                        h, w = img.shape[:2]
                        # 不resize，保持原始尺寸，放在max_width的画布上（左对齐）
                        canvas = np.zeros((h, max_width, 3), dtype=np.uint8)
                        canvas[:, :w] = img
                        processed_imgs.append(canvas)
                    else:
                        # 如果读取失败，使用第一帧的尺寸创建黑色图像
                        h, w = cam_shapes.get(cam, (480, 640))
                        canvas = np.zeros((h, max_width, 3), dtype=np.uint8)
                        processed_imgs.append(canvas)
                else:
                    # 如果文件不存在，使用第一帧的尺寸创建黑色图像
                    h, w = cam_shapes.get(cam, (480, 640))
                    canvas = np.zeros((h, max_width, 3), dtype=np.uint8)
                    processed_imgs.append(canvas)

            # 垂直拼接所有图像
            if processed_imgs:
                final = np.vstack(processed_imgs)
            else:
                final = np.zeros((480, max_width, 3), dtype=np.uint8)

            # 将 BGR 转换为 RGB（OpenCV 使用 BGR，mediapy 需要 RGB）
            final_rgb = cv2.cvtColor(final, cv2.COLOR_BGR2RGB)
            frames.append(final_rgb)

        # 使用 mediapy 写入视频
        frame_count = len(frames)
        if frame_count > 0:
            mediapy.write_video(output_path, frames, fps=preview_fps)
        print(f"[AsyncWriter] 🎉 Preview saved: {output_path} ({frame_count} frames at {preview_fps:.1f} fps)")

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
            "task": self.task,
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
