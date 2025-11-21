import pyrealsense2 as rs
import numpy as np
import cv2
import time
import threading
from queue import Queue
from single_episode_writer import EpisodeWriter


MAIN_SERIAL = "317222075228"
WRIST_SERIAL = "943222073615"


# ===== 初始化 EpisodeWriter =====
recorder = EpisodeWriter(
    data_dir="zzj_test/record",
    task="dual_realsense_test",
    close_width=0.0,
    open_width=0.07,
    frequency=30,
)
recorder.create_episode()
recorder.main_img_size = (640, 480)
recorder.wrist_img_size = (640, 480)

# ===== 异步写入线程 =====
write_queue = Queue(maxsize=50)
stop_flag = False

def writer_thread_func():
    while not stop_flag or not write_queue.empty():
        try:
            main_data, wrist_data, robot_data = write_queue.get(timeout=1)
            recorder.add_item(main_data, wrist_data, robot_data)
        except:
            continue

writer_thread = threading.Thread(target=writer_thread_func)
writer_thread.start()

# ===== 相机配置 =====
pipeline_main = rs.pipeline()
config_main = rs.config()
config_main.enable_device(MAIN_SERIAL)
config_main.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config_main.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
align_main = rs.align(rs.stream.color)

pipeline_wrist = rs.pipeline()
config_wrist = rs.config()
config_wrist.enable_device(WRIST_SERIAL)
config_wrist.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

print("🎥 启动 RealSense 双相机...")
pipeline_main.start(config_main)
time.sleep(2)
pipeline_wrist.start(config_wrist)
print("✅ 两台相机已启动")

# ===== 录制循环 =====
frame_count = 0
total_frames = 150
print(f"⏳ 开始录制 {total_frames / 30:.1f} 秒视频...")

try:
    while frame_count < total_frames:
        # 主相机帧
        frames_main = pipeline_main.wait_for_frames(timeout_ms=1000)
        aligned_main = align_main.process(frames_main)
        color_main = aligned_main.get_color_frame()
        depth_main = aligned_main.get_depth_frame()

        # 腕部相机帧
        frames_wrist = pipeline_wrist.poll_for_frames()
        color_wrist = frames_wrist.get_color_frame() if frames_wrist else None

        if not color_main or not depth_main or not color_wrist:
            print(f"⚠️ 第 {frame_count} 帧不完整，跳过。")
            continue

        # 立即拷贝数据（释放 SDK 缓冲）
        color_main_np = np.asanyarray(color_main.get_data()).copy()
        depth_np = np.asanyarray(depth_main.get_data()).copy()
        color_wrist_np = np.asanyarray(color_wrist.get_data()).copy()

        # 编码后入队
        _, color_encoded = cv2.imencode('.jpg', color_main_np)
        depth_bytes = depth_np.tobytes()
        main_data = (color_encoded, depth_bytes)
        wrist_data = {"rgb_wrist_0": color_wrist_np}
        robot_data = {"states": {}, "actions": {}}

        try:
            write_queue.put_nowait((main_data, wrist_data, robot_data))
        except:
            print("⚠️ 写入队列满，丢帧。")

        if frame_count % 10 == 0:
            print(f"📸 已录制 {frame_count}/{total_frames} 帧")

        frame_count += 1

    print("✅ 录制完成，等待写入线程结束...")

except Exception as e:
    print(f"❌ 捕获异常: {e}")

finally:
    stop_flag = True
    writer_thread.join(timeout=5)
    pipeline_main.stop()
    pipeline_wrist.stop()
    recorder.save_episode()
    print("💾 Episode 保存完成。")
    print("✅ 相机资源已释放。")
