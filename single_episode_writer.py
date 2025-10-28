import os
import cv2
import json
import datetime
import numpy as np
import time

from pickle import loads
from collections import defaultdict
from moviepy.editor import ImageSequenceClip

class EpisodeWriter(object):
    def __init__(self, data_dir, task, close_width=0, open_width=0.07, frequency=20, version='1.0.0', date=None, operator=None):
        """
        main_img_size: [width, height]
        """
        print("==> EpisodeWriter initializing...\n")
        self.data_dir = data_dir
        self.task = task
        self.close_width = close_width
        self.open_width = open_width
        self.frequency = frequency
        
        self.data = {}
        self.episode_data = []
        self.item_id = -1
        self.episode_id = -1
        if os.path.exists(self.data_dir):
            episode_dirs = [episode_dir for episode_dir in os.listdir(self.data_dir) if 'episode_' in episode_dir]
            if len(episode_dirs) > 0:
                episode_last = sorted(episode_dirs)[-1]
                self.episode_id = int(episode_last.split('_')[-1])
                print(f"==> Data directory already exists ({self.episode_id+1} episodes).\n")
            else:
                print(f"==> An empty data directory exists.\n")
        else:
            os.makedirs(self.data_dir)
            print(f"==> Data directory does not exist, now create one.\n")
        
        self.version = version
        self.date = date
        self.operator = operator
        print("==> EpisodeWriter initialized successfully.\n")

    def create_episode(self):
        """
        Create a new episode, each episode needs to specify the episode_id.
        """

        self.item_id = -1
        self.episode_data = []
        self.episode_id = self.episode_id + 1
        
        self.episode_dir = os.path.join(self.data_dir, f"episode_{str(self.episode_id).zfill(4)}")
        self.color_dir = os.path.join(self.episode_dir, 'colors')
        self.depth_dir = os.path.join(self.episode_dir, 'depths')
        self.video_dir = os.path.join(self.episode_dir, 'videos')
        self.json_path = os.path.join(self.episode_dir, 'data.json')
        os.makedirs(self.episode_dir, exist_ok=True)
        os.makedirs(self.color_dir, exist_ok=True)
        os.makedirs(self.depth_dir, exist_ok=True)
        os.makedirs(self.video_dir, exist_ok=True)
        
    def add_item(self, main_vision_data, wrist_vision_data, robot_data):
        self.episode_data.append((main_vision_data, wrist_vision_data, robot_data))

    def save_episode(self):
        """
            with open("./hmm.json",'r',encoding='utf-8') as json_file:
                model=json.load(json_file)
        """
        self.data['info'] = {
            "version": "1.0.0" if self.version is None else self.version, 
            "datetime": datetime.datetime.now().strftime("%Y-%m-%d-%H:%M") if self.date is None else self.date, 
            "operator": "someone" if self.operator is None else self.operator,
            "frequency": self.frequency,
            "close_width": self.close_width,
            "open_width": self.open_width,
            "episode_dir": self.episode_dir,
            "episode_id": self.episode_id,
            "task": self.task,
            "total_steps": len(self.episode_data),
        }
        episode_items = []
        video_dict = defaultdict(list)
        for (main_vision_data, wrist_vision_data, robot_data) in self.episode_data:
            self.item_id += 1
            item_data = {
                'idx': self.item_id,
                'colors': {'rgb_main': None, 'rgb_wrist_0': None},
                'depths': {'depth_main': None},
            }
            
            combine_img = np.zeros((self.main_img_size[1]+self.wrist_img_size[1], self.wrist_img_size[0]*2, 3), dtype=np.uint8)
            main_x_axis_shif = (self.wrist_img_size[0]*2 - self.main_img_size[0]) // 2

            # 主视觉
            rgb_main_data, depth_main_data = main_vision_data
            # rgb_main
            rgb_data = cv2.imdecode(rgb_main_data, cv2.IMREAD_COLOR)
            combine_img[:self.main_img_size[1], main_x_axis_shif:main_x_axis_shif+self.main_img_size[0]] = rgb_data
            save_dir = self.color_dir
            save_name = f'{str(self.item_id).zfill(6)}_rgb_main.jpg'
            cv2.imwrite(os.path.join(save_dir, save_name), rgb_data)
            item_data['colors']['rgb_main'] = os.path.join('colors', save_name)
            # depth_main
            depth_data = np.frombuffer(depth_main_data, dtype=np.uint16)
            save_dir = self.depth_dir
            save_name = f'{str(self.item_id).zfill(6)}_depth_main.npy'
            np.save(os.path.join(save_dir, save_name), depth_data)
            item_data['depths']['depth_main'] = os.path.join('depths', save_name)

            # 腕部视觉
            for (k, v) in wrist_vision_data.items():
                save_dir = self.color_dir
                save_name = f'{str(self.item_id).zfill(6)}_{k}.jpg'
                rgb_data = v
                wrist_id = int(k.split('_')[-1])
                x_axis_shif = wrist_id * self.wrist_img_size[0]
                combine_img[self.main_img_size[1]:, x_axis_shif:x_axis_shif+self.wrist_img_size[0]] = rgb_data
                cv2.imwrite(os.path.join(save_dir, save_name), rgb_data)
                item_data['colors'][k] = os.path.join('colors', save_name)
            video_dict['combine'].append(combine_img)
            
            item_data.update(robot_data)

            episode_items.append(item_data)
        self.data['data'] = episode_items
        
        for k, v in video_dict.items():
            images = [img[:,:,::-1] for img in v]
            video = ImageSequenceClip(images, fps=self.frequency)
            video.write_videofile(os.path.join(self.video_dir, k)+'.mp4', fps=self.frequency, logger=None)
        with open(self.json_path,'w',encoding='utf-8') as jsonf:
            jsonf.write(json.dumps(self.data, indent=2, ensure_ascii=False))
        