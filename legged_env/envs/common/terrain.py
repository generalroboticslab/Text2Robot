# terrain generator
# from isaacgym.terrain_utils import *
import numpy as np
from collections import Mapping
from typing import Union

from isaacgym import gymapi
import torch

from isaacgym.terrain_utils import (
    random_uniform_terrain,
    pyramid_sloped_terrain,
    discrete_obstacles_terrain,
    pyramid_stairs_terrain,
    stepping_stones_terrain,
    convert_heightfield_to_trimesh,
    SubTerrain,
)


class Terrain:
    def __init__(self, cfg: Mapping, num_robots: int, device: Union[torch.device, str], gym, sim) -> None:
        """
        Initializes the Terrain object with the given configuration parameters.

        Parameters:
            cfg (Mapping): A dictionary containing the configuration parameters for the terrain.
            num_robots (int): The number of robots in the environment.
            device (Union[torch.device, str]): The device to use for computation.
            gym (GymApi): The GymApi object for creating the terrain.
            sim (gymapi.Sim): The GymApi simulation object.

        Returns:
            None
        """
        self.device = device

        self.terrain_type: str = cfg["terrainType"]
        self.horizontal_scale: float = cfg["horizontalScale"]
        self.vertical_scale: float = cfg["verticalScale"]
        self.border_size: float = cfg["borderSize"]
        self.env_length: float = cfg["mapLength"]
        self.env_width: float = cfg["mapWidth"]
        self.env_rows: int = cfg["numLevels"]
        self.env_cols: int = cfg["numTerrains"]
        self.difficulty_scale: float = cfg.get("difficultySale", 1.0)  # difficulty multiplier
        self.platform_size: float = cfg["platformSize"]

        self.stair_width: float = cfg["stair"]["width"]
        self.stair_height: float = cfg["stair"]["height"]

        self.uniform_height: float = cfg["uniform"]["height"]
        self.uniform_step: float = cfg["uniform"]["step"]

        self.slope = cfg["slope"]

        proportions = cfg["terrainProportions"]
        if len(proportions) == 6:
            # original terrain types: [smooth slope, rough slope, stairs up, stairs down, discrete, stepping stone]
            p = proportions[:]
            proportions = [p[1], 0, 0, p[2], p[3], p[0], 0, p[4], p[5]]

            # [0] rough_up,   \⎽/  # [1] rough_down, /⎺\   # [2] rough_flat      ⎽
            # [3] stair_up,   \⎽/  # [4] stair_down, /⎺\   # [5] smooth_up,     \⎽/
            # [6] smooth_down,/⎺\  # [7] discrete,   ☵☷   # [8] stepping_stone ▦▦

        self.proportions = np.cumsum(proportions) / np.sum(proportions)

        self.num_maps = self.env_rows * self.env_cols
        self.num_per_env = int(num_robots / self.num_maps)
        # sub terrain env origin
        self.env_origins = torch.zeros((self.env_rows, self.env_cols, 3), dtype=torch.float, device=self.device)

        self.width_per_env_pixels = int(self.env_width / self.horizontal_scale)
        self.length_per_env_pixels = int(self.env_length / self.horizontal_scale)

        self.border = int(self.border_size / self.horizontal_scale)
        self.tot_cols = int(self.env_cols * self.width_per_env_pixels) + 2 * self.border
        self.tot_rows = int(self.env_rows * self.length_per_env_pixels) + 2 * self.border

        self.height_field_raw = np.zeros((self.tot_rows, self.tot_cols), dtype=np.int16)  # raw height field

        if cfg["curriculum"]:  # generate height field
            self.curriculum(num_robots, num_terrains=self.env_cols, num_levels=self.env_rows, randomize=False)
        else:
            self.curriculum(num_robots, num_terrains=self.env_cols, num_levels=self.env_rows, randomize=True)

        def set_common_params(params, offset=-self.border_size):
            if offset:
                params.transform.p.x = offset
                params.transform.p.y = offset
                params.transform.p.z = 0.0
            params.static_friction = cfg["staticFriction"]
            params.dynamic_friction = cfg["dynamicFriction"]
            params.restitution = cfg["restitution"]

        if self.terrain_type == "trimesh":
            # # create trimesh
            self.vertices, self.triangles = convert_heightfield_to_trimesh(
                self.height_field_raw, self.horizontal_scale, self.vertical_scale, cfg["slopeTreshold"]
            )
            params = gymapi.TriangleMeshParams()
            params.nb_vertices = self.vertices.shape[0]
            params.nb_triangles = self.triangles.shape[0]
            set_common_params(params)
            gym.add_triangle_mesh(sim, self.vertices.flatten(order='C'), self.triangles.flatten(order='C'), params)
        elif self.terrain_type == "heightfield":
            # create height field
            params = gymapi.HeightFieldParams()
            params.column_scale = self.horizontal_scale
            params.row_scale = self.horizontal_scale
            params.vertical_scale = self.vertical_scale
            params.nbRows = self.tot_cols
            params.nbColumns = self.tot_rows
            set_common_params(params)
            gym.add_heightfield(sim, self.height_field_raw.ravel('F'), params)
        elif self.terrain_type == "plane":
            params = gymapi.PlaneParams()
            params.normal = gymapi.Vec3(0.0, 0.0, 1.0)
            set_common_params(params, offset=0)
            gym.add_ground(sim, params)

        # to torch
        self.height_samples = torch.tensor(self.height_field_raw, device=self.device).view(self.tot_rows, self.tot_cols)
        self.height_sample_limit = (self.height_samples.shape[0] - 2, self.height_samples.shape[1] - 2)

    def curriculum(self, num_robots, num_terrains, num_levels, randomize):
        """
        Generates a heght field for the terrain based on the given parameters.

        Parameters:
            num_robots (int): The total number of robots.
            num_terrains (int): The total number of terrains.
            num_levels (int): The total number of levels.
            randomize (bool): Whether to randomize the level and choice.

        Returns:
            None
        """
        num_robots_per_map = int(num_robots / num_terrains)
        left_over = num_robots % num_terrains
        proportions = np.round(self.proportions * self.env_cols).astype(int)
        for j in range(num_terrains):
            for i in range(num_levels):

                if randomize:
                    level = np.random.randint(0, self.env_rows)
                    choice = np.random.randint(0, num_terrains)
                else:
                    level = i
                    choice = j
                difficulty = (level + 1) / num_levels
                # difficulty = (i + np.random.uniform(low=0, high=0.5)) / num_levels
                # choice = j / num_terrains

                slope = self.slope * self.difficulty_scale * difficulty
                # obstacle_height = (0.025 + difficulty * 0.15) * self.difficulty_scale
                obstacle_height = self.stair_height * self.difficulty_scale * difficulty
                # stepping stone
                stone_size = (2 - 1.5 * difficulty) * self.difficulty_scale
                stone_distance = 0.1 * self.difficulty_scale * difficulty

                uniform_height = self.uniform_height * self.difficulty_scale * difficulty
                uniform_step = max(self.uniform_step * self.difficulty_scale, self.vertical_scale)
                # uniform_step = self.vertical_scale
                uniform_downsample = self.horizontal_scale * 2

                stair_width = self.stair_width
                stair_height = self.stair_height * self.difficulty_scale * difficulty
                # stair_height = (0.05 + 0.175 * difficulty) * self.difficulty_scale

                terrain = SubTerrain(
                    "terrain",
                    width=self.length_per_env_pixels,
                    length=self.width_per_env_pixels,
                    vertical_scale=self.vertical_scale,
                    horizontal_scale=self.horizontal_scale,
                )
                # [0] rough_up,      \⎽/
                # [1] rough_down,    /⎺\
                # [2] rough_flat      ⎽
                # [3] stair_up,      \⎽/
                # [4] stair_down,    /⎺\
                # [5] smooth_up,     \⎽/
                # [6] smooth_down,   /⎺\
                # [7] discrete,      ☵☷
                # [8] stepping_stone  ▦▦

                # terrain types: [rough_up, rough_down, rough_flat, stair_up,
                # stair_down, smooth_up, smooth_down, discrete, stepping_stone]
                if choice < proportions[0]:  # [0] rough_up,      \⎽/
                    pyramid_sloped_terrain(terrain, slope=slope, platform_size=self.platform_size)
                    random_uniform_terrain(terrain, -uniform_height, uniform_height, uniform_step, uniform_downsample)
                elif choice < proportions[1]:  # [1] rough_down,    /⎺\
                    pyramid_sloped_terrain(terrain, slope=-slope, platform_size=self.platform_size)
                    random_uniform_terrain(terrain, -uniform_height, uniform_height, uniform_step, uniform_downsample)
                elif choice < proportions[2]:  # [2] rough_flat      ⎽
                    random_uniform_terrain(terrain, -uniform_height, uniform_height, uniform_step, uniform_downsample)
                elif choice < proportions[3]:  # [3] stair_up,      \⎽/
                    pyramid_stairs_terrain(
                        terrain, step_width=stair_width, step_height=stair_height, platform_size=self.platform_size
                    )
                elif choice < proportions[4]:  # [4] stair_down,    /⎺\
                    pyramid_stairs_terrain(
                        terrain, step_width=stair_width, step_height=-stair_height, platform_size=self.platform_size
                    )
                elif choice < proportions[5]:  # [5] smooth_up,     \⎽/
                    pyramid_sloped_terrain(terrain, slope=-slope, platform_size=self.platform_size)
                elif choice < proportions[6]:  # [6] smooth_down,   /⎺\
                    pyramid_sloped_terrain(terrain, slope=slope, platform_size=self.platform_size)
                elif choice < proportions[7]:  # [7] discrete,      ☵☷
                    discrete_obstacles_terrain(terrain, obstacle_height, 1.0, 2.0, 40, platform_size=self.platform_size)
                else:  # [8] stepping_stone  ▦▦
                    stepping_stones_terrain(
                        terrain,
                        stone_size=stone_size,
                        stone_distance=stone_distance,
                        max_height=0.0,
                        platform_size=self.platform_size,
                        depth=-1,
                    )

                # Heightfield coordinate system
                start_x = self.border + i * self.length_per_env_pixels
                end_x = self.border + (i + 1) * self.length_per_env_pixels
                start_y = self.border + j * self.width_per_env_pixels
                end_y = self.border + (j + 1) * self.width_per_env_pixels
                self.height_field_raw[start_x:end_x, start_y:end_y] = terrain.height_field_raw

                robots_in_map = num_robots_per_map
                if j < left_over:
                    robots_in_map += 1

                env_origin_x = (i + 0.5) * self.env_length
                env_origin_y = (j + 0.5) * self.env_width
                x1 = int((self.env_length / 2.0 - 1) / self.horizontal_scale)
                x2 = int((self.env_length / 2.0 + 1) / self.horizontal_scale)
                y1 = int((self.env_width / 2.0 - 1) / self.horizontal_scale)
                y2 = int((self.env_width / 2.0 + 1) / self.horizontal_scale)
                env_origin_z = np.max(terrain.height_field_raw[x1:x2, y1:y2]) * self.vertical_scale
                self.env_origins[i, j] = torch.tensor([env_origin_x, env_origin_y, env_origin_z])

    def get_heights(self, points: torch.Tensor):
        points_ = ((points + self.border_size) / self.horizontal_scale).long()
        px = torch.clip(points_[..., 0].view(-1), 0, self.height_sample_limit[0])
        py = torch.clip(points_[..., 1].view(-1), 0, self.height_sample_limit[1])
        heights1 = self.height_samples[px, py]
        heights2 = self.height_samples[px + 1, py + 1]
        heights = torch.min(heights1, heights2) * self.vertical_scale
        return heights.reshape(points.shape[:-1])
