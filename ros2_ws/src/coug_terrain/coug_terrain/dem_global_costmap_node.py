# Copyright 2026 BYU FROST Lab
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import math

import numpy as np
import pymap3d as pm
import rclpy
from nav_msgs.msg import OccupancyGrid
from osgeo import gdal, osr
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_system_default,
)
from sensor_msgs.msg import NavSatFix

_UNKNOWN = -1
_FREE = 0
_LETHAL = 100

gdal.UseExceptions()


class DemGlobalCostmapNode(Node):
    def __init__(self) -> None:
        super().__init__("dem_global_costmap_node")

        self.declare_parameter("dem_file", "")
        self.declare_parameter("max_slope_degrees", 25.0)
        self.declare_parameter("resolution", 1.0)
        self.declare_parameter("outside_is_lethal", False)
        self.declare_parameter("origin_topic", "/origin")
        self.declare_parameter("output_topic", "terrain/occupancy")
        self.declare_parameter("map_frame", "map")

        dem_file = self.get_parameter("dem_file").value
        self._max_slope_degrees = self.get_parameter("max_slope_degrees").value
        self._resolution = self.get_parameter("resolution").value
        self._outside_is_lethal = self.get_parameter("outside_is_lethal").value
        origin_topic = self.get_parameter("origin_topic").value
        output_topic = self.get_parameter("output_topic").value
        self._map_frame = self.get_parameter("map_frame").value

        self._published = False

        if dem_file:
            self._slope, self._lat, self._lon = self._load_dem(dem_file)

            latched_qos = QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                history=HistoryPolicy.KEEP_LAST,
            )

            self._origin_sub = self.create_subscription(
                NavSatFix, origin_topic, self._origin_callback, qos_profile_system_default
            )
            self._output_pub = self.create_publisher(OccupancyGrid, output_topic, latched_qos)
        else:
            self.get_logger().info("No 'dem_file' set. Terrain prior disabled.")

        self.get_logger().info("Initialization complete.")

    def _load_dem(self, path: str) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        dem_dataset = gdal.Open(path, gdal.GA_ReadOnly)
        slope_dataset = gdal.DEMProcessing(
            "", dem_dataset, "slope", format="MEM", computeEdges=True
        )
        slope_band = slope_dataset.GetRasterBand(1)
        slope = slope_band.ReadAsArray()

        nodata_value = slope_band.GetNoDataValue()
        if nodata_value is not None:
            slope[slope == nodata_value] = np.nan

        width, height = dem_dataset.RasterXSize, dem_dataset.RasterYSize
        geo_transform = dem_dataset.GetGeoTransform()

        cols, rows = np.meshgrid(np.arange(width) + 0.5, np.arange(height) + 0.5)
        raster_x = geo_transform[0] + cols * geo_transform[1] + rows * geo_transform[2]
        raster_y = geo_transform[3] + cols * geo_transform[4] + rows * geo_transform[5]

        raster_crs = osr.SpatialReference(wkt=dem_dataset.GetProjection())
        wgs84_crs = osr.SpatialReference()
        wgs84_crs.ImportFromEPSG(4326)
        raster_crs.SetAxisMappingStrategy(osr.OAMS_TRADITIONAL_GIS_ORDER)
        wgs84_crs.SetAxisMappingStrategy(osr.OAMS_TRADITIONAL_GIS_ORDER)
        to_wgs84 = osr.CoordinateTransformation(raster_crs, wgs84_crs)

        lon_lat = np.asarray(
            to_wgs84.TransformPoints(np.column_stack([raster_x.ravel(), raster_y.ravel()]).tolist())
        )
        lon = lon_lat[:, 0].reshape(height, width)
        lat = lon_lat[:, 1].reshape(height, width)

        self.get_logger().info(
            f"DEM loaded: {width}x{height} at {geo_transform[1]:.2f} m/px, "
            f"slope {np.nanmin(slope):.1f}-{np.nanmax(slope):.1f} deg."
        )
        return slope, lat, lon

    def _origin_callback(self, msg: NavSatFix) -> None:
        if self._published:
            return

        east, north, _ = pm.geodetic2enu(
            self._lat, self._lon, 0.0, msg.latitude, msg.longitude, 0.0
        )

        resolution = self._resolution
        min_east, max_east = float(np.min(east)), float(np.max(east))
        min_north, max_north = float(np.min(north)), float(np.max(north))
        width = max(1, math.ceil((max_east - min_east) / resolution))
        height = max(1, math.ceil((max_north - min_north) / resolution))

        col_idx = np.clip(((east - min_east) / resolution).astype(np.int32), 0, width - 1)
        row_idx = np.clip(((north - min_north) / resolution).astype(np.int32), 0, height - 1)

        grid = np.full((height, width), _UNKNOWN, dtype=np.int8)
        is_valid = np.isfinite(self._slope)
        cell_indices = row_idx[is_valid] * width + col_idx[is_valid]
        is_steep = self._slope[is_valid] > self._max_slope_degrees
        flat_grid = grid.reshape(-1)
        flat_grid[cell_indices] = _FREE
        flat_grid[cell_indices[is_steep]] = _LETHAL

        if self._outside_is_lethal:
            grid[grid == _UNKNOWN] = _LETHAL

        self.get_logger().info(
            f"Costmap published: {width}x{height} at {resolution:.2f} m, "
            f"{int(np.count_nonzero(grid == _LETHAL))} lethal, "
            f"{int(np.count_nonzero(grid == _FREE))} free, "
            f"{int(np.count_nonzero(grid == _UNKNOWN))} unknown."
        )
        occupancy_grid_msg = OccupancyGrid()
        occupancy_grid_msg.header.stamp = self.get_clock().now().to_msg()
        occupancy_grid_msg.header.frame_id = self._map_frame
        occupancy_grid_msg.info.resolution = resolution
        occupancy_grid_msg.info.width = width
        occupancy_grid_msg.info.height = height
        occupancy_grid_msg.info.origin.position.x = min_east
        occupancy_grid_msg.info.origin.position.y = min_north
        occupancy_grid_msg.info.origin.orientation.w = 1.0
        occupancy_grid_msg.data = grid.ravel().tolist()

        self._output_pub.publish(occupancy_grid_msg)
        self._published = True

        self.get_logger().info(f"DEM anchored: Lat {msg.latitude:.6f}, Lon {msg.longitude:.6f}")


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    dem_global_costmap_node = DemGlobalCostmapNode()
    try:
        rclpy.spin(dem_global_costmap_node)
    except KeyboardInterrupt:
        pass
    finally:
        dem_global_costmap_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
