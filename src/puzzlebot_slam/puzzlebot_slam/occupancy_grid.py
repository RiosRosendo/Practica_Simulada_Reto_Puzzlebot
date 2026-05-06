import numpy as np


class OccupancyGrid:

    def __init__(self, width, height, resolution,
                 l_occ=0.85, l_free=-0.40, l_min=-5.0, l_max=5.0):
        self.width      = width       # cells
        self.height     = height      # cells
        self.resolution = resolution  # m/cell

        self.l_occ  = l_occ
        self.l_free = l_free
        self.l_min  = l_min
        self.l_max  = l_max

        # Log-odds grid, initialised to 0 (unknown)
        self._log = np.zeros((height, width), dtype=np.float32)

        # Robot starts at the centre of the map
        self.origin_x = -(width  * resolution) / 2.0
        self.origin_y = -(height * resolution) / 2.0

    # ------------------------------------------------------------------
    # Coordinate helpers
    # ------------------------------------------------------------------

    def world_to_cell(self, x, y):
        cx = int((x - self.origin_x) / self.resolution)
        cy = int((y - self.origin_y) / self.resolution)
        return cx, cy

    def in_bounds(self, cx, cy):
        return 0 <= cx < self.width and 0 <= cy < self.height

    # ------------------------------------------------------------------
    # Bresenham ray-casting update
    # ------------------------------------------------------------------

    def update_scan(self, robot_x, robot_y, robot_theta,
                    ranges, angle_min, angle_increment,
                    range_min, range_max,
                    laser_x=0.0, laser_y=0.0, laser_yaw=0.0,
                    scan_omega=0.0, scan_time=0.0):
        """
        Update the grid for every ray in the laser scan.

        (laser_x, laser_y, laser_yaw) is the static SE(2) offset of the
        LiDAR frame relative to the robot's base frame (from TF).

        scan_omega [rad/s] and scan_time [s] enable within-scan rotation
        deskewing: each ray's effective robot heading is corrected by the
        rotation that occurred between the scan's stamp and that ray's
        capture instant.  Pass scan_omega=0 to disable.
        """
        n = len(ranges)
        if n == 0:
            return

        # Composed yaw at the scan's reference instant
        base_yaw = robot_theta + laser_yaw

        # Per-ray heading offset from in-scan rotation.  For RPLidar A1 the
        # scan stamp is the first ray; ray i was captured `(i/N) * scan_time`
        # later, by which point the robot has rotated `omega * dt`.
        if scan_omega != 0.0 and scan_time > 0.0 and n > 1:
            ray_dt = (np.arange(n, dtype=np.float64) / (n - 1)) * scan_time
            yaw_offsets = scan_omega * ray_dt
        else:
            yaw_offsets = np.zeros(n)

        # The LiDAR origin also shifts in-scan during rotation (small for
        # the Puzzlebot's 5 cm offset, but compute it for correctness).
        cT0, sT0 = np.cos(robot_theta), np.sin(robot_theta)
        ox0 = robot_x + cT0 * laser_x - sT0 * laser_y
        oy0 = robot_y + sT0 * laser_x + cT0 * laser_y

        for i, r in enumerate(ranges):
            if r < range_min or not np.isfinite(r):
                continue

            hit = r < range_max
            ray_range = r if hit else range_max

            # Per-ray-deskewed robot heading and laser origin
            yaw_off = yaw_offsets[i]
            if yaw_off != 0.0:
                cT = np.cos(robot_theta + yaw_off)
                sT = np.sin(robot_theta + yaw_off)
                ox = robot_x + cT * laser_x - sT * laser_y
                oy = robot_y + sT * laser_x + cT * laser_y
            else:
                ox, oy = ox0, oy0
            rx, ry = self.world_to_cell(ox, oy)

            angle = base_yaw + yaw_off + angle_min + i * angle_increment
            ex = ox + ray_range * np.cos(angle)
            ey = oy + ray_range * np.sin(angle)
            ex_c, ey_c = self.world_to_cell(ex, ey)

            # Trace free cells along ray with Bresenham
            for cx, cy in self._bresenham(rx, ry, ex_c, ey_c):
                if not self.in_bounds(cx, cy):
                    break
                self._log[cy, cx] = np.clip(
                    self._log[cy, cx] + self.l_free, self.l_min, self.l_max)

            # Mark endpoint as occupied
            if hit and self.in_bounds(ex_c, ey_c):
                self._log[ey_c, ex_c] = np.clip(
                    self._log[ey_c, ex_c] + self.l_occ, self.l_min, self.l_max)

    @staticmethod
    def _bresenham(x0, y0, x1, y1):
        """Yield integer (x, y) cells from (x0,y0) to (x1,y1)."""
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy

        while True:
            yield x0, y0
            if x0 == x1 and y0 == y1:
                break
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x0  += sx
            if e2 < dx:
                err += dx
                y0  += sy

    # ------------------------------------------------------------------
    # Serialise for ROS
    # ------------------------------------------------------------------

    def to_ros_data(self):
        """
        Return flat int8 list for OccupancyGrid.data.
          log < l_free  →   0 (free)
          log > l_occ   → 100 (occupied)
          else          →  -1 (unknown)
        """
        out = np.full((self.height, self.width), -1, dtype=np.int8)
        out[self._log >  self.l_occ]  = 100
        out[self._log <  self.l_free] = 0
        return out.flatten().tolist()
