import numpy as np

# @njit(fastmath=False, cache=True)
# def nearest_point_on_trajectory(point, trajectory):
#     """
#     Return the nearest point along the given piecewise linear trajectory.

#     Same as nearest_point_on_line_segment, but vectorized. This method is quite fast, time constraints should
#     not be an issue so long as trajectories are not insanely long.

#         Order of magnitude: trajectory length: 1000 --> 0.0002 second computation (5000fps)

#     point: size 2 numpy array
#     trajectory: Nx2 matrix of (x,y) trajectory waypoints
#         - these must be unique. If they are not unique, a divide by 0 error will destroy the world
#     """
#     diffs = trajectory[1:,:] - trajectory[:-1,:]
#     l2s   = diffs[:,0]**2 + diffs[:,1]**2
#     # this is equivalent to the elementwise dot product
#     # dots = np.sum((point - trajectory[:-1,:]) * diffs[:,:], axis=1)
#     dots = np.empty((trajectory.shape[0]-1, ))
#     for i in range(dots.shape[0]):
#         dots[i] = np.dot((point - trajectory[i, :]), diffs[i, :])
#     t = dots / l2s
#     t[t<0.0] = 0.0
#     t[t>1.0] = 1.0
#     # t = np.clip(dots / l2s, 0.0, 1.0)
#     projections = trajectory[:-1,:] + (t*diffs.T).T
#     # dists = np.linalg.norm(point - projections, axis=1)
#     dists = np.empty((projections.shape[0],))
#     for i in range(dists.shape[0]):
#         temp = point - projections[i]
#         dists[i] = np.sqrt(np.sum(temp*temp))
#     min_dist_segment = np.argmin(dists)
#     return projections[min_dist_segment], dists[min_dist_segment], t[min_dist_segment], min_dist_segment

def nearest_point_on_trajectory(point, trajectory, eps=1e-9):
    """
    Return the nearest point along the piecewise linear trajectory.
    Robust to duplicate/near-duplicate consecutive waypoints.
    """
    traj = np.asarray(trajectory, dtype=float)

    # 1) 연속 중복(혹은 거의 같은 점) 제거
    keep = [0]
    for i in range(1, traj.shape[0]):
        if np.linalg.norm(traj[i] - traj[keep[-1]]) > eps:
            keep.append(i)
    traj = traj[keep]

    if traj.shape[0] < 2:
        # 선분이 없으면: 그 점(하나)만 있는 경우로 취급
        return traj[0], np.linalg.norm(point - traj[0]), 0.0, 0

    # 2) 세그먼트 벡터/길이
    diffs = traj[1:, :] - traj[:-1, :]
    l2s = np.einsum('ij,ij->i', diffs, diffs)  # 각 세그먼트 길이^2

    # 3) 점-세그먼트 시작점 도트 (elementwise dot)
    dots = np.empty((traj.shape[0] - 1,), dtype=float)
    for i in range(dots.shape[0]):
        dots[i] = np.dot(point - traj[i, :], diffs[i, :])

    # 4) l2s==0(이론상 없겠지만 안전빵) 보호
    t = np.zeros_like(dots)
    nonzero = l2s > eps
    t[nonzero] = dots[nonzero] / l2s[nonzero]
    # 5) 클립
    t = np.clip(t, 0.0, 1.0)

    # 6) 투영점/거리
    projections = traj[:-1, :] + (t * diffs.T).T
    dvec = projections - point
    dists = np.sqrt(np.einsum('ij,ij->i', dvec, dvec))

    min_idx = int(np.argmin(dists))
    return projections[min_idx], dists[min_idx], float(t[min_idx]), min_idx

# # @njit(fastmath=False, cache=True)
# def first_point_on_trajectory_intersecting_circle(point, radius, trajectory, t=0.0, wrap=False):
#     """
#     starts at beginning of trajectory, and find the first point one radius away from the given point along the trajectory.

#     Assumes that the first segment passes within a single radius of the point

#     http://codereview.stackexchange.com/questions/86421/line-segment-to-circle-collision-algorithm
#     """
#     start_i = int(t)
#     start_t = t % 1.0
#     first_t = None
#     first_i = None
#     first_p = None
#     trajectory = np.ascontiguousarray(trajectory)
#     for i in range(start_i, trajectory.shape[0]-1):
#         start = trajectory[i,:]
#         end = trajectory[i+1,:]+1e-6
#         V = np.ascontiguousarray(end - start)

#         a = np.dot(V,V)
#         b = 2.0*np.dot(V, start - point)
#         c = np.dot(start, start) + np.dot(point,point) - 2.0*np.dot(start, point) - radius*radius
#         discriminant = b*b-4*a*c

#         if discriminant < 0:
#             continue
#         #   print "NO INTERSECTION"
#         # else:
#         # if discriminant >= 0.0:
#         discriminant = np.sqrt(discriminant)
#         t1 = (-b - discriminant) / (2.0*a)
#         t2 = (-b + discriminant) / (2.0*a)
#         if i == start_i:
#             if t1 >= 0.0 and t1 <= 1.0 and t1 >= start_t:
#                 first_t = t1
#                 first_i = i
#                 first_p = start + t1 * V
#                 break
#             if t2 >= 0.0 and t2 <= 1.0 and t2 >= start_t:
#                 first_t = t2
#                 first_i = i
#                 first_p = start + t2 * V
#                 break
#         elif t1 >= 0.0 and t1 <= 1.0:
#             first_t = t1
#             first_i = i
#             first_p = start + t1 * V
#             break
#         elif t2 >= 0.0 and t2 <= 1.0:
#             first_t = t2
#             first_i = i
#             first_p = start + t2 * V
#             break
#     # wrap around to the beginning of the trajectory if no intersection is found1
#     if wrap and first_p is None:
#         for i in range(-1, start_i):
#             start = trajectory[i % trajectory.shape[0],:]
#             end = trajectory[(i+1) % trajectory.shape[0],:]+1e-6
#             V = end - start

#             a = np.dot(V,V)
#             b = 2.0*np.dot(V, start - point)
#             c = np.dot(start, start) + np.dot(point,point) - 2.0*np.dot(start, point) - radius*radius
#             discriminant = b*b-4*a*c

#             if discriminant < 0:
#                 continue
#             discriminant = np.sqrt(discriminant)
#             t1 = (-b - discriminant) / (2.0*a)
#             t2 = (-b + discriminant) / (2.0*a)
#             if t1 >= 0.0 and t1 <= 1.0:
#                 first_t = t1
#                 first_i = i
#                 first_p = start + t1 * V
#                 break
#             elif t2 >= 0.0 and t2 <= 1.0:
#                 first_t = t2
#                 first_i = i
#                 first_p = start + t2 * V
#                 break

#     return first_p, first_i, first_t

def first_point_on_trajectory_intersecting_circle(point, radius, trajectory, t=0.0, wrap=False, eps=1e-9):
    """
    Find first intersection of circle(center=point, radius) with the trajectory.
    Robust to zero-length segments and NaN start t.
    """
    traj = np.asarray(trajectory, dtype=float)

    # 연속 중복 제거 (nearest 함수와 동일 로직)
    keep = [0]
    for i in range(1, traj.shape[0]):
        if np.linalg.norm(traj[i] - traj[keep[-1]]) > eps:
            keep.append(i)
    traj = traj[keep]

    if traj.shape[0] < 2:
        return None, None, None

    # t NaN/inf 보호
    if not np.isfinite(t):
        t = 0.0

    start_i = int(np.floor(max(t, 0.0)))  # 음수 방지
    start_t = float(t % 1.0)

    def _scan(from_i, to_i, require_start_t=False):
        for i in range(from_i, to_i):
            start = traj[i, :]
            end = traj[i + 1, :]
            V = end - start
            a = float(np.dot(V, V))
            if a <= eps:
                # 0길이 세그먼트 스킵
                continue

            b = 2.0 * np.dot(V, start - point)
            c = (np.dot(start, start) + np.dot(point, point)
                 - 2.0 * np.dot(start, point) - radius * radius)
            disc = b * b - 4.0 * a * c
            if disc < 0.0:
                continue
            root = np.sqrt(disc)
            t1 = (-b - root) / (2.0 * a)
            t2 = (-b + root) / (2.0 * a)

            # 시작 세그먼트에서는 start_t 이후만 허용
            if require_start_t:
                if 0.0 <= t1 <= 1.0 and t1 >= start_t:
                    return start + t1 * V, i, t1
                if 0.0 <= t2 <= 1.0 and t2 >= start_t:
                    return start + t2 * V, i, t2
            else:
                if 0.0 <= t1 <= 1.0:
                    return start + t1 * V, i, t1
                if 0.0 <= t2 <= 1.0:
                    return start + t2 * V, i, t2
        return None, None, None

    # 1차: start_i부터 끝까지 스캔
    p, i_hit, t_hit = _scan(start_i, traj.shape[0] - 1, require_start_t=True)
    if p is not None:
        return p, i_hit, t_hit

    # wrap 옵션이면 처음부터 start_i-1까지
    if wrap:
        p, i_hit, t_hit = _scan(0, start_i, require_start_t=False)
        if p is not None:
            return p, i_hit, t_hit

    return None, None, None


# @njit(fastmath=False, cache=True)
def get_actuation(pose_theta, lookahead_point, position, lookahead_distance, wheelbase):
    """
    Returns actuation
    """
    waypoint_y = np.dot(np.array([np.sin(-pose_theta), np.cos(-pose_theta)]), lookahead_point[0:2]-position)
    speed = lookahead_point[2]
    if np.abs(waypoint_y) < 1e-6:
        return speed, 0.
    radius = 1/(2.0*waypoint_y/lookahead_distance**2)
    steering_angle = np.arctan(wheelbase/radius)
    return speed, steering_angle



class PurePursuitPlanner:
    """
    Example Planner
    """
    def __init__(self, ld, wb, ss, vg, mr, wp):
        self.lookahead_distance = ld 
        self.wheelbase = wb
        self.sprint_speed = ss
        self.vgain = vg
        self.max_reacquire = mr
        self.waypoint_path = wp
        
        self.waypoints = []
        self.drawn_waypoints = []

        self.wpt_vind = 5
        self.wpt_xind = 1
        self.wpt_yind = 2

        self.load_waypoints()

    def load_waypoints(self):
        """
        loads waypoints
        """
        self.waypoints = np.loadtxt(self.waypoint_path, delimiter=";", skiprows=3)

        
    # def _get_current_waypoint(self, waypoints, lookahead_distance, position, theta):
    #     """
    #     gets the current waypoint to follow
    #     """
    #     wpts = np.vstack((self.waypoints[:, self.wpt_xind], self.waypoints[:, self.wpt_yind])).T
    #     nearest_point, nearest_dist, t, i = nearest_point_on_trajectory(position, wpts)
        
        
    #     if nearest_dist < lookahead_distance:
    #         lookahead_point, i2, t2 = first_point_on_trajectory_intersecting_circle(position, lookahead_distance, wpts, i+t, wrap=True)
    #         if i2 == None:
    #             return None
    #         current_waypoint = np.empty((3, ))
    #         # x, y
    #         current_waypoint[0:2] = wpts[i2, :]
    #         # speed
    #         current_waypoint[2] = waypoints[i, self.wpt_vind]
    #         return current_waypoint
    #     elif nearest_dist < self.max_reacquire:
    #         print(self.max_reacquire)
    #         return np.append(wpts[i, :], waypoints[i, self.wpt_vind])
    #     else:
    #         return None
    def _get_current_waypoint(self, waypoints, lookahead_distance, position, theta):
        """
        gets the current waypoint to follow
        """
    # (x, y) 열 추출
        wpts = np.vstack((self.waypoints[:, self.wpt_xind], self.waypoints[:, self.wpt_yind])).T

    # (a) 현재 위치에 대한 최근접 투영점/세그먼트 찾기
        nearest_point, nearest_dist, t, i = nearest_point_on_trajectory(position, wpts)

    # (b) lookahead 원과 경로의 첫 번째 교차점 구하기
        first_p, i2, t2 = first_point_on_trajectory_intersecting_circle(
        position, lookahead_distance, wpts, i + t, wrap=True
        )

        if first_p is None:
        # reacquire 로직: 주행 궤도에서 너무 멀리 떨어졌으면 None
            if nearest_dist < self.max_reacquire:
            # 최근접 투영점을 사용 (좌표는 투영점, 속도는 해당 세그먼트 보간)
                seg_i = min(i, wpts.shape[0]-2)
                v0 = waypoints[seg_i, self.wpt_vind]
                v1 = waypoints[seg_i+1, self.wpt_vind]
                v  = (1.0 - t) * v0 + t * v1
                return np.array([nearest_point[0], nearest_point[1], v], dtype=float)
            else:
                return None

    # (c) 교차점이 있으면 **반드시 그 좌표(first_p)** 를 사용
    #     속도는 해당 세그먼트 i2 ~ i2+1 사이를 **t2로 선형 보간**
        seg_i2 = int(i2) % (waypoints.shape[0]-1)  # wrap 대비
        v0 = waypoints[seg_i2,   self.wpt_vind]
        v1 = waypoints[seg_i2+1, self.wpt_vind]
        v  = (1.0 - t2) * v0 + t2 * v1

        current_waypoint = np.empty((3,), dtype=float)
        current_waypoint[0:2] = first_p  # ★ 핵심: wpts[i2,:]가 아니라 교차점 좌표 사용
        current_waypoint[2]   = v
        return current_waypoint


    def plan(self, pose_x, pose_y, pose_theta, lookahead_distance=None, vgain=None):

        """
        gives actuation given observation
        """

        if not lookahead_distance:
            lookahead_distance = self.lookahead_distance

        if not vgain:
            vgain = self.vgain


        position = np.array([pose_x, pose_y], dtype=float)
        lookahead_point = self._get_current_waypoint(self.waypoints, lookahead_distance, position, pose_theta)

        if lookahead_point is None:
            return self.sprint_speed, 0.0

        speed, steering_angle = get_actuation(pose_theta, lookahead_point, position, lookahead_distance, self.wheelbase)
        speed = vgain * speed

        return speed, steering_angle, lookahead_point
