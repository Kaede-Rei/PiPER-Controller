#!/usr/bin/env python3
import rospy
import cv2
import numpy as np

from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Float32

from pyorbbecsdk import (
    OBAlignMode,
    OBFormat,
    OBPropertyID,
    OBSensorType,
    Config,
    Pipeline,
    VideoFrame,
)

try:
    from pyorbbecsdk import AlignFilter, OBFrameAggregateOutputMode, OBStreamType
except ImportError:
    AlignFilter = None
    OBFrameAggregateOutputMode = None
    OBStreamType = None


def intrinsic_to_matrix(intrinsic: VideoFrame.Intrinsic) -> np.ndarray:
    return np.array(
        [
            [intrinsic.fx, 0.0, intrinsic.cx],
            [0.0, intrinsic.fy, intrinsic.cy],
            [0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )


def frame_to_bgr_image(frame: VideoFrame) -> np.ndarray:
    width = frame.get_width()
    height = frame.get_height()
    color_format = frame.get_format()
    data = np.asanyarray(frame.get_data())

    if color_format == OBFormat.RGB:
        image = np.frombuffer(frame.get_data(), dtype=np.uint8).reshape(
            height, width, 3
        )
        return cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

    if color_format == OBFormat.YUYV:
        image = np.frombuffer(frame.get_data(), dtype=np.uint8).reshape(
            height, width, 2
        )
        return cv2.cvtColor(image, cv2.COLOR_YUV2BGR_YUYV)

    if color_format == OBFormat.MJPG:
        return cv2.imdecode(data, cv2.IMREAD_COLOR)

    return None


class PiperOrbbec:
    def __init__(self):
        rospy.init_node("piper_orbbec")

        self.bridge = CvBridge()

        self.color_width = rospy.get_param("~color_width", 1280)
        self.color_height = rospy.get_param("~color_height", 720)
        self.color_fps = rospy.get_param("~color_fps", 30)

        self.depth_width = rospy.get_param("~depth_width", 1280)
        self.depth_height = rospy.get_param("~depth_height", 800)
        self.depth_fps = rospy.get_param("~depth_fps", 30)

        self.rotate_180 = rospy.get_param("~rotate_180", False)
        self.min_depth_m = rospy.get_param("~min_depth_m", 0.10)
        self.max_depth_m = rospy.get_param("~max_depth_m", 10.0)
        self.enable_lrm = rospy.get_param("~enable_lrm", True)
        self.frame_id = rospy.get_param("~frame_id", "eef_camera_link")

        self.pipeline = None
        self.device = None
        self.align_filter = None

        self.color_pub = rospy.Publisher(
            "/piper/camera/orbbec/color/image_raw", Image, queue_size=1
        )
        self.depth_pub = rospy.Publisher(
            "/piper/camera/orbbec/depth/image_projection", Image, queue_size=1
        )
        self.info_pub = rospy.Publisher(
            "/piper/camera/orbbec/info", CameraInfo, queue_size=1
        )
        self.lrm_pub = rospy.Publisher(
            "/piper/camera/orbbec/lrm_distance", Float32, queue_size=1
        )

    def _select_color_profile(
        self, profile_list: VideoFrame.StreamProfileList
    ) -> VideoFrame.VideoStreamProfile:
        try:
            return profile_list.get_video_stream_profile(
                self.color_width, self.color_height, OBFormat.RGB, self.color_fps
            )
        except Exception:
            try:
                return profile_list.get_video_stream_profile(
                    self.color_width, self.color_height, OBFormat.MJPG, self.color_fps
                )
            except Exception:
                return profile_list.get_default_video_stream_profile()

    def _select_depth_profile(
        self, profile_list: VideoFrame.StreamProfileList
    ) -> VideoFrame.VideoStreamProfile:
        try:
            return profile_list.get_video_stream_profile(
                self.depth_width, self.depth_height, OBFormat.Y16, self.depth_fps
            )
        except Exception:
            return profile_list.get_default_video_stream_profile()

    def _set_disparity_search_range_256(self) -> bool:
        if self.device is None:
            return False

        prop = getattr(OBPropertyID, "OB_PROP_DISP_SEARCH_RANGE_MODE_INT", None)
        if prop is None:
            rospy.logwarn("pyorbbecsdk 未暴露 OB_PROP_DISP_SEARCH_RANGE_MODE_INT")
            return False

        try:
            self.device.set_int_property(prop, 2)
            applied = self.device.get_int_property(prop)
            rospy.loginfo("视差搜索范围模式值: %s", applied)
            return applied == 2
        except Exception as e:
            rospy.logwarn("视差搜索范围设置失败: %s", e)
            return False

    def _build_depth_m(self, depth_frame: VideoFrame) -> np.ndarray:
        depth_u16 = np.frombuffer(depth_frame.get_data(), dtype=np.uint16).reshape(
            (depth_frame.get_height(), depth_frame.get_width())
        )
        depth_u16 = cv2.medianBlur(depth_u16, 5)
        depth_m = (
            depth_u16.astype(np.float32) * float(depth_frame.get_depth_scale()) / 1000.0
        )
        depth_m = np.where(
            (depth_m > self.min_depth_m) & (depth_m < self.max_depth_m),
            depth_m,
            0.0,
        ).astype(np.float32)
        return depth_m

    def _build_camera_info(
        self, width: int, height: int, K: np.ndarray, stamp: rospy.Time
    ) -> CameraInfo:
        msg = CameraInfo()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        msg.width = width
        msg.height = height
        msg.distortion_model = "plumb_bob"
        msg.D = []
        msg.K = [
            K[0, 0],
            0.0,
            K[0, 2],
            0.0,
            K[1, 1],
            K[1, 2],
            0.0,
            0.0,
            1.0,
        ]
        msg.R = [
            1.0,
            0.0,
            0.0,
            0.0,
            1.0,
            0.0,
            0.0,
            0.0,
            1.0,
        ]
        msg.P = [
            K[0, 0],
            0.0,
            K[0, 2],
            0.0,
            0.0,
            K[1, 1],
            K[1, 2],
            0.0,
            0.0,
            0.0,
            1.0,
            0.0,
        ]
        return msg

    def setup(self):
        self.pipeline = Pipeline()
        self.device = self.pipeline.get_device()
        config = Config()

        color_profile_list = self.pipeline.get_stream_profile_list(
            OBSensorType.COLOR_SENSOR
        )
        depth_profile_list = self.pipeline.get_stream_profile_list(
            OBSensorType.DEPTH_SENSOR
        )

        color_profile = self._select_color_profile(color_profile_list)
        depth_profile = self._select_depth_profile(depth_profile_list)

        config.enable_stream(color_profile)
        config.enable_stream(depth_profile)

        if hasattr(config, "set_align_mode"):
            try:
                config.set_align_mode(OBAlignMode.SW_MODE)
            except Exception:
                pass

        if (
            hasattr(config, "set_frame_aggregate_output_mode")
            and OBFrameAggregateOutputMode is not None
            and hasattr(OBFrameAggregateOutputMode, "FULL_FRAME_REQUIRE")
        ):
            try:
                config.set_frame_aggregate_output_mode(
                    OBFrameAggregateOutputMode.FULL_FRAME_REQUIRE
                )
            except Exception:
                pass

        self.pipeline.enable_frame_sync()
        self.pipeline.start(config)

        if AlignFilter is not None and OBStreamType is not None:
            try:
                self.align_filter = AlignFilter(
                    align_to_stream=OBStreamType.COLOR_STREAM
                )
                rospy.loginfo("已启用 AlignFilter: Depth -> Color")
            except Exception as e:
                self.align_filter = None
                rospy.logwarn("AlignFilter 初始化失败: %s", e)
        else:
            rospy.logwarn("当前 pyorbbecsdk 未暴露 AlignFilter，将回退到原始 depth")

        disparity_ok = self._set_disparity_search_range_256()

        if self.enable_lrm:
            try:
                self.device.set_bool_property(OBPropertyID.OB_PROP_LDP_BOOL, True)
                rospy.loginfo("LRM 激光补盲模块已开启")
            except Exception as e:
                rospy.logwarn("LRM 开启失败: %s", e)

        rospy.loginfo("相机启动成功, disp256=%s", "OK" if disparity_ok else "NO")

    def spin(self):
        rate = rospy.Rate(self.color_fps)

        while not rospy.is_shutdown():
            frames = self.pipeline.wait_for_frames(100)
            if frames is None:
                rate.sleep()
                continue

            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()
            if color_frame is None or depth_frame is None:
                rate.sleep()
                continue

            aligned_depth_frame = None
            if self.align_filter is not None:
                try:
                    aligned_frames = self.align_filter.process(frames)
                    if aligned_frames is not None:
                        aligned_depth_frame = (
                            aligned_frames.as_frame_set().get_depth_frame()
                        )
                except Exception as e:
                    rospy.logwarn_throttle(2.0, "AlignFilter 处理失败: %s", e)
                    aligned_depth_frame = None

            color_image = frame_to_bgr_image(color_frame)
            if color_image is None:
                rate.sleep()
                continue

            if self.rotate_180:
                color_image = cv2.rotate(color_image, cv2.ROTATE_180)

            raw_depth_m = self._build_depth_m(depth_frame)
            aligned_depth_m = None
            if aligned_depth_frame is not None:
                try:
                    aligned_depth_m = self._build_depth_m(aligned_depth_frame)
                except Exception as e:
                    rospy.logwarn_throttle(2.0, "对齐深度解析失败: %s", e)

            color_video_profile = (
                color_frame.get_stream_profile().as_video_stream_profile()
            )
            depth_video_profile = (
                depth_frame.get_stream_profile().as_video_stream_profile()
            )
            K_color = intrinsic_to_matrix(color_video_profile.get_intrinsic())
            K_depth = intrinsic_to_matrix(depth_video_profile.get_intrinsic())

            projection_depth_m = None
            projection_K = None

            if aligned_depth_m is not None and aligned_depth_m.shape[:2] == (
                color_frame.get_height(),
                color_frame.get_width(),
            ):
                projection_depth_m = aligned_depth_m
                projection_K = K_color
            elif raw_depth_m.shape[:2] == (
                color_frame.get_height(),
                color_frame.get_width(),
            ):
                projection_depth_m = raw_depth_m
                projection_K = K_color
            else:
                projection_depth_m = raw_depth_m
                projection_K = K_depth

            if self.rotate_180:
                projection_depth_m = cv2.rotate(projection_depth_m, cv2.ROTATE_180)

            now = rospy.Time.now()

            color_msg = self.bridge.cv2_to_imgmsg(color_image, encoding="bgr8")
            color_msg.header.stamp = now
            color_msg.header.frame_id = self.frame_id

            depth_msg = self.bridge.cv2_to_imgmsg(projection_depth_m, encoding="32FC1")
            depth_msg.header.stamp = now
            depth_msg.header.frame_id = self.frame_id

            info_msg = self._build_camera_info(
                projection_depth_m.shape[1],
                projection_depth_m.shape[0],
                projection_K,
                now,
            )

            self.color_pub.publish(color_msg)
            self.depth_pub.publish(depth_msg)
            self.info_pub.publish(info_msg)

            if self.enable_lrm:
                try:
                    lrm_dist_mm = self.device.get_int_property(
                        OBPropertyID.OB_PROP_LDP_MEASURE_DISTANCE_INT
                    )
                    self.lrm_pub.publish(Float32(data=lrm_dist_mm / 1000.0))
                except Exception:
                    pass

            rate.sleep()


if __name__ == "__main__":
    node = PiperOrbbec()
    node.setup()
    node.spin()
