import threading

import carla
import cv2
import numpy as np

from config import (
    RGB_CAMERA_WIDTH,
    RGB_CAMERA_HEIGHT,
    RGB_CAMERA_FOV,
    RGB_CAMERA_X,
    RGB_CAMERA_Z,
    RGB_CAMERA_TICK,
    RGB_CAMERA_GAMMA,
)


class RGBCamera:
    def __init__(self, world, vehicle):
        self.world = world
        self.vehicle = vehicle
        self.sensor = None
        self.latest_frame = None
        self.lock = threading.Lock()

    def start(self):
        bp = self.world.get_blueprint_library().find(
            "sensor.camera.rgb"
        )

        bp.set_attribute(
            "image_size_x",
            str(RGB_CAMERA_WIDTH),
        )
        bp.set_attribute(
            "image_size_y",
            str(RGB_CAMERA_HEIGHT),
        )
        bp.set_attribute(
            "fov",
            str(RGB_CAMERA_FOV),
        )

        if bp.has_attribute("sensor_tick"):
            bp.set_attribute(
                "sensor_tick",
                str(RGB_CAMERA_TICK),
            )

        if bp.has_attribute("gamma"):
            bp.set_attribute(
                "gamma",
                str(RGB_CAMERA_GAMMA),
            )

        if bp.has_attribute(
            "enable_postprocess_effects"
        ):
            bp.set_attribute(
                "enable_postprocess_effects",
                "false",
            )

        transform = carla.Transform(
            carla.Location(
                x=RGB_CAMERA_X,
                y=0.0,
                z=RGB_CAMERA_Z,
            ),
            carla.Rotation(
                pitch=-10.0,
            ),
        )

        self.sensor = self.world.spawn_actor(
            bp,
            transform,
            attach_to=self.vehicle,
            attachment_type=carla.AttachmentType.Rigid,
        )

        self.sensor.listen(self._on_image)

    def _on_image(self, image):
        image.convert(carla.ColorConverter.Raw)

        array = np.frombuffer(
            image.raw_data,
            dtype=np.uint8,
        ).reshape(
            image.height,
            image.width,
            4,
        )

        frame = np.ascontiguousarray(
            array[:, :, :3]
        )

        with self.lock:
            self.latest_frame = frame

    def get_frame(self):
        with self.lock:
            if self.latest_frame is None:
                return None

            return self.latest_frame.copy()

    def destroy(self):
        if self.sensor is not None:
            try:
                self.sensor.stop()
            except RuntimeError:
                pass

            try:
                self.sensor.destroy()
            except RuntimeError:
                pass

        cv2.destroyAllWindows()
