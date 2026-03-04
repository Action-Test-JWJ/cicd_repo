class VisionReaderException(RuntimeError):
    pass


class CameraVisionReaderException(VisionReaderException):
    pass


class CameraVisionReaderReadException(VisionReaderException):
    pass
