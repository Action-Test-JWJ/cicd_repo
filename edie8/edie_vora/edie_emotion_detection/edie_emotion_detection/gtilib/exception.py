from gtilib.enum import GtiDeviceExceptionCause, GtiModelExceptionCause


class GtiException(RuntimeError):
    pass


class GtiDeviceException(GtiException):

    def __init__(self, cause: GtiDeviceExceptionCause) -> None:
        super().__init__(cause)

    @property
    def cause(self) -> GtiDeviceExceptionCause:
        return self.args[0]


class GtiModelException(GtiException):
    def __init__(self, cause: GtiModelExceptionCause) -> None:
        super().__init__(cause)

    @property
    def cause(self) -> GtiModelExceptionCause:
        return self.args[0]
