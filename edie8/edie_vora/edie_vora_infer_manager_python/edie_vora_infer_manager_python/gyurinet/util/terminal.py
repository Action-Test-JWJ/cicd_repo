from abc import ABC, abstractmethod


class TerminalControlCodeBase(ABC):
    """
    terminal escape summary(ANSI escape code)
    '\033[' oct terminal Control Sequence Introduce (CSI)
    '\xb1[' hex terminal Control Sequence Introduce (CSI)
    '\u001b[' utf16 hex terminal Control Sequence Introduce (CSI)
    """

    @abstractmethod
    def create(self) -> str:
        pass

    def __str__(self) -> str:
        return self.create()


class TerminalColor8Bit:
    _color_idx: int

    def __init__(self, color_idx: int) -> None:
        super().__init__()
        self._color_idx = color_idx


class TerminalColorClear(TerminalControlCodeBase):

    def create(self) -> str:
        return f'\033[0m'


class TerminalColorRGB(ABC):
    _r: int
    _g: int
    _b: int

    def __init__(self, r: int, g: int, b: int) -> None:
        self._r = r
        self._g = g
        self._b = b


class TerminalFont16ColorsCode(TerminalControlCodeBase, TerminalColor8Bit):
    BLACK = 30
    RED = 31
    GREEN = 32
    YELLOW = 33
    BLUE = 34
    MAGENTA = 35
    CYAN = 36
    WHITE = 37

    BRIGHT_BLACK = 90
    BRIGHT_RED = 91
    BRIGHT_GREEN = 92
    BRIGHT_YELLOW = 93
    BRIGHT_BLUE = 94
    BRIGHT_MAGENTA = 95
    BRIGHT_CYAN = 96
    BRIGHT_WHITE = 97

    def create(self) -> str:
        return f'\033[{self._color_idx}m'


class TerminalForeground16ColorsCode(TerminalControlCodeBase, TerminalColor8Bit):
    BLACK = 40
    RED = 41
    GREEN = 42
    YELLOW = 43
    BLUE = 44
    MAGENTA = 45
    CYAN = 46
    WHITE = 47

    BRIGHT_BLACK = 100
    BRIGHT_RED = 101
    BRIGHT_GREEN = 102
    BRIGHT_YELLOW = 103
    BRIGHT_BLUE = 104
    BRIGHT_MAGENTA = 105
    BRIGHT_CYAN = 106
    BRIGHT_WHITE = 107

    def create(self) -> str:
        return f'\033[{self._color_idx}m'


class TerminalFont8BitColorsCode(TerminalControlCodeBase, TerminalColor8Bit):
    def create(self) -> str:
        return f'\033[38;5;{self._color_idx}m'


class TerminalForeground8BitColorsCode(TerminalControlCodeBase, TerminalColor8Bit):
    def create(self) -> str:
        return f'\033[48;5;{self._color_idx}m'


class TerminalFont24BitColorsCode(TerminalControlCodeBase, TerminalColorRGB):
    def create(self) -> str:
        return f'\033[38;2;{self._r};{self._g};{self._b}m'


class TerminalForeground24BitColorsCode(TerminalColorRGB, TerminalControlCodeBase):

    def create(self) -> str:
        return f'\033[48;2;{self._r};{self._g};{self._b}m'
