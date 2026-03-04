class BoundingBox:
    x1: float
    x2: float
    y1: float
    y2: float

    score: float
    label: int

    def __init__(self, x1: float, y1: float, x2: float, y2: float, score: float, label: int) -> None:
        self.x1 = x1
        self.y1 = y1
        self.x2 = x2
        self.y2 = y2
        self.score = score
        self.label = label

        if x2 < x1 or y2 < y1:
            self.size = 0
        else:
            width: float = x2 - x1 + 1
            height: float = y2 - y1 + 1
            self.size = width * height


class IntersectBoundingBox:
    x_min: float
    x_max: float
    y_min: float
    y_max: float

    _width: float
    _height: float
    _size: float

    def __init__(self, inferred_bounding_box: BoundingBox, bounding_box: BoundingBox) -> None:
        if bounding_box.x1 > inferred_bounding_box.x2 or bounding_box.x2 < inferred_bounding_box.x1 or \
                bounding_box.y1 > inferred_bounding_box.y2 or bounding_box.y2 < inferred_bounding_box.y1:
            self.x_min = .0
            self.y_min = .0
            self.x_max = .0
            self.y_max = .0

            self._width = 0
            self._height = 0
            self._size = 0
        else:
            self.x_min = max(inferred_bounding_box.x1, bounding_box.x1)
            self.y_min = max(inferred_bounding_box.y1, bounding_box.y1)
            self.x_max = min(inferred_bounding_box.x2, bounding_box.x2)
            self.y_max = min(inferred_bounding_box.y2, bounding_box.y2)

            self._width = self.x_max - self.x_min + 1
            self._height = self.y_max - self.y_min + 1
            self._size = self._width * self._height

    @property
    def width(self) -> float:
        return self._width

    @property
    def height(self) -> float:
        return self._height

    def size(self) -> float:
        return self._size
