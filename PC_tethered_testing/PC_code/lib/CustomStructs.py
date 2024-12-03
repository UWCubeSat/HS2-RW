class DataEntry:
    target_quaternion: list[float] = [0, 0, 0, 0]
    current_quaternion: list[float] = [-1, -1, -1, -1]
    target_rpm: list[float] = [0, 0, 0, 0]
    current_rpm: list[float] = [-1, -1, -1, -1]
    current_pwm: list[int] = [500, 500, 500, 500]


class ReferenceDataEntry:
    current_quaternion = (-1, -1, -1, -1)
    current_rpm = (-1, -1, -1, -1)
    current_pwm = (500, 500, 500, 500)


class TestEntry:
    test_duration: float
    is_indefinite: bool

    test_information: tuple[float]
    is_quaternion: bool

    def __init__(
        self,
        _inp_test_duration,
        _inp_is_indefinite,
        _inp_test_information,
        _inp_is_quaternion,
    ):
        self.test_duration = _inp_test_duration
        self.is_indefinite = _inp_is_indefinite
        self.test_information = _inp_test_information
        self.is_quaternion = _inp_is_quaternion
