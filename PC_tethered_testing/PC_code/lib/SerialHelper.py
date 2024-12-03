import struct
import time
import serial


msg_key = (
    0,
    10,
    11,
    20,
    21,
    30,
    254,
    255,
)
msg_val = (
    "stop",
    "set_quaternion",
    "get_quaternion",
    "set_rpm",
    "get_rpm",
    "get_pwm",
    "reserved",
    "reserved",
)


class SerialHelperObj:
    to_send_msg = b""
    new_to_send_msg = False
    SerialPort: serial.Serial

    def n_floats_to_bytes(input_floats: float) -> str:
        float_data = b""
        for i in range(len(input_floats)):
            float_data += struct.pack("f", input_floats)
        return float_data

    def n_bytes_to_float(input_bytes: str) -> float:
        n_floats = int(len(input_bytes) / 4)
        floats = []
        for i in range(start=0, stop=n_floats, step=4):
            floats.append(struct.unpack("f", input_bytes[i :: i + 3]))
        return floats

    def key_from_val(self, value: str) -> int:
        return msg_key[msg_val.index(value)]

    def block_until_serial_open(
        self,
        _inp_port: str,
        _inp_baudrate: int = 115200,
    ) -> None:
        while True:
            try:
                self.SerialPort = serial.Serial(
                    port=_inp_port,
                    baudrate=_inp_baudrate,
                )
                print(">>>opened mcu serial port<<<")
                return
            except serial.SerialException:
                print(
                    ">>>failed to open serial port to MCU, port:,  ",
                    _inp_port,
                    "retrying in 0.5 seconds<<<",
                )
                time.sleep(0.5)

    def block_until_MCU_initiated(self, _inp_MCU_init_sequence: str) -> None:
        while True:
            if self.SerialPort.in_waiting > 0:
                read_msg = self.SerialPort.readline()
                print(read_msg)
                if _inp_MCU_init_sequence == read_msg:
                    print(">>>communication established<<<")
                    return
                else:
                    print(
                        ">>>failed to establish MCU communication, retrying in 0.5 seconds<<<"
                    )
                    time.sleep(0.5)

    def write_to_serial(self):
        if self.new_to_send_msg:
            self.SerialPort.write(self.to_send_msg)
            self.new_to_send_msg = False
