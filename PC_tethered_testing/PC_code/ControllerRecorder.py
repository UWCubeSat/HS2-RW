import time
import pathlib

import random

import lib.SerialHelper as SerialHelper
import lib.ExcelWrapperXLSX as ExcelWrapper
from lib.CustomStructs import TestEntry, DataEntry


test_delay = 1
array_of_tests = (
    TestEntry(1, False, (1, 0, 0, 0), True),
    TestEntry(1, False, (0, 1, 0, 0), True),
    TestEntry(1, False, (500, 500, 500, 500), False),
    TestEntry(1, False, (5, 5, 5, 5), False),
)


class RecorderAndController(ExcelWrapper.ExcelWrapperObj, SerialHelper.SerialHelperObj):
    using_serial_port = False  # during actual operation we will always be using the serial port, so should be true, but for testing this program its useful to be able to run without serial port

    # ---serial config---#
    start_char: str = b"<"
    stop_char: str = b">"
    MCU_init_sequence: str = b"<MCU_init>\n"
    port: str = r"\\dev\\ttyACM0"
    baud_rate: int = 115200

    # ---spreadsheet config---#
    seconds_between_saves = 2
    seconds_between_entries = 0.001

    def __init__(self):
        print(">>>begin setup<<<")
        # super(RecorderAndController, self).__init__()

        # ---testing config---#

        # ---random config---#
        self.spreadsheet_decimal_places = 6
        self.init_time = time.time()

        # ---spreadsheet config---#
        print(
            f">>>saving testdata as: <{self.create_name(
                _inp_name=(r"rws_test_" + str(time.ctime())))}>"
        )

        # ---opening serial port---#
        if self.using_serial_port:
            self.block_until_serial_open(self.port, self.baud_rate)

        # ---checking for mcu connection---#
        if self.using_serial_port:
            self.block_until_MCU_initiated(self.MCU_init_sequence)

        # ---spreadsheet variables and stuff---#
        self.init_spreadsheet_values()

        self.spreadsheet_entry = DataEntry()
        self.last_save_time = self.init_time
        self.last_entry_time = self.init_time

        # ---test indexing stuff---#
        self.test_index = 0
        self.last_test_start = self.init_time
        self.test_setup = False
        self.tests_over = False

        self.init_time = time.time()
        print(">>>init successful<<<")

    def __del__(self):

        self.safe_save_on_exit()

        if self.using_serial_port:
            msg = (
                self.start_char +
                self.key_from_val("stop").to_bytes() + self.stop_char
            )
            self.new_to_send_msg = True
            self.to_send_msg = msg
            self.write_to_serial()

            self.SerialPort.close()

        print(">>>safely saved and exited<<<")

    def mainloop(self):
        self.read_from_serial()
        self.write_to_serial()

        current_time = time.time()

        
        if (current_time - self.last_save_time) > self.seconds_between_saves:
            self.safe_save_to_file()
            self.last_save_time = current_time

        """
        if (current_time - self.last_entry_time) > self.seconds_between_entries:
            self.write_DataEntry_to_sheet(
                self.spreadsheet_entry,
                array_of_tests[self.test_index],
                round(current_time - self.init_time, self.spreadsheet_decimal_places),
            )
            self.last_entry_time = current_time
        """

        # if (current_time - self.last_graph_update_time) > 0.5:
        #    self.update_graph()
        #    self.last_graph_update_time = current_time

        if ((current_time - self.init_time) > test_delay) and (
            self.tests_over == False
        ):
            if (self.test_index == 0) and (self.test_setup == False):
                print(">>>test delay over, starting with test 1<<<")
                self.last_test_start = current_time
            if (array_of_tests[self.test_index].is_indefinite == True) or (
                (current_time - self.last_test_start)
                < array_of_tests[self.test_index].test_duration
            ):
                # print("meow")
                if self.test_setup == False:
                    self.to_send_msg = b""
                    self.new_to_send_msg = False
                    self.test_setup = True

            else:
                # print("nya")
                if self.test_index < len(array_of_tests) - 1:
                    print(
                        f">>>completed test {
                            self.test_index+1}, moving to test {self.test_index+2} (index starts at 1)<<<"
                    )
                    self.test_index += 1
                    self.test_setup = False
                    self.last_test_start = current_time

                else:
                    self.tests_over = True
                    print(">>>completed all tests<<<")

    def read_from_serial(self):
        if self.using_serial_port == True:
            if self.SerialPort.in_waiting > 0:
                read_msg = self.SerialPort.readline()
                msg_identifier = int.from_bytes(read_msg[0])
                msg_data = read_msg[1::]

                match msg_identifier:
                    case self.key_from_val(self, "get_quaternion"):
                        self.spreadsheet_entry.current_quaternion = (
                            self.n_bytes_to_float(msg_data)
                        )

                    case self.key_from_val(self, "get_rpm"):
                        self.spreadsheet_entry.current_rpm = self.n_bytes_to_float(
                            msg_data
                        )

                    case self.key_from_val(self, "get_pwm"):
                        self.spreadsheet_entry.current_quaternion = (
                            self.n_bytes_to_float(msg_data)
                        )

                        self.write_DataEntry_to_sheet(
                            self.spreadsheet_entry,
                            array_of_tests[self.test_index],
                            round(
                                time.time() - self.init_time,
                                self.spreadsheet_decimal_places,
                            ),
                        )
                        self.spreadsheet_entry = DataEntry()

        else:
            self.spreadsheet_entry.current_quaternion = (
                random.uniform(-1, 1),
                random.uniform(-1, 1),
                random.uniform(-1, 1),
                random.uniform(-1, 1),
            )
            self.spreadsheet_entry.current_rpm = (
                random.uniform(-1, 1),
                random.uniform(-1, 1),
                random.uniform(-1, 1),
                random.uniform(-1, 1),
            )
            self.spreadsheet_entry.current_pwm = (
                random.uniform(-1, 1),
                random.uniform(-1, 1),
                random.uniform(-1, 1),
                random.uniform(-1, 1),
            )
            self.write_DataEntry_to_sheet(
                self.spreadsheet_entry,
                array_of_tests[self.test_index],
                round(time.time() - self.init_time,
                      self.spreadsheet_decimal_places),
            )
            self.spreadsheet_entry = DataEntry()
            # time.sleep(0.0005)

    def update_goals(self, new_goal: list[float], is_quaternion: bool):
        self.to_send_msg = self.start_char

        if is_quaternion:
            self.to_send_msg += self.key_from_val("set_quaternion")
        else:
            self.to_send_msg += self.key_from_val("set_rpm")
        self.to_send_msg += self.n_floats_to_bytes(new_goal)

        self.new_to_send_msg = True


def main():
    RecorderAndController_Object = RecorderAndController()

    try:
        while True:
            RecorderAndController_Object.mainloop()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
