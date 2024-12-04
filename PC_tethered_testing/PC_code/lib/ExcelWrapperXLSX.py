import xlsxwriter
import pathlib


import lib.CustomStructs


class ExcelWrapperObj:
    _m_workbook_setup = False
    _m_Workbook: xlsxwriter.Workbook
    _m_name: str
    ExcelSheet: xlsxwriter.worksheet
    current_row = 1

    def create_name(self, _inp_name: str, _inp_path: str = "testlogs/") -> str:
        self._m_name = _inp_path+_inp_name.replace(
            ":", ".").replace("  ", "_").replace(" ", "_")+".xlsx"

        return self._m_name

    def init_spreadsheet_values(self) -> None:

        self._m_Workbook = xlsxwriter.Workbook(self._m_name)
        self.ExcelSheet = self._m_Workbook.add_worksheet()

        self._m_center_alignment = self._m_Workbook.add_format(
            {"align": "center"})
        self._m_orange_color = self._m_Workbook.add_format(
            {"align": "center", "bg_color": "#ff6700"}
        )
        self._m_green_color = self._m_Workbook.add_format(
            {"align": "center", "bg_color": "#00ff00"}
        )

        self.ExcelSheet.write(0, 0, "Time", self._m_center_alignment)
        self.ExcelSheet.write(0, 1, "Target Quaternion",
                              self._m_center_alignment)
        self.ExcelSheet.write(0, 2, "Current Quaternion",
                              self._m_center_alignment)
        self.ExcelSheet.write(0, 3, "Target RPM", self._m_center_alignment)
        self.ExcelSheet.write(0, 4, "Current RPM", self._m_center_alignment)
        self.ExcelSheet.write(0, 5, "Current PWM", self._m_center_alignment)
        self.ExcelSheet.write(0, 6, "Verification", self._m_center_alignment)

        self._m_workbook_setup = True

    def safe_save_to_file(
        self,
    ) -> None:
        pass

    def safe_save_on_exit(self):
        print("\n>>>saving file to drive, this may take a while!<<<")

        if self._m_workbook_setup == True:
            try:
                self.ExcelSheet.autofit()
            except Exception:
                pass
            self._m_Workbook.close()

    def write_DataEntry_to_sheet(
        self,
        _inp_DataEntry: lib.CustomStructs.DataEntry,
        _inp_TestEntry: lib.CustomStructs.TestEntry,
        _inp_time_since_start: float,
    ) -> None:
        self.ExcelSheet.write(
            self.current_row, 0, _inp_time_since_start, self._m_center_alignment
        )
        if _inp_TestEntry.is_quaternion == True:
            self.ExcelSheet.write(
                self.current_row,
                1,
                str(
                    self.round_array(
                        _inp_TestEntry.test_information, self.spreadsheet_decimal_places
                    )
                ),
                self._m_center_alignment,
            )
            self.ExcelSheet.write(
                self.current_row,
                3,
                str("unset"),
                self._m_center_alignment,
            )
        else:
            self.ExcelSheet.write(
                self.current_row,
                1,
                "unset",
                self._m_center_alignment,
            )
            self.ExcelSheet.write(
                self.current_row,
                3,
                str(
                    self.round_array(
                        _inp_TestEntry.test_information, self.spreadsheet_decimal_places
                    )
                ),
                self._m_center_alignment,
            )

        self.ExcelSheet.write(
            self.current_row,
            2,
            str(
                self.round_array(
                    _inp_DataEntry.current_quaternion, self.spreadsheet_decimal_places
                )
            ),
            self._m_center_alignment,
        )

        self.ExcelSheet.write(
            self.current_row,
            4,
            str(
                self.round_array(
                    _inp_DataEntry.current_rpm, self.spreadsheet_decimal_places
                )
            ),
            self._m_center_alignment,
        )
        self.ExcelSheet.write(
            self.current_row,
            5,
            str(
                self.round_array(
                    _inp_DataEntry.current_pwm, self.spreadsheet_decimal_places
                )
            ),
            self._m_center_alignment,
        )

        if (
            (
                _inp_DataEntry.current_quaternion
                == lib.CustomStructs.ReferenceDataEntry.current_quaternion
            )
            or (
                _inp_DataEntry.current_rpm
                == lib.CustomStructs.ReferenceDataEntry.current_rpm
            )
            or (
                _inp_DataEntry.current_pwm
                == lib.CustomStructs.ReferenceDataEntry.current_pwm
            )
        ):
            self.ExcelSheet.write(self.current_row, 6,
                                  "fail", self._m_orange_color)
        else:
            self.ExcelSheet.write(self.current_row, 6,
                                  "pass", self._m_green_color)
        self.current_row += 1

    def round_array(self, _inp_array: list, n_decimals: int) -> list:
        output_array = []
        for i in range(len(_inp_array)):
            output_array.append(round(_inp_array[i], n_decimals))
        return output_array
