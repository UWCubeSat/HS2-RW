import xlwt
import pathlib

import lib.CustomStructs


class ExcelWrapperObj:
    _m_workbook_setup = False
    _m_Workbook: xlwt.Workbook
    _m_name: str
    ExcelSheet: xlwt.Worksheet
    current_row = 1

    def create_name(self, seedname: str, path: str = "") -> str:
        self._m_name = seedname.replace(":", ".").replace("  ", "_").replace(" ", "_")

        if path != "":
            self._m_name = (
                (str(pathlib.Path(__file__).parent.resolve())).replace(r"\lib", "")
                + r"\\"
                + self._m_name
                + ".xls"
            )

        else:
            self._m_name = path + self._m_name + ".xls"

        return self._m_name

    def init_spreadsheet_values(self) -> None:

        self._m_Workbook = xlwt.Workbook()
        self.ExcelSheet = self._m_Workbook.add_sheet("Sheet 1")

        self._m_color_orange = xlwt.easyxf(
            "pattern: fore_colour orange, pattern solid; alignment: horizontal center;"
        )
        self._m_color_green = xlwt.easyxf(
            "pattern: fore_colour green, pattern solid; alignment: horizontal center;"
        )
        self._m_center_alignment = xlwt.easyxf("alignment: horizontal center;")

        self.ExcelSheet.write(0, 0, "Time", style=self._m_center_alignment)
        self.ExcelSheet.write(0, 1, "Target Quaternion", style=self._m_center_alignment)
        self.ExcelSheet.write(
            0, 2, "Current Quaternion", style=self._m_center_alignment
        )
        self.ExcelSheet.write(0, 3, "Target RPM", style=self._m_center_alignment)
        self.ExcelSheet.write(0, 4, "Current RPM", style=self._m_center_alignment)
        self.ExcelSheet.write(0, 5, "Current PWM", style=self._m_center_alignment)
        self.ExcelSheet.write(0, 6, "Verification", style=self._m_center_alignment)

        for i in range(6):
            self.ExcelSheet.col(i).width = 256 * 25

        self._m_workbook_setup = True

    def safe_save_to_file(
        self,
    ) -> None:
        if self._m_workbook_setup == True:
            self._m_Workbook.save(self._m_name)

    def write_DataEntry_to_sheet(
        self,
        _inp_DataEntry: lib.CustomStructs.DataEntry,
        _inp_TestEntry: lib.CustomStructs.TestEntry,
        _inp_time_since_start: float,
    ) -> None:
        self.ExcelSheet.write(
            r=self.current_row,
            c=0,
            label=_inp_time_since_start,
            style=self._m_center_alignment,
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
                style=self._m_center_alignment,
            )
            self.ExcelSheet.write(
                self.current_row, 3, str("unset"), style=self._m_center_alignment
            )
        else:
            self.ExcelSheet.write(
                self.current_row, 1, "unset", style=self._m_center_alignment
            )
            self.ExcelSheet.write(
                self.current_row,
                3,
                str(
                    self.round_array(
                        _inp_TestEntry.test_information, self.spreadsheet_decimal_places
                    )
                ),
                style=self._m_center_alignment,
            )

        self.ExcelSheet.write(
            self.current_row,
            2,
            str(
                self.round_array(
                    _inp_DataEntry.current_quaternion, self.spreadsheet_decimal_places
                )
            ),
            style=self._m_center_alignment,
        )

        self.ExcelSheet.write(
            self.current_row,
            4,
            str(
                self.round_array(
                    _inp_DataEntry.current_rpm, self.spreadsheet_decimal_places
                )
            ),
            style=self._m_center_alignment,
        )
        self.ExcelSheet.write(
            self.current_row,
            5,
            str(
                self.round_array(
                    _inp_DataEntry.current_pwm, self.spreadsheet_decimal_places
                )
            ),
            style=self._m_center_alignment,
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
            self.ExcelSheet.write(self.current_row, 6, "fail", self._m_color_orange)
        else:
            self.ExcelSheet.write(
                self.current_row, 6, "pass", style=self._m_color_green
            )
        self.current_row += 1

    def round_array(self, _inp_array: list, n_decimals: int) -> list:
        output_array = []
        for i in range(len(_inp_array)):
            output_array.append(round(_inp_array[i], n_decimals))
        return output_array
