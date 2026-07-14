import unittest

from elrstest.crsf import ParameterType, decode_parameter, encode_parameter_value


class ParameterTests(unittest.TestCase):
    def test_selection(self) -> None:
        data = bytes([0, ParameterType.SELECTION]) + b"Packet Rate\x00D50;D100;F100\x00" + bytes([1, 0, 2, 0]) + b"Hz\x00"
        parameter = decode_parameter(3, data)
        self.assertEqual(parameter.name, "Packet Rate")
        self.assertEqual(parameter.options, ("D50", "D100", "F100"))
        self.assertEqual(parameter.value, 1)
        self.assertEqual(parameter.unit, "Hz")
        self.assertEqual(encode_parameter_value(parameter, "F100"), b"\x02")

    def test_edgetx_arrow_bytes(self) -> None:
        data = bytes([0, ParameterType.SELECTION]) + b"Aux\x00AUX1\xc0;AUX1\xc1\x00" + bytes([0, 0, 1, 0]) + b"\x00"
        parameter = decode_parameter(2, data)
        self.assertEqual(parameter.options, ("AUX1+", "AUX1-"))

    def test_signed_int16(self) -> None:
        values = (-12, -100, 100, 0)
        data = bytes([2, ParameterType.INT16]) + b"Offset\x00" + b"".join(value.to_bytes(2, "big", signed=True) for value in values) + b"us\x00"
        parameter = decode_parameter(7, data)
        self.assertEqual((parameter.value, parameter.minimum, parameter.maximum, parameter.default), values)
        self.assertEqual(encode_parameter_value(parameter, "-33"), (-33).to_bytes(2, "big", signed=True))

    def test_float(self) -> None:
        values = (1234, -5000, 5000, 0)
        data = bytes([0, ParameterType.FLOAT]) + b"Gain\x00" + b"".join(value.to_bytes(4, "big", signed=True) for value in values)
        data += bytes([2]) + (25).to_bytes(4, "big") + b"dB\x00"
        parameter = decode_parameter(8, data)
        self.assertEqual(parameter.precision, 2)
        self.assertEqual(parameter.step, 25)
        self.assertEqual(encode_parameter_value(parameter, "-1.25"), (-125).to_bytes(4, "big", signed=True))

    def test_folder_and_command(self) -> None:
        folder = decode_parameter(4, bytes([0, ParameterType.FOLDER]) + b"Other Devices\x00\x01\x02\xff")
        command = decode_parameter(5, bytes([4, ParameterType.COMMAND]) + b"Bind\x00\x03\xc8Confirm bind\x00")
        self.assertEqual(folder.children, (1, 2))
        self.assertEqual(command.command_status, 3)
        self.assertEqual(command.command_timeout, 200)
        self.assertEqual(command.command_info, "Confirm bind")


if __name__ == "__main__":
    unittest.main()
