# SPDX-FileCopyrightText: 2024-2025 Espressif Systems (Shanghai) CO LTD
# SPDX-License-Identifier: CC0-1.0
import pytest
from pytest_embedded import Dut
from pytest_embedded_idf.utils import idf_parametrize


@pytest.mark.usb_serial_jtag
@pytest.mark.parametrize(
    'port, flash_port, config',
    [
        pytest.param('/dev/serial_ports/ttyACM-esp32', '/dev/serial_ports/ttyUSB-esp32', 'release'),
    ],
    indirect=True,
)
@pytest.mark.parametrize('test_message', ['test123456789!@#%^&*'])
@idf_parametrize('target', ['esp32s3', 'esp32c3', 'esp32c6', 'esp32h2'], indirect=['target'])
def test_usj_vfs_release(dut: Dut, test_message: list) -> None:
    dut.expect_exact('Press ENTER to see the list of tests')
    dut.write('"test select read, write and timeout"')
    dut.expect_exact('select timed out', timeout=2)
    dut.write(test_message)
    dut.expect_exact(test_message, timeout=2)
    dut.expect(r'\d{1} Tests 0 Failures 0 Ignored', timeout=10)
