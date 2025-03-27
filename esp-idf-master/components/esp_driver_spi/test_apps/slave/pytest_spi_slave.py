# SPDX-FileCopyrightText: 2021-2025 Espressif Systems (Shanghai) CO LTD
# SPDX-License-Identifier: Apache-2.0
import pytest
from pytest_embedded_idf.utils import idf_parametrize
# If `test_env` is define, should not run on generic runner


@pytest.mark.generic
@pytest.mark.parametrize('config', ['release', 'iram_safe'], indirect=True)
@idf_parametrize('target', ['supported_targets'], indirect=['target'])
def test_slave_single_dev(case_tester) -> None:  # type: ignore
    for case in case_tester.test_menu:
        if 'test_env' in case.attributes:
            continue
        case_tester.run_normal_case(case=case, reset=True)


# if `test_env` not defined, will run on `generic_multi_device` by default
# TODO: [ESP32C61] IDF-10949
@pytest.mark.temp_skip_ci(targets=['esp32c61'], reason='no multi-dev runner')
@pytest.mark.generic_multi_device
@pytest.mark.parametrize('count, config', [(2, 'release'), (2, 'iram_safe')], indirect=True)
@idf_parametrize('target', ['supported_targets'], indirect=['target'])
def test_slave_multi_dev(case_tester) -> None:  # type: ignore
    for case in case_tester.test_menu:
        if case.attributes.get('test_env', 'generic_multi_device') == 'generic_multi_device':
            case_tester.run_multi_dev_case(case=case, reset=True)
