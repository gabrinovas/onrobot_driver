# Copyright 2024 Your Name
# SPDX-License-Identifier: Apache-2.0

import os
from ament_copyright.main import main
import pytest


@pytest.mark.copyright
@pytest.mark.linter
def test_copyright():
    # Test that all source files have copyright headers
    file_paths = []
    for root, dirs, files in os.walk('src'):
        for file in files:
            if file.endswith('.py'):
                file_paths.append(os.path.join(root, file))
    
    # This will check copyright headers in all Python files
    ret = main(argv=['--exclude', 'test', '.'])
    assert ret == 0, 'Found files without copyright headers'