# Unit Tests for SephsBIOME

This `unit_tests` directory contains unit test scripts for the SephsBIOME project, critical for verifying the functionality of individual code segments.

## Current Tests

- `BR_unittest.py`: Contains unit tests for the `bipedal_robot.py` module, ensuring that the bipedal robot functionalities are working as intended.
- `ev_unittest.py`: Contains unit tests for the evolutionary algorithms used in the project to simulate and evaluate adaptive behaviors.

## Need for More Tests

The current test suite is a work in progress, and we require more tests to achieve comprehensive coverage. Contributors are invited to write additional tests, especially for modules that lack them.

### Writing New Tests

To contribute new unit tests, please adhere to the following:

1. Name your test file reflecting the module or feature it tests (e.g., `module_unittest.py`).
2. Use clear and descriptive comments within your test scripts to explain test case functionality.
3. Assert conditions to confirm the code behaves as expected under various scenarios.

### Executing Tests

Execute the unit tests by running:

```bash
python -m unittest BR_unittest.py
python -m unittest ev_unittest.py
```

You can also run all tests in the directory using:

```bash
python -m unittest discover
```

## Call for Contributions

We appeal to all contributors to assist in expanding our unit test coverage. By doing so, you help us maintain the quality and robustness of the SephsBIOME project.

## Support and Collaboration

If you encounter difficulties or require assistance, please open an issue in the repository or reach out to the project maintainers.

Thank you for your valuable contributions to the SephsBIOME project.
