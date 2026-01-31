## 1.0.0 (2026-01-31)

### Features

* initial setup ([cdd0e39](https://github.com/iftahnaf/vctracker/commit/cdd0e3949206888481e3d75799dbcc6552a58afa))
* working build ([ea323af](https://github.com/iftahnaf/vctracker/commit/ea323af3bb90799d55254e022e39d2a66e484c21))

### Bug Fixes

* Add write permissions to semantic-release workflow ([8a6ea51](https://github.com/iftahnaf/vctracker/commit/8a6ea51f3b8a392de67439cbd70fa701c3d99435))
* bug fix across workspace ([435eb2c](https://github.com/iftahnaf/vctracker/commit/435eb2cc436692b5ea0c4776f0dc32d2346c1c4c))
* Update CI workflow to use correct build output paths ([571abda](https://github.com/iftahnaf/vctracker/commit/571abda4c9aa0c5959fabfd56c284441a531d496))
* Upload all build products as artifacts and configure semantic-release ([a289c6b](https://github.com/iftahnaf/vctracker/commit/a289c6b1f054f3cee279266416938b3ff5c3422c))

# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added
- Initial release of vctracker antenna tracker
- GPS module support (NMEA 0183 protocol)
- ROS2 NavSatFix message parsing via UART
- Geodetic calculations for pan/tilt angles
- Status LED indicators (GPS fix, ROS data)
- Automatic tracking with safety limits
- Complete documentation (hardware, setup, ROS2 integration)
- Build and flash scripts
- CI/CD with semantic-release
