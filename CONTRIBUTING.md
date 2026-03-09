# Contributing to Drone Project

Thank you for your interest in contributing! Here's how to get started.

## Development Setup

1. **Clone the repository**:
   ```bash
   git clone https://github.com/harsh-pandhe/drone-project.git
   cd drone-project
   ```

2. **Create a virtual environment** (recommended):
   ```bash
   python3 -m venv venv
   source venv/bin/activate
   ```

3. **Install dependencies**:
   ```bash
   pip install -r requirements.txt
   pip install -e ".[dev]"
   ```

## Project Structure

- **src/** – Main flight control, server, and utility scripts
- **tests/** – Test suites and validation scripts  
- **config/** – Flight controller parameters and telemetry logs
- **data/** – Flight data, telemetry, events, and LiDAR scans
- **models/** – Pre-trained models (e.g. TensorFlow Lite)

## Coding Standards

- Follow **PEP 8** style guidelines
- Use **type hints** where practical
- Format code with **Black**: `black src/ tests/`
- Lint with **Flake8**: `flake8 src/ tests/`

## Testing

Run tests before submitting a PR:

```bash
pytest tests/
pytest --cov=src tests/  # with coverage report
```

## Submitting Changes

1. Create a feature branch: `git checkout -b feature/my-feature`
2. Make your changes and commit with clear messages
3. Push to your fork and open a Pull Request with a description
4. Ensure all tests pass and code is properly formatted

## Reporting Issues

Report bugs or request features via GitHub Issues. Include:
- Clear description of the issue
- Steps to reproduce (for bugs)
- Expected vs. actual behaviour
- Environment details (Python version, OS, hardware)

## Hardware Requirements

This project targets:
- **Pixhawk** flight controller (MAVLink protocol)
- **Raspberry Pi** companion computer (serial connection)
- **Optional**: LiDAR (Livox) for obstacle detection
- **Optional**: Edge TPU for on-device ML inference

## Questions?

Feel free to open an issue with the `question` label or check existing discussions.

Happy flying! 🚁
