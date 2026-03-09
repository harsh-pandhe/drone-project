.PHONY: help install install-dev test lint format clean run-server run-tui docs

help:
	@echo "Drone Project - Available Commands"
	@echo "===================================="
	@echo "make install       - Install production dependencies"
	@echo "make install-dev   - Install with dev dependencies"
	@echo "make test          - Run test suite"
	@echo "make test-cov      - Run tests with coverage report"
	@echo "make lint          - Run flake8 linter"
	@echo "make format        - Auto-format code with Black"
	@echo "make format-check  - Check code formatting without changes"
	@echo "make clean         - Remove caches and build artifacts"
	@echo "make run-server    - Start Flask web server (port 5000)"
	@echo "make run-tui       - Start terminal UI"
	@echo "make check-conn    - Verify Pixhawk connection"
	@echo "make docs          - Open README and documentation"

install:
	pip install -r requirements.txt

install-dev:
	pip install -r requirements.txt -e ".[dev]"

test:
	pytest tests/ -v

test-cov:
	pytest tests/ --cov=src --cov-report=html
	@echo "Coverage report generated in htmlcov/index.html"

lint:
	flake8 src/ tests/ --max-line-length=100 --ignore=E501,W503

format:
	black src/ tests/ --line-length=100

format-check:
	black src/ tests/ --line-length=100 --check

clean:
	find . -type d -name __pycache__ -exec rm -rf {} + 2>/dev/null || true
	find . -type f -name "*.pyc" -delete
	rm -rf .pytest_cache .coverage htmlcov dist build *.egg-info
	@echo "Clean complete"

run-server:
	python3 src/ascend_server.py

run-tui:
	python3 src/ascend_tui.py

check-conn:
	python3 src/check_connection.py

docs:
	@echo "Documentation files:"
	@echo "- README.md (this directory)"
	@echo "- SETUP.md (hardware and software setup)"
	@echo "- CONTRIBUTING.md (development guidelines)"

.DEFAULT_GOAL := help
