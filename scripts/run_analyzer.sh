#!/bin/bash
# ASCEND Telemetry Analyzer Launcher
# Quick start script for the flight data visualization dashboard

set -e

echo "🚁 ASCEND Telemetry Analyzer"
echo "=================================="

# Check if Python is installed
if ! command -v python3 &> /dev/null; then
    echo "❌ Python 3 not found! Install Python 3.8+ and try again."
    exit 1
fi

echo "✓ Python found: $(python3 --version)"

# Check if pip is available
if ! command -v pip3 &> /dev/null; then
    echo "❌ pip3 not found! Install Python with pip support."
    exit 1
fi

# Auto-create and activate virtual environment
if [ ! -d "venv" ]; then
    echo ""
    echo "📦 Setting up Python virtual environment..."
    python3 -m venv venv
    if [ ! -d "venv" ]; then
        echo "❌ Failed to create virtual environment"
        exit 1
    fi
    echo "✓ Virtual environment created"
fi

# Activate venv
if [ -f "venv/bin/activate" ]; then
    source venv/bin/activate
    echo "✓ Virtual environment activated"
else
    echo "❌ Could not activate virtual environment"
    exit 1
fi

# Install/update dependencies
echo ""
echo "📚 Installing dependencies..."
pip install --upgrade pip > /dev/null 2>&1
pip install -q streamlit plotly pandas numpy || {
    echo "❌ Failed to install dependencies"
    echo "Try manually: pip install streamlit plotly pandas numpy"
    exit 1
}
echo "✓ Dependencies installed"

# Check for recent log files
echo ""
echo "📁 Checking for flight logs..."
LOG_COUNT=$(find . -name "*flight*log*.csv" -o -name "*telemetry*.csv" 2>/dev/null | wc -l)
if [ "$LOG_COUNT" -gt 0 ]; then
    echo "✓ Found $LOG_COUNT log file(s)"
    echo ""
    echo "📂 Recent logs:"
    find . -name "*flight*log*.csv" -o -name "*telemetry*.csv" 2>/dev/null | sort -r | head -5 | sed 's/^/   /'
else
    echo "⚠️  No log files found in current directory"
    echo "   Run: python src/ascend_blackbox.py   (to generate a log)"
    echo "   Then: $0                             (to analyze it)"
fi

# Launch Streamlit
echo ""
echo "🚀 Launching Telemetry Analyzer..."
echo "   Remote URL: http://localhost:8501"
echo ""
echo "Press Ctrl+C to stop"
echo ""

streamlit run telemetry_analyzer.py --logger.level=warning
