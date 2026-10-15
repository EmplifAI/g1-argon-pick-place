#!/usr/bin/env python3
"""Launch the local console without installing any robot dependencies."""
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
from web_app import main

if __name__ == '__main__':
    main()
