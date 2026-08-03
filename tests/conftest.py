import os
import sys

# 让 tests/ 能直接 import 仓库根目录的 analyze_ulg.py
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)
