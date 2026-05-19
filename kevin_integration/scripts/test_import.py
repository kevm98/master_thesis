import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

print("[OK] Python is running.")
print(f"[INFO] Project root: {ROOT}")

import kevin_integration

print("[OK] kevin_integration imported successfully.")