import os
import sys
from matplotlib import pyplot as plt
import pandas as pd

path = input("Enter the path to the CSV file: ").strip().strip('"').strip("'")

if not os.path.isfile(path):
    print("File not found:", path)
    print("Tip: do not include surrounding quotes; you can use forward slashes or escape backslashes.")
    sys.exit(1)

# Try common separators if default fails
for sep in [",", ";", "\t"]:
    try:
        df = pd.read_csv(path, sep=sep)
        if df.shape[1] > 1:
            break
    except Exception:
        df = None

if df is None:
    print("Failed to read CSV. Try opening it in a text editor to check delimiter/encoding.")
    sys.exit(1)

print("Loaded file:", path)
print("Columns:", list(df.columns))
print(df.head(5))

# prefer common column sets used by your PID exports
expected_y_sets = [
    ['proportional', 'derivative', 'integral', 'output', 'desiredValue'],
    ['proportional', 'derivative', 'integral', 'output'],
    ['error', 'derivative', 'totalError', 'output'],
    ['error', 'derivative', 'integral', 'output'],
]

ycols = None
for expected in expected_y_sets:
    found = [c for c in expected if c in df.columns]
    if found:
        ycols = found
        break

# fallback: any numeric columns except the first (use first col as x)
if ycols is None:
    numeric_cols = df.select_dtypes(include='number').columns.tolist()
    if len(numeric_cols) <= 1:
        print("No suitable numeric columns found to plot. Available columns:", list(df.columns))
        sys.exit(1)
    xcol = numeric_cols[0]
    ycols = numeric_cols[1:]
else:
    xcol = 'time' if 'time' in df.columns else df.columns[0]

print("Plotting x =", xcol, "y =", ycols)
ax = df.plot(x=xcol, y=ycols, grid=True, figsize=(10,6))
ax.set_xlabel(xcol)
ax.set_ylabel("value")
plt.legend()
plt.tight_layout()
plt.show()