# run_vals_flat.py
import os, sys, glob, re, argparse, subprocess
import pandas as pd

def parse_viscosity_from_name(name: str):
    """
    Extract viscosity value and a stable text tag from a filename like:
      viscosity_val-0p050_rep-1.svg  -> (0.050, "val-0p050")
    Rule: take the number right before 'p' after 'val-' or 'val_'.
    """
    m = re.search(r'(val[-_]?)(\d+(?:p\d+)?)', name, flags=re.IGNORECASE)
    if not m:
        return None, None
    tag_prefix = m.group(1)              # "val-" or "val_"
    num_token  = m.group(2)              # e.g., "0p050" or "10p000"
    val_tag = f"{tag_prefix}{num_token}" # e.g., "val-0p050"
    visc = float(num_token.replace('p', '.'))
    return visc, val_tag

def parse_rep_from_name(name: str):
    """rep-1 → 1 ; returns None if not found."""
    m = re.search(r'rep[-_]?(\d+)', name, flags=re.IGNORECASE)
    return int(m.group(1)) if m else None

def ensure_dir(p):
    os.makedirs(p, exist_ok=True)
    return p

def main():
    ap = argparse.ArgumentParser(
        description="Average per viscosity from a flat folder of SVGs and build data/avg/* + data/global.csv"
    )
    ap.add_argument("data_root", help="Path to folder containing all SVGs (relative or absolute)")
    ap.add_argument("--averager", default=os.path.join(os.path.dirname(__file__), "average_svg_shape.py"),
                    help="Path to average_svg_shape.py (default: alongside this script)")
    ap.add_argument("--points", type=int, default=512, help="Sample points per file for averaging")
    ap.add_argument("--avg-unit", default="mm", help='Physical unit for per-viscosity output SVGs; use "mm" or empty for none')
    ap.add_argument("--csv-decimal", default='.', help="Decimal separator for CSVs (e.g., ',' for EU style)")
    ap.add_argument("--csv-sep", default=',', help="Field separator for CSVs (e.g., ';' when decimal=',')")
    ap.add_argument("--global-name", default="global.csv", help="Name of the merged CSV written in data_root")
    args = ap.parse_args()

    data_root   = os.path.abspath(args.data_root)
    averager    = os.path.abspath(args.averager)
    avg_dir     = ensure_dir(os.path.join(data_root, "avg"))

    # 1) collect and group files by viscosity tag
    svg_files = sorted(glob.glob(os.path.join(data_root, "*.svg")))
    if not svg_files:
        print(f"No .svg files found in {data_root}")
        return

    groups = {}         # visc_float -> { 'tag': 'val-0p050', 'files': [paths] }
    reps_by_file = {}   # base filename -> rep int
    for p in svg_files:
        base = os.path.basename(p)
        visc, val_tag = parse_viscosity_from_name(base)
        if visc is None:
            print(f"[skip] cannot parse viscosity from: {base}")
            continue
        rep = parse_rep_from_name(base)
        reps_by_file[base] = rep
        g = groups.setdefault(visc, {"tag": val_tag, "files": []})
        g["files"].append(p)

    if not groups:
        print("No files with recognizable 'val-...p...' pattern.")
        return

    global_rows = []

    # 2) process each viscosity group
    for visc, info in sorted(groups.items(), key=lambda kv: kv[0]):
        files = sorted(info["files"])
        val_tag = info["tag"]  # e.g., "val-0p050"

        if len(files) < 2:
            print(f"[warn] {val_tag}: only {len(files)} file(s); averaging anyway")

        # Run the averager directly into data_root/avg and tag all outputs
        cmd = [
            sys.executable, averager,
            "--input", *files,
            "--points", str(args.points),
            "--outdir", avg_dir,
            "--tag", val_tag,
            "--csv-decimal", args.csv_decimal,
            "--csv-sep", args.csv_sep,
        ]
        if args.avg_unit:
            cmd += ["--avg-unit", args.avg_unit]

        print(f"-> {val_tag}: averaging {len(files)} file(s)")
        subprocess.run(cmd, check=True)

        # annotate the per-tag stats with viscosity + rep; file already lives at avg/{val_tag}_shape_stats.csv
        stats_path = os.path.join(avg_dir, f"{val_tag}_shape_stats.csv")
        if not os.path.exists(stats_path):
            print(f"[warn] missing {stats_path}")
            continue

        df = pd.read_csv(stats_path, sep=args.csv_sep, decimal=args.csv_decimal)
        df.insert(0, "val_tag", val_tag)
        df.insert(1, "viscosity", visc)
        df["rep"] = df["file"].map(reps_by_file)  # OVERALL rows stay NaN
        df.to_csv(stats_path, index=False, sep=args.csv_sep, decimal=args.csv_decimal)

        # For GLOBAL CSV: keep ONLY the actual measurements (exclude OVERALL rows)
        df_measures = df[~df["file"].isin(["OVERALL_MEAN", "OVERALL_STDDEV"])].copy()
        global_rows.append(df_measures)

    # 3) write global CSV with ALL measures (every SVG row)
    if global_rows:
        global_csv = os.path.join(data_root, args.global_name)
        pd.concat(global_rows, ignore_index=True).to_csv(global_csv, index=False, sep=args.csv_sep, decimal=args.csv_decimal)
        print(f"Global CSV (all measures) -> {global_csv}")
    else:
        print("No rows collected for global CSV.")

if __name__ == "__main__":
    main()
