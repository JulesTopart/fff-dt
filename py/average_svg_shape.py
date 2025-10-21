#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Average multiple SVG outlines into one profile + stats CSV, with unit-aware SVG metadata.
Writes (in --outdir), using --tag=<name> as a prefix:
  {tag}_shape_stats.csv            # per-file rows + OVERALL_MEAN/STDDEV
  {tag}_average_profile.csv        # per-vertex mean/std
  {tag}_averaged_values.csv        # single-row summary of averaged metrics
  {tag}_average_profile.svg        # averaged curve

Usage:
  pip install numpy pandas svgpathtools
  python average_svg_shape_v2.py --input *.svg --points 512 --outdir ./avg --avg-unit mm --tag val-0p050
"""
import os, re, math, glob, argparse, xml.etree.ElementTree as ET
from typing import List, Tuple, Optional, Dict
import numpy as np, pandas as pd

try:
    from svgpathtools import Path, parse_path, Line
    SVGPATH_AVAILABLE = True
except Exception:
    SVGPATH_AVAILABLE = False

PX_PER_INCH = 96.0
MM_PER_INCH = 25.4
MM_PER_PX = MM_PER_INCH / PX_PER_INCH
PT_PER_INCH = 72.0
MM_PER_PT = MM_PER_INCH / PT_PER_INCH
MM_PER_PC = MM_PER_PT * 12.0

def parse_length_to_mm(s: Optional[str]) -> Tuple[Optional[float], Optional[str]]:
    if s is None:
        return None, None
    s = str(s).strip()
    if s == "":
        return None, None
    m = re.fullmatch(r'([+-]?\d+(?:\.\d+)?)([a-zA-Z%]*)', s)
    if not m:
        return None, None
    val = float(m.group(1))
    unit = m.group(2).lower() or 'px'
    if unit == 'mm': return val, 'mm'
    if unit == 'cm': return val * 10.0, 'cm'
    if unit == 'in': return val * MM_PER_INCH, 'in'
    if unit == 'pt': return val * MM_PER_PT, 'pt'
    if unit == 'pc': return val * MM_PER_PC, 'pc'
    if unit == 'px': return val * MM_PER_PX, 'px'
    return None, unit

def parse_viewbox(s: Optional[str]) -> Optional[Tuple[float,float,float,float]]:
    if not s: return None
    parts = re.split(r'[,\s]+', s.strip())
    if len(parts) != 4: return None
    try:
        return tuple(map(float, parts))  # (x, y, w, h)
    except Exception:
        return None

def _poly_perimeter(points: np.ndarray) -> float:
    diffs = np.diff(np.vstack([points, points[0]]), axis=0)
    return float(np.sum(np.hypot(diffs[:, 0], diffs[:, 1])))

def _poly_area(points: np.ndarray) -> float:
    x = points[:, 0]; y = points[:, 1]
    return float(abs(0.5 * np.sum(x * np.roll(y, -1) - y * np.roll(x, -1))))

def _bbox(points: np.ndarray) -> Tuple[float, float, float, float]:
    return float(points[:,0].min()), float(points[:,1].min()), float(points[:,0].max()), float(points[:,1].max())

def _resample_closed_polyline(points: np.ndarray, n: int) -> np.ndarray:
    if not np.allclose(points[0], points[-1]):
        points = np.vstack([points, points[0]])
    seg_vecs = np.diff(points, axis=0)
    seg_lens = np.hypot(seg_vecs[:,0], seg_vecs[:,1])
    cum = np.concatenate([[0.0], np.cumsum(seg_lens)])
    total = cum[-1]
    if total == 0: return np.tile(points[0], (n, 1))
    targets = np.linspace(0, total, n, endpoint=False)
    out = np.zeros((n, 2), dtype=float); si = 0
    for i, t in enumerate(targets):
        while si < len(seg_lens) - 1 and t > cum[si + 1]:
            si += 1
        seg_start = points[si]; seg_len = seg_lens[si]
        if seg_len == 0:
            out[i] = seg_start; continue
        local = (t - cum[si]) / seg_len
        out[i] = seg_start + local * seg_vecs[si]
    return out

def _procrustes_rotation(X_ref: np.ndarray, Y: np.ndarray) -> np.ndarray:
    A = Y.T @ X_ref
    U, _, Vt = np.linalg.svd(A)
    R = U @ Vt
    if np.linalg.det(R) < 0: U[:, -1] *= -1; R = U @ Vt
    return R

def _best_shift_and_rotation(X_ref: np.ndarray, Y: np.ndarray) -> Tuple[int, np.ndarray, float]:
    n = X_ref.shape[0]
    Xc = X_ref - X_ref.mean(axis=0, keepdims=True)
    best = (0, np.eye(2), np.inf)
    for k in range(n):
        Yk = np.roll(Y, -k, axis=0)
        Yc = Yk - Yk.mean(axis=0, keepdims=True)
        R = _procrustes_rotation(Xc, Yc)
        err = np.mean(np.sum((Yc @ R - Xc)**2, axis=1))
        if err < best[2]: best = (k, R, err)
    return best

def _parse_transform(transform_str: str) -> np.ndarray:
    M = np.eye(3)
    if not transform_str: return M
    for cmd, args in re.findall(r'([a-zA-Z]+)\s*\(([^)]*)\)', transform_str):
        vals = [float(v) for v in re.split(r'[,\s]+', args.strip()) if v]
        cmd = cmd.lower()
        if cmd == 'matrix' and len(vals) == 6:
            a,b,c,d,e,f = vals; T = np.array([[a,c,e],[b,d,f],[0,0,1]], float)
        elif cmd == 'translate':
            tx = vals[0] if vals else 0.0; ty = vals[1] if len(vals)>1 else 0.0
            T = np.array([[1,0,tx],[0,1,ty],[0,0,1]], float)
        elif cmd == 'scale':
            sx = vals[0] if vals else 1.0; sy = vals[1] if len(vals)>1 else sx
            T = np.array([[sx,0,0],[0,sy,0],[0,0,1]], float)
        elif cmd == 'rotate':
            a = math.radians(vals[0] if vals else 0.0); ca, sa = math.cos(a), math.sin(a)
            if len(vals)>=3:
                cx, cy = vals[1], vals[2]
                T = np.array([[ca,-sa,cx - ca*cx + sa*cy],[sa,ca,cy - sa*cx - ca*cy],[0,0,1]], float)
            else:
                T = np.array([[ca,-sa,0],[sa,ca,0],[0,0,1]], float)
        elif cmd == 'skewx':
            a = math.radians(vals[0] if vals else 0.0); T = np.array([[1, math.tan(a), 0],[0,1,0],[0,0,1]], float)
        elif cmd == 'skewy':
            a = math.radians(vals[0] if vals else 0.0); T = np.array([[1,0,0],[math.tan(a),1,0],[0,0,1]], float)
        else:
            continue
        M = M @ T
    return M

def _apply_transform(points: np.ndarray, M: np.ndarray) -> np.ndarray:
    homo = np.hstack([points, np.ones((points.shape[0], 1))])
    out = homo @ M.T
    return out[:, :2]

def _points_from_polygon_points_attr(attr: str) -> np.ndarray:
    nums = [float(v) for v in re.split(r'[,\s]+', attr.strip()) if v]
    return np.array(nums, float).reshape(-1, 2)

def _sample_ellipse(cx: float, cy: float, rx: float, ry: float, n: int = 400) -> np.ndarray:
    t = np.linspace(0, 2*math.pi, n, endpoint=False)
    x = cx + rx * np.cos(t); y = cy + ry * np.sin(t)
    return np.column_stack([x,y])

def _sample_rect(x: float, y: float, w: float, h: float, n: int = 400) -> np.ndarray:
    per = 2*(w+h)
    n_top = max(1, int(round(n*(w/per))))
    n_right = max(1, int(round(n*(h/per))))
    n_bottom = max(1, int(round(n*(w/per))))
    n_left = max(1, int(round(n*(h/per))))
    top = np.column_stack([np.linspace(x, x+w, n_top, endpoint=False), np.full(n_top, y)])
    right = np.column_stack([np.full(n_right, x+w), np.linspace(y, y+h, n_right, endpoint=False)])
    bottom = np.column_stack([np.linspace(x+w, x, n_bottom, endpoint=False), np.full(n_bottom, y+h)])
    left = np.column_stack([np.full(n_left, x), np.linspace(y+h, y, n_left, endpoint=False)])
    return np.vstack([top, right, bottom, left])

def _sample_path_d(d: str, n: int = 400) -> Optional[np.ndarray]:
    """Largest closed subpath sampler (tolerant)."""
    if not SVGPATH_AVAILABLE:
        return None
    path: Path = parse_path(d)
    try:
        subpaths = list(path.continuous_subpaths())
    except AttributeError:
        subpaths = []
        if len(path) == 0: return None
        current = Path(path[0])
        for seg in path[1:]:
            if seg.start != current[-1].end:
                subpaths.append(current); current = Path(seg)
            else:
                current.append(seg)
        subpaths.append(current)
    if not subpaths: return None

    eps = 1e-9
    candidates = []
    for sp in subpaths:
        if len(sp) == 0: continue
        if abs(sp[0].start - sp[-1].end) > eps:
            sp = Path(*sp, Line(sp[-1].end, sp[0].start))
        seg_lengths = np.array([seg.length(error=1e-5) for seg in sp], float)
        total = float(seg_lengths.sum())
        if total == 0.0:
            p = sp[0].start; pts = np.tile([p.real, p.imag], (n, 1))
        else:
            cum = np.concatenate([[0.0], np.cumsum(seg_lengths)])
            targets = np.linspace(0.0, total, n, endpoint=False)
            pts = np.zeros((n, 2), float); si = 0
            for i, s in enumerate(targets):
                while si < len(seg_lengths) - 1 and s > cum[si + 1]:
                    si += 1
                seg = sp[si]; L = seg_lengths[si]
                z = seg.start if L == 0 else seg.point((s - cum[si]) / L)
                pts[i] = [z.real, z.imag]
        if not np.allclose(pts[0], pts[-1], rtol=1e-9, atol=1e-9):
            pts_for_area = np.vstack([pts, pts[0]])
        else:
            pts_for_area = pts
        area = _poly_area(pts_for_area[:-1])
        candidates.append((area, pts))
    if not candidates: return None
    candidates.sort(key=lambda x: x[0], reverse=True)
    return candidates[0][1]

def _collect_shapes_points(elem: ET.Element, M_parent: np.ndarray, results: List[np.ndarray], sample_n: int = 400):
    M_here = M_parent.copy()
    tr = elem.get('transform')
    if tr: M_here = M_here @ _parse_transform(tr)
    tag = elem.tag.split('}')[-1]
    if tag in {'g','svg','symbol'}:
        for child in elem: _collect_shapes_points(child, M_here, results, sample_n)
        return
    if tag == 'path':
        d = elem.get('d')
        if d:
            pts = _sample_path_d(d, n=sample_n)
            if pts is not None: results.append(_apply_transform(pts, M_here))
        return
    if tag in {'polygon','polyline'}:
        pts_attr = elem.get('points')
        if pts_attr:
            pts = _points_from_polygon_points_attr(pts_attr)
            if tag == 'polygon' and not np.allclose(pts[0], pts[-1]):
                pts = np.vstack([pts, pts[0]])
            pts = _resample_closed_polyline(pts, sample_n)
            results.append(_apply_transform(pts, M_here)); return
    if tag == 'circle':
        cx = float(elem.get('cx','0') or 0); cy = float(elem.get('cy','0') or 0); r = float(elem.get('r','0') or 0)
        pts = _sample_ellipse(cx, cy, r, r, n=sample_n)
        results.append(_apply_transform(pts, M_here)); return
    if tag == 'ellipse':
        cx = float(elem.get('cx','0') or 0); cy = float(elem.get('cy','0') or 0)
        rx = float(elem.get('rx','0') or 0); ry = float(elem.get('ry','0') or 0)
        pts = _sample_ellipse(cx, cy, rx, ry, n=sample_n)
        results.append(_apply_transform(pts, M_here)); return
    if tag == 'rect':
        x = float(elem.get('x','0') or 0); y = float(elem.get('y','0') or 0)
        w = float(elem.get('width','0') or 0); h = float(elem.get('height','0') or 0)
        if w>0 and h>0:
            pts = _sample_rect(x,y,w,h,n=sample_n)
            results.append(_apply_transform(pts, M_here)); return

def _largest_closed_contour(points_list: List[np.ndarray]) -> Optional[np.ndarray]:
    if not points_list: return None
    best = None; best_area = -1.0
    for pts in points_list:
        if pts.shape[0] < 3: continue
        pts_c = np.vstack([pts, pts[0]]) if not np.allclose(pts[0], pts[-1]) else pts
        area = _poly_area(pts_c[:-1])
        if area > best_area: best_area = area; best = pts
    return best

def read_svg_outline_and_meta(svg_path: str, sample_n: int = 800):
    try:
        tree = ET.parse(svg_path)
    except ET.ParseError as e:
        raise RuntimeError(f"Failed to parse XML of '{svg_path}': {e}")
    root = tree.getroot()
    vb = parse_viewbox(root.get('viewBox'))
    width_attr = root.get('width'); height_attr = root.get('height')
    width_mm, width_unit = parse_length_to_mm(width_attr)
    height_mm, height_unit = parse_length_to_mm(height_attr)

    pts_list: List[np.ndarray] = []
    _collect_shapes_points(root, np.eye(3), pts_list, sample_n=sample_n)
    if not pts_list:
        raise RuntimeError(f"No supported shapes found in '{svg_path}'. If it only has <path>, ensure 'svgpathtools' is installed.")
    contour = _largest_closed_contour(pts_list)
    if contour is None:
        raise RuntimeError(f"Could not identify a closed contour in '{svg_path}'.")
    contour = _resample_closed_polyline(contour, n=sample_n)

    minx, miny, maxx, maxy = _bbox(contour)
    geom_w, geom_h = (maxx - minx), (maxy - miny)
    margins = {'left': None, 'right': None, 'top': None, 'bottom': None,
               'left_pct_of_max': None, 'right_pct_of_max': None, 'top_pct_of_max': None, 'bottom_pct_of_max': None}
    if vb is not None:
        vx, vy, vw, vh = vb
        margins['left'] = (minx - vx)
        margins['right'] = (vx + vw - maxx)
        margins['top'] = (miny - vy)
        margins['bottom'] = (vy + vh - maxy)
        maxspan = max(geom_w, geom_h) if max(geom_w, geom_h) > 0 else 1.0
        for k in ['left','right','top','bottom']:
            if margins[k] is not None:
                margins[k.replace('', '') + '_pct_of_max'] = 100.0 * margins[k] / maxspan

    mm_per_user = None
    if vb is not None and width_mm is not None and vb[2] != 0.0:
        mm_per_user = width_mm / vb[2]

    meta = {
        'svg_viewBox_x': vb[0] if vb else None,
        'svg_viewBox_y': vb[1] if vb else None,
        'svg_viewBox_w': vb[2] if vb else None,
        'svg_viewBox_h': vb[3] if vb else None,
        'svg_width_attr': width_attr, 'svg_height_attr': height_attr,
        'svg_width_mm': width_mm, 'svg_height_mm': height_mm,
        'mm_per_user_unit': mm_per_user,
        'margin_left': margins['left'], 'margin_right': margins['right'],
        'margin_top': margins['top'], 'margin_bottom': margins['bottom'],
        'margin_left_pct_of_max': margins['left_pct_of_max'],
        'margin_right_pct_of_max': margins['right_pct_of_max'],
        'margin_top_pct_of_max': margins['top_pct_of_max'],
        'margin_bottom_pct_of_max': margins['bottom_pct_of_max'],
        'geom_bbox_xmin': minx, 'geom_bbox_ymin': miny, 'geom_bbox_xmax': maxx, 'geom_bbox_ymax': maxy,
        'geom_width': geom_w, 'geom_height': geom_h
    }
    return contour, meta

def align_and_average(contours: List[np.ndarray]) -> Tuple[List[np.ndarray], np.ndarray, np.ndarray, Dict[str, float]]:
    if len(contours) == 0:
        raise ValueError("No contours provided.")
    per_file_stats = []
    for pts in contours:
        minx, miny, maxx, maxy = _bbox(pts)
        width, height = (maxx - minx), (maxy - miny)
        perim = _poly_perimeter(pts); area = _poly_area(pts); cx, cy = pts.mean(axis=0)
        per_file_stats.append((width, height, perim, area, cx, cy))
    ref = contours[0]
    aligned = [ref.copy()]
    for pts in contours[1:]:
        k, R, _ = _best_shift_and_rotation(ref, pts)
        Yk = np.roll(pts, -k, axis=0)
        Yc = Yk - Yk.mean(axis=0, keepdims=True)
        Yar = Yc @ R + ref.mean(axis=0, keepdims=True)
        aligned.append(Yar)
    stack = np.stack(aligned, axis=0)
    mean_pts = np.mean(stack, axis=0)
    std_pts = np.std(stack, axis=0, ddof=1) if len(contours)>1 else np.zeros_like(mean_pts)
    arr = np.array(per_file_stats)
    summary = {
        'width_mean': float(arr[:, 0].mean()),
        'width_std': float(arr[:, 0].std(ddof=1)) if len(contours)>1 else 0.0,
        'height_mean': float(arr[:, 1].mean()),
        'height_std': float(arr[:, 1].std(ddof=1)) if len(contours)>1 else 0.0,
        'perimeter_mean': float(arr[:, 2].mean()),
        'perimeter_std': float(arr[:, 2].std(ddof=1)) if len(contours)>1 else 0.0,
        'area_mean': float(arr[:, 3].mean()),
        'area_std': float(arr[:, 3].std(ddof=1)) if len(contours)>1 else 0.0,
        'centroid_x_mean': float(arr[:, 4].mean()),
        'centroid_x_std': float(arr[:, 4].std(ddof=1)) if len(contours)>1 else 0.0,
        'centroid_y_mean': float(arr[:, 5].mean()),
        'centroid_y_std': float(arr[:, 5].std(ddof=1)) if len(contours)>1 else 0.0,
    }
    return aligned, mean_pts, std_pts, summary

def write_stats_csv(outdir: str, files: List[str], contours: List[np.ndarray], metas: List[Dict],
                    mean_pts: np.ndarray, std_pts: np.ndarray, overall: Dict[str, float],
                    base: str, csv_decimal: str='.', csv_sep: str=','):
    os.makedirs(outdir, exist_ok=True)

    # Per-file rows
    rows = []
    for fpath, pts, meta in zip(files, contours, metas):
        minx, miny, maxx, maxy = _bbox(pts)
        width, height = (maxx - minx), (maxy - miny)
        perim = _poly_perimeter(pts); area = _poly_area(pts); cx, cy = pts.mean(axis=0)
        row = {
            'file': os.path.basename(fpath),
            'width': width, 'height': height, 'perimeter': perim, 'area': area,
            'centroid_x': cx, 'centroid_y': cy,
            'svg_viewBox_x': meta.get('svg_viewBox_x'), 'svg_viewBox_y': meta.get('svg_viewBox_y'),
            'svg_viewBox_w': meta.get('svg_viewBox_w'), 'svg_viewBox_h': meta.get('svg_viewBox_h'),
            'svg_width_attr': meta.get('svg_width_attr'), 'svg_height_attr': meta.get('svg_height_attr'),
            'svg_width_mm': meta.get('svg_width_mm'), 'svg_height_mm': meta.get('svg_height_mm'),
            'mm_per_user_unit': meta.get('mm_per_user_unit'),
            'geom_bbox_xmin': meta.get('geom_bbox_xmin'), 'geom_bbox_ymin': meta.get('geom_bbox_ymin'),
            'geom_bbox_xmax': meta.get('geom_bbox_xmax'), 'geom_bbox_ymax': meta.get('geom_bbox_ymax'),
            'geom_width': meta.get('geom_width'), 'geom_height': meta.get('geom_height'),
            'margin_left': meta.get('margin_left'), 'margin_right': meta.get('margin_right'),
            'margin_top': meta.get('margin_top'), 'margin_bottom': meta.get('margin_bottom'),
            'margin_left_pct_of_max': meta.get('margin_left_pct_of_max'),
            'margin_right_pct_of_max': meta.get('margin_right_pct_of_max'),
            'margin_top_pct_of_max': meta.get('margin_top_pct_of_max'),
            'margin_bottom_pct_of_max': meta.get('margin_bottom_pct_of_max'),
        }
        rows.append(row)

    df = pd.DataFrame(rows)

    # OVERALL rows from 'overall'
    mean_row = {'file': 'OVERALL_MEAN'}
    std_row  = {'file': 'OVERALL_STDDEV'}
    for k, v in overall.items():
        if k.endswith('_mean'):
            mean_row[k.replace('_mean', '')] = v
        elif k.endswith('_std'):
            std_row[k.replace('_std', '')] = v

    df_all = pd.concat([df, pd.DataFrame([mean_row, std_row])], ignore_index=True)

    stats_csv_path = os.path.join(outdir, f'{base}_shape_stats.csv')
    df_all.to_csv(stats_csv_path, index=False, decimal=csv_decimal, sep=csv_sep)

    # Per-vertex average profile
    idx = np.arange(mean_pts.shape[0])
    cmean = mean_pts.mean(axis=0, keepdims=True)
    r_mean = np.sqrt(np.sum((mean_pts - cmean) ** 2, axis=1))
    if len(contours) > 1:
        radii_stack = np.stack([np.sqrt(np.sum((c - cmean) ** 2, axis=1)) for c in contours], axis=0)
        r_std = radii_stack.std(axis=0, ddof=1)
    else:
        r_std = np.zeros_like(r_mean)

    prof = pd.DataFrame({
        'index': idx,
        'avg_x': mean_pts[:, 0],
        'avg_y': mean_pts[:, 1],
        'std_x': std_pts[:, 0],
        'std_y': std_pts[:, 1],
        'avg_radius_from_centroid': r_mean,
        'std_radius_from_centroid': r_std
    })
    profile_csv_path = os.path.join(outdir, f'{base}_average_profile.csv')
    prof.to_csv(profile_csv_path, index=False, decimal=csv_decimal, sep=csv_sep)

    # Single-row averaged metrics
    avg_only = {
        'n_files': len(contours),
        'width': mean_row.get('width'),
        'height': mean_row.get('height'),
        'perimeter': mean_row.get('perimeter'),
        'area': mean_row.get('area'),
        'centroid_x': mean_row.get('centroid_x'),
        'centroid_y': mean_row.get('centroid_y'),
    }
    averaged_only_csv_path = os.path.join(outdir, f'{base}_averaged_values.csv')
    pd.DataFrame([avg_only]).to_csv(averaged_only_csv_path, index=False, decimal=csv_decimal, sep=csv_sep)

    return stats_csv_path, profile_csv_path, averaged_only_csv_path

def _points_to_svg_path_d(points: np.ndarray, decimals: int = 9) -> str:
    fmt = f"{{:.{decimals}f}}"
    if points.shape[0] == 0:
        return "M 0 0 Z"
    if np.allclose(points[0], points[-1], rtol=1e-9, atol=1e-9):
        pts = points[:-1]
    else:
        pts = points
    cmds = [f"M {fmt.format(pts[0,0])} {fmt.format(pts[0,1])}"]
    for p in pts[1:]:
        cmds.append(f"L {fmt.format(p[0])} {fmt.format(p[1])}")
    cmds.append(f"L {fmt.format(pts[0,0])} {fmt.format(pts[0,1])}")
    cmds.append("Z")
    return " ".join(cmds)

def write_average_svg(outdir: str, mean_pts: np.ndarray, padding: float = 5.0,
                      set_physical_unit: Optional[str] = None, mm_per_user_unit: float = 1.0,
                      filename: str = 'average_profile.svg') -> str:
    os.makedirs(outdir, exist_ok=True)
    minx, miny, maxx, maxy = _bbox(mean_pts)
    width = maxx - minx; height = maxy - miny
    vb_minx = minx - padding; vb_miny = miny - padding
    vb_w = width + 2 * padding; vb_h = height + 2 * padding
    d = _points_to_svg_path_d(mean_pts)
    attrib_wh = ""
    if set_physical_unit and set_physical_unit.lower() == 'mm':
        width_mm = vb_w * mm_per_user_unit
        height_mm = vb_h * mm_per_user_unit
        attrib_wh = f' width="{width_mm:.6f}mm" height="{height_mm:.6f}mm"'
    svg = f"""<?xml version="1.0" encoding="UTF-8" standalone="no"?>
<svg xmlns="http://www.w3.org/2000/svg" version="1.1"{attrib_wh}
     viewBox="{vb_minx:.6f} {vb_miny:.6f} {vb_w:.6f} {vb_h:.6f}">
  <g fill="none" stroke="black" stroke-width="1">
    <path d="{d}" />
  </g>
</svg>
"""
    out_path = os.path.join(outdir, filename)
    with open(out_path, 'w', encoding='utf-8') as f:
        f.write(svg)
    return out_path

def main():
    ap = argparse.ArgumentParser(description="Average SVG outlines + unit-aware stats.")
    ap.add_argument('--input', nargs='+', required=True, help='Input SVG file paths or globs')
    ap.add_argument('--points', type=int, default=512, help='Sample points per file')
    ap.add_argument('--outdir', type=str, default='./avg', help='Output directory (e.g., data/avg)')
    ap.add_argument('--avg-unit', type=str, default=None, help='If "mm", write average SVG with physical size')
    ap.add_argument('--tag', type=str, default="average", help='Name prefix for all outputs (e.g., "val-0p050")')
    ap.add_argument('--csv-decimal', default='.', help="Decimal separator for CSVs")
    ap.add_argument('--csv-sep', default=',', help="Field separator for CSVs")
    args = ap.parse_args()

    # Expand globs
    files: List[str] = []
    for pattern in args.input:
        files.extend(glob.glob(pattern))
    files = sorted(list(dict.fromkeys(files)))
    if not files:
        print("No input SVG files found."); return

    contours = []; metas = []; bad = []
    for f in files:
        try:
            pts, meta = read_svg_outline_and_meta(f, sample_n=args.points)
            contours.append(pts); metas.append(meta)
        except Exception as e:
            bad.append((f, str(e))); print(f"[WARN] {f}: {e}")

    if not contours:
        print("No valid contours extracted."); return

    aligned, mean_pts, std_pts, overall = align_and_average(contours)
    stats_csv, profile_csv, avg_only_csv = write_stats_csv(
        args.outdir, files[:len(aligned)], aligned, metas, mean_pts, std_pts, overall,
        base=args.tag, csv_decimal=args.csv_decimal, csv_sep=args.csv_sep
    )
    mm_per_user = 1.0  # assume your input units are mm already unless known otherwise
    avg_svg = write_average_svg(
        args.outdir, mean_pts,
        set_physical_unit=(args.avg_unit or None),
        mm_per_user_unit=mm_per_user,
        filename=f"{args.tag}_average_profile.svg"
    )

    print("Done.")
    print(f"Stats CSV: {stats_csv}")
    print(f"Average-only CSV: {avg_only_csv}")
    print(f"Average profile CSV: {profile_csv}")
    print(f"Average profile SVG: {avg_svg}")
    if bad:
        print("\nSkipped files:")
        for f, err in bad: print(f" - {f}: {err}")

if __name__ == "__main__":
    main()
