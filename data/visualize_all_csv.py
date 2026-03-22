#!/usr/bin/env python3

import re
from pathlib import Path

import matplotlib.pyplot as plt
import pandas as pd

# CSV 기본 디렉토리
processed_dir = Path('/home/hannibal/Jejudol_ws/data')

# -------------------------------------------------------------
# CSV 슬롯: 원하는 만큼 csv1, csv2, csv3 ... 추가해서 사용하세요.
# 상대경로는 processed_dir 기준입니다.
# -------------------------------------------------------------
csv1 = 'Jeju_Map/1_5_map_easy.csv'
csv2 = 'Jeju_Map/1_5_map_middle.csv'
csv3 = 'Jeju_Map/1_5_map_middle2.csv'
csv4 = 'Jeju_Map/1_5_map_hard.csv'
# csv5 = 'Jeju_Map/1_5_map_easy_reverse.csv'
# csv6 = 'Jeju_Map/1_5_map_middle2_reverse.csv'


def _slot_key(name: str) -> int:
    match = re.fullmatch(r'csv(\d+)', name)
    if match is None:
        return 10**9
    return int(match.group(1))


def collect_csv_paths() -> list[Path]:
    csv_paths = []
    for name in sorted(globals().keys(), key=_slot_key):
        if re.fullmatch(r'csv\d+', name) is None:
            continue
        value = globals().get(name)
        if not isinstance(value, str):
            continue
        value = value.strip()
        if not value:
            continue

        path = Path(value).expanduser()
        if not path.is_absolute():
            path = processed_dir / path

        if not path.is_file():
            print(f'[WARN] file not found: {path}')
            continue

        csv_paths.append(path.resolve())
    return csv_paths


def get_xy_columns(df: pd.DataFrame) -> tuple[str, str] | None:
    candidates = [
        ('X(E/m)', 'Y(N/m)'),
        ('UTM_X', 'UTM_Y'),
        ('utm_x', 'utm_y'),
        ('x', 'y'),
        ('X', 'Y'),
    ]
    for x_col, y_col in candidates:
        if x_col in df.columns and y_col in df.columns:
            return x_col, y_col
    return None


def main():
    csv_paths = collect_csv_paths()
    if not csv_paths:
        print('[ERROR] no CSV files to visualize.')
        return

    plt.figure(figsize=(12, 12))
    color_map = plt.cm.get_cmap('tab20', max(len(csv_paths), 1))

    plotted_count = 0
    for idx, csv_path in enumerate(csv_paths):
        try:
            data = pd.read_csv(csv_path)
        except Exception as exc:
            print(f'[WARN] failed to read {csv_path}: {exc}')
            continue

        cols = get_xy_columns(data)
        if cols is None:
            print(f'[WARN] skip {csv_path.name}: XY columns not found')
            continue

        x_col, y_col = cols
        data[x_col] = pd.to_numeric(data[x_col], errors='coerce')
        data[y_col] = pd.to_numeric(data[y_col], errors='coerce')
        data = data.dropna(subset=[x_col, y_col])

        if data.empty:
            print(f'[WARN] skip {csv_path.name}: no valid numeric rows')
            continue

        color = color_map(idx % color_map.N)

        label_name = csv_path.name
        try:
            label_name = str(csv_path.relative_to(processed_dir))
        except ValueError:
            pass

        plt.plot(
            data[x_col].to_numpy(),
            data[y_col].to_numpy(),
            linestyle='-',
            linewidth=2,
            color=color,
            label=label_name,
        )

        # 파일명을 경로 시작점에 텍스트로 표기
        plt.text(
            float(data[x_col].iloc[0]),
            float(data[y_col].iloc[0]),
            label_name,
            fontsize=8,
            color=color,
        )

        # direction 컬럼이 있으면 점 색상으로 함께 표시
        if 'direction' in data.columns:
            colors = {1: 'red', 0: 'black', -1: 'blue'}
            for direction, dot_color in colors.items():
                subset = data[data['direction'] == direction]
                if subset.empty:
                    continue
                plt.scatter(
                    subset[x_col].to_numpy(),
                    subset[y_col].to_numpy(),
                    c=dot_color,
                    s=10,
                    alpha=0.8,
                )

        plotted_count += 1

    if plotted_count == 0:
        print('[ERROR] CSV files were found, but none could be plotted.')
        return

    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('Combined CSV Map Visualization')
    plt.legend()
    plt.grid(True)
    plt.axis('equal')
    plt.tight_layout()
    plt.show()


if __name__ == '__main__':
    main()
