# 清除 PNG 檔案的腳本
"""
因為輸出圖片太多了，所以寫個腳本來刪除這些 PNG 檔案，保持資料夾整潔。

"""
from pathlib import Path

def clear_png_files(directory: Path) -> int:
    """Delete all .png files in the given directory and return how many were removed."""
    deleted_count = 0
    for png_file in directory.glob("*.png"):
        try:
            png_file.unlink()
            deleted_count += 1
        except OSError as error:
            print(f"無法刪除 {png_file}: {error}")
    return deleted_count


def main() -> None:
    figures_dir = Path(__file__).parent
    print(f"正在清除 {figures_dir} 中的所有 PNG 檔案...")

    deleted = clear_png_files(figures_dir)

    if deleted:
        print(f"已刪除 {deleted} 個 PNG 檔案。")
    else:
        print("此目錄中沒有找到 PNG 檔案。")


if __name__ == "__main__":
    main()
