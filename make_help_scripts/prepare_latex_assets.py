#!/usr/bin/env python3

import argparse
import re
from pathlib import Path

from PIL import Image


GIF_REFERENCE_PATTERN = re.compile(r"\{\{([^}]+)\}\.gif\}")


def convert_gif_references(tex_path: Path) -> int:
    content = tex_path.read_text(encoding="utf-8")
    gif_names = sorted(set(GIF_REFERENCE_PATTERN.findall(content)))

    for gif_name in gif_names:
        gif_path = tex_path.parent / f"{gif_name}.gif"
        png_path = tex_path.parent / f"{gif_name}.png"

        if not gif_path.exists():
            raise FileNotFoundError(f"Missing GIF asset referenced by LaTeX: {gif_path}")

        with Image.open(gif_path) as image:
            image.seek(0)
            image.convert("RGBA").save(png_path)

        content = content.replace(f"{{{{{gif_name}}}.gif}}", f"{{{{{gif_name}}}.png}}")

    tex_path.write_text(content, encoding="utf-8")
    return len(gif_names)


def main() -> None:
    parser = argparse.ArgumentParser(description="Prepare LaTeX assets for PDF builds.")
    parser.add_argument("--latex-dir", required=True, help="Directory containing generated LaTeX files.")
    args = parser.parse_args()

    latex_dir = Path(args.latex_dir)
    converted = 0
    for tex_path in latex_dir.glob("*.tex"):
        converted += convert_gif_references(tex_path)

    print(f"Prepared {converted} GIF asset(s) for LaTeX compilation in {latex_dir}")


if __name__ == "__main__":
    main()